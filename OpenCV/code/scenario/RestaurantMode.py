import random, time
from typing import Dict, List, Optional, Set, Tuple
import json
from pathlib import Path
import os
import numpy as np

import grid

Cell = Tuple[int, int]


class RestaurantMode:
    name = "RestaurantMode"
    """
    레스토랑 운영 흐름:
      • enter(): 전체 초기화 → (필요 시) 중앙>방향 정렬을 보내 '명령 수신 준비' 상태로 맞춤
                 → 각 로봇 goal=HOME 지정 → 도착 시 중앙>방향 정렬 후 HOME 상태 진입
      • 주문 생성: 0~30초 사이에 1개씩, 장애물 옆 4칸 중에서 중복 목적지는 제외
        - order_new: UI 표시용 휘발 버퍼
        - order_list: (rid, dst) 큐 (로봇별 최대 2개)
        - order_start: 현재 수행중인 (rid, dst) 집합
        - order_done: 완료된 (rid, dst) 리스트(로그)
      • HOME 상태 로봇의 번호키 입력: 해당 rid의 주문이 order_list에 있으면
        맨 앞 하나를 꺼내 수행 시작(=order_start에 넣고 goal 지정).
      • 목적지 도착하면 order_done에 기록 + order_list에서 같은 명령 중 '가장 위' 1개 제거
        → 자동으로 HOME 복귀 명령 생성 → HOME 도착 후 정렬 완료 시 다시 HOME 상태 진입
    """

    # ---- Base utilities (ScenarioManager/BaseMode가 제공한다고 가정) ----
    # ensure_agent_ctx(self, ctx, rid) -> dict
    # occupied_from_tags(self, tag_info) -> Set[Cell]
    # result(self, *, replan=False, waiters=None, waiter_cells=None,
    #         align_center=None, align_direction=None, ready=None, reason="") -> dict
    # 위 유틸은 RandomMode 설계와 동일한 인터페이스로 사용.

    def __init__(self, manager = None, *, home_provider=None, order_span_sec=(0, 30)):
        """
        manager: ScenarioManager 인스턴스 (push_order_history 호출용)
        home_provider: rid -> home Cell 반환 함수
        """
        self.manager = manager                  # ⭐ ScenarioManager 주입 (history용)
        self.home_provider = home_provider
        self.span_min, self.span_max = order_span_sec

        self._cfg_path = Path(__file__).parent / "table_coords.json"
        self._cfg_mtime: float | None = None
        self.table_chairs: List[Cell] = []

        self._reload_table_config(force=True)

        # 주문 상태
        self.order_new: Optional[Tuple[int, Cell]] = None
        self.order_list: List[Tuple[int, Cell]] = []
        self.order_start: Set[Tuple[int, Cell]] = set()
        self.order_done: List[Tuple[int, Cell]] = []
        self.order_to_table: Set[Tuple[int, Cell]] = set()
        self.order_to_home: Set[Tuple[int, Cell]] = set()

        # HOME 상태 관리
        self.home_set: Set[int] = set()

        # 주문 타이머
        self._next_order_at: float = time.time() + self._rnd_span()

        # 현재 프레임에서 받은 agents 리스트를 보관 ( _agent_by_id 용 )
        self.agents: List = []

    # ------------------------------ Lifecycle ------------------------------
    def enter(self, *, tag_info: dict, grid: np.ndarray,
              agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # 상태 리셋 및 HOME 세팅
        self.agents = agents
        self.order_new = None
        self.order_list.clear()
        self.order_start.clear()
        self.order_done.clear()
        self.home_set.clear()
        self.order_to_table.clear()
        self.order_to_home.clear()
        self._next_order_at = time.time() + self._rnd_span()

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            home_from_main = self.home_provider(a.id) if self.home_provider else None
            s["home"] = tuple(home_from_main) if home_from_main is not None else None
            s["homeless"] = (home_from_main is None)
            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["last_pos"] = tuple(a.start) if a.start else None
            s["idle_frames"] = 0
            if s.get("home") and a.start and tuple(a.start) == tuple(s["home"]):
                self.home_set.add(a.id)

        # 초기 정렬 대상
        initial_align_ids = [a.id for a in agents if a.start]
        ctx["_init_align_pending"] = set(initial_align_ids)

        # HOME이 아닌 로봇들은 HOME으로 goal 설정
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not a.start or not s.get("home"):
                continue
            if tuple(a.start) != tuple(s["home"]):
                a.goal = tuple(s["home"])

        # 시작 시점에 기본 주문들을 Seed
        self._seed_initial_orders(grid, agents, ctx, tag_info)

        # 초기에는 정렬만 요청 (replan은 바로 안 함)
        return self.result(
            replan=False,
            align_center=set(initial_align_ids),
            align_direction=set(initial_align_ids),
            reason="enter_init_align_only"
        )

    # ------------------------------ Helpers ------------------------------
    def _rnd_span(self) -> float:
        return random.uniform(self.span_min, self.span_max)

    def _neighbors4(self, r: int, c: int, H: int, W: int):
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W:
                yield rr, cc

    def _pick_from_table_meta(self, grid: np.ndarray,
                              forbidden: Set[Cell]) -> Optional[Cell]:
        """
        table_coords.json 에서 불러온 self.table_chairs들 중
        - grid 범위 안
        - 통로(grid==0)
        - forbidden/taken 에 없는 좌표만 후보로 두고 랜덤 선택
        """
        if not self.table_chairs:
            return None

        H, W = grid.shape
        fbd = set(forbidden)

        # 이미 사용 중인 목적지 (현재 큐 + 실행 중)
        taken_dsts = {dst for _, dst in self.order_list} | {
            dst for _, dst in self.order_start
        }
        fbd |= taken_dsts

        candidates: List[Cell] = []
        for (r, c) in self.table_chairs:
            if not self._in_bounds(r, c, H, W):
                continue
            if grid[r, c] != 0:
                continue
            if (r, c) in fbd:
                continue
            candidates.append((r, c))

        if not candidates:
            return None
        return random.choice(candidates)

    def _home_of(self, ctx: Dict[int, dict], rid: int) -> Optional[Cell]:
        h = self.ensure_agent_ctx(ctx, rid).get("home")
        return tuple(h) if h else None

    def _dir_vec(self, d: str):
        # 행(r)↓, 열(c)→ 기준
        return {"n": (-1, 0), "e": (0, 1), "s": (1, 0), "w": (0, -1)}.get(
            d.lower(), (0, 0)
        )

    def _in_bounds(self, r, c, H, W):
        return 0 <= r < H and 0 <= c < W

    # ------------------------------ Ticking ------------------------------
    def tick(self, *, tag_info: dict, grid: np.ndarray,
             agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """주문 생성, HOME 상태 유지, 필요 시 CBS 트리거."""
        replan = False
        now = time.time()

        # 매 프레임 받은 agents 리스트 보관 (history·헬퍼 함수에서 사용)
        self.agents = agents

        self._reload_table_config(force=False)

        # 1) 신규 주문 생성 타이밍이면 한 개 만든다
        if now >= self._next_order_at:
            self._next_order_at = now + self._rnd_span()

            # 목적지 후보 생성(중복 제외)
            occ = self.occupied_from_tags(tag_info)
            starts = {tuple(a.start) for a in agents if a.start}
            goals = {tuple(a.goal) for a in agents if a.goal}
            homes = {
                tuple(self._home_of(ctx, a.id))
                for a in agents
                if self._home_of(ctx, a.id)
            }
            forbidden = set(starts) | set(goals) | set(occ) | set(homes)
            taken_dsts = {dst for _, dst in self.order_list} | {
                dst for _, dst in self.order_start
            }
            dst = self._pick_from_table_meta(grid, forbidden | taken_dsts)

            # 대상 로봇은 (보이는 로봇 중) 랜덤, 로봇별 최대 2개
            visible = [a.id for a in agents if a.start]
            random.shuffle(visible)
            rid = None
            if dst is not None:
                for cand in visible:
                    cnt = sum(1 for r, _ in self.order_list if r == cand)
                    if cnt < 2:
                        rid = cand
                        break

            if rid is not None and dst is not None:
                self.order_new = (rid, dst)        # UI용 휘발 버퍼
                self.order_list.append((rid, dst)) # 큐 뒤에 추가

                # ⭐ 새로 생성된 주문도 히스토리에 남기고 싶다면 여기에서:
                if self.manager is not None:
                    self.manager.push_order_history(
                        rid=rid,
                        order_id=(rid, dst),
                        goal=dst,
                        status="ORDER_CREATED",
                    )
            else:
                self.order_new = None  # 생성 실패

        # 2) HOME 판정 및 유지 (HOME 벗어나면 home_set에서 제거)
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not s.get("home") or not a.start:
                continue
            if tuple(a.start) != tuple(s["home"]):
                self.home_set.discard(a.id)

        # 별도 CBS 트리거 없음 (번호키 on_home_key 호출에서 CBS 요청)
        return None

    # ------------------------------ 외부 이벤트 ------------------------------
    def on_home_key(self, rid: int, *, agents: List, ctx: Dict[int, dict]):
        """
        메인에서 번호키가 눌렸을 때 호출.
        HOME 상태가 아니면 무시.
        HOME이고 order_list에 내 주문이 있으면 '맨 앞' 하나를 꺼내서 수행 시작.
        """
        if rid not in self.home_set:
            return None

        # 맨 앞에서 해당 rid 주문 하나 찾기
        idx = next(
            (i for i, (r, _) in enumerate(self.order_list) if r == rid), None
        )
        if idx is None:
            return None

        (r, dst) = self.order_list.pop(idx)
        self.order_start.add((r, dst))
        self.order_to_table.add((r, dst))
        self.home_set.discard(rid)

        # 해당 로봇 goal 설정 → CBS 재계획
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        a.goal = tuple(dst)

        # ⭐ 주문 수행 시작 시 히스토리 기록
        if self.manager is not None:
            self.manager.push_order_history(
                rid=rid,
                order_id=(rid, dst),
                goal=dst,
                status="ORDER_ACCEPTED",
            )

        return self.result(replan=True, reason="order_start")

    # ------------------------------ 콜백 ------------------------------
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray,
                             agents: List, ctx: Dict[int, dict],
                             runstate: Dict[int, dict]):
        """
        컨트롤러 전체 시퀀스(여러 로봇의 한 번 이동 블럭)가 끝났을 때 호출.
        여기서는 '로봇 하나의 진짜 도착'보다는,
        도중 경계마다 상태를 다시 보면서 HOME 복귀 등을 체크하는 용도.
        """
        replan = False

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not a.start:
                continue
            home = self._home_of(ctx, a.id)

            # 1) 주문 목적지에 도착했는가?
            if any((a.id, tuple(a.start)) == (r, dst)
                   for (r, dst) in self.order_start):
                self.order_to_table.discard((a.id, tuple(a.start)))

                finished = next(
                    (
                        (r, dst)
                        for (r, dst) in self.order_start
                        if r == a.id and tuple(dst) == tuple(a.start)
                    ),
                    None,
                )
                if finished:
                    self.order_start.discard(finished)
                    self.order_done.append(finished)

                    # order_list에서 동일 명령 맨 앞 1개 제거
                    rm_idx = next(
                        (
                            i
                            for i, (r, dst) in enumerate(self.order_list)
                            if r == finished[0]
                            and tuple(dst) == tuple(finished[1])
                        ),
                        None,
                    )
                    if rm_idx is not None:
                        self.order_list.pop(rm_idx)

                    # ⭐ 여기서도 "테이블 도착 완료" 히스토리 남길 수 있음
                    if self.manager is not None:
                        self.manager.push_order_history(
                            rid=a.id,
                            order_id=finished,
                            goal=tuple(a.start),
                            status="ARRIVED_TABLE",
                        )

                    # 자동 HOME 복귀 명령 생성
                    if home and tuple(a.start) != tuple(home):
                        a.goal = tuple(home)
                        self.order_to_home.add((a.id, tuple(home)))
                        replan = True

            # 2) HOME에 도착했는가? → order_to_home 정리
            if home and tuple(a.start) == tuple(home):
                self.order_to_home = {
                    p for p in self.order_to_home if p[0] != a.id
                }

        if replan:
            return self.result(replan=True, reason="arrive_home_replan")
        return None

    def on_alignment_complete(self, rid: int, *, tag_info, grid,
                              agents, ctx, runstate):
        # 1) 초기 정렬 완료 추적
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        home = self._home_of(ctx, rid)

        # 초기 정렬 펜딩 관리
        pending = ctx.get("_init_align_pending")
        if isinstance(pending, set) and rid in pending:
            pending.discard(rid)
            if not pending:
                print("✅ 초기 중앙→방향 정렬 완료 (all robots)")

        # HOME이 아니라면: HOME으로 goal 부여 + replan
        if home and a.start and tuple(a.start) != tuple(home):
            a.goal = tuple(home)
            return self.result(replan=True, reason="align_then_home")

        # HOME 위라면 HOME 상태 진입
        if home and a.start and tuple(a.start) == tuple(home):
            self.home_set.add(rid)
        return None

    def on_robot_complete(self, rid: int, *, tag_info, grid,
                          agents, ctx, runstate):
        """
        컨트롤러에서 '이 로봇의 시퀀스가 끝났다'고 알려줄 때 호출.
        여기서:
          - 테이블 도착 → HOME 복귀 명령
          - HOME 도착 → HOME 정렬 요청
          - 그리고 history 로깅 (ARRIVED 등)
        """
        a = next((x for x in agents if x.id == rid), None)
        if not a or not a.start:
            return None

        home = self._home_of(ctx, rid)
        at_home = bool(home and tuple(a.start) == tuple(home))
        at_table = any(
            (rid, tuple(a.start)) == (r, dst)
            for (r, dst) in self.order_start
        )

        # A) 테이블 도착 → 즉시 HOME 목표 부여 + replan
        if at_table:
            self.order_to_table.discard((rid, tuple(a.start)))
            finished = next(
                (
                    (r, dst)
                    for (r, dst) in self.order_start
                    if r == rid and tuple(dst) == tuple(a.start)
                ),
                None,
            )
            if finished:
                self.order_start.discard(finished)
                self.order_done.append(finished)

                # order_list에서 동일 명령 맨 앞 1개 제거
                rm_idx = next(
                    (
                        i
                        for i, (r, dst) in enumerate(self.order_list)
                        if r == finished[0]
                        and tuple(dst) == tuple(finished[1])
                    ),
                    None,
                )
                if rm_idx is not None:
                    self.order_list.pop(rm_idx)

                # ⭐ 테이블 도착 히스토리
                if self.manager is not None:
                    self.manager.push_order_history(
                        rid=rid,
                        order_id=finished,
                        goal=tuple(a.start),
                        status="ARRIVED_TABLE",
                    )

            # HOME으로 자동 복귀
            if home and tuple(a.start) != tuple(home):
                a.goal = tuple(home)
                self.order_to_home.add((rid, tuple(home)))

                # ⭐ "HOME으로 복귀 시작" 히스토리 (선택)
                if self.manager is not None:
                    self.manager.push_order_history(
                        rid=rid,
                        order_id=("RETURN_HOME", finished),
                        goal=tuple(home),
                        status="GO_HOME",
                    )

                return self.result(replan=True, reason="table_done_go_home")
            return None  # 홈이 없거나 이미 홈이면 끝

        # B) HOME 도착 → 정렬 1회 요청 + 히스토리
        if at_home:
            if self.manager is not None:
                self.manager.push_order_history(
                    rid=rid,
                    order_id=("HOME", rid),
                    goal=tuple(home),
                    status="ARRIVED_HOME",
                )

            return self.result(
                replan=False,
                align_center={rid},
                align_direction={rid},
                reason="home_first_arrival_align",
            )

        # 그 외(중간 경유지/기타 완료)는 아무 것도 하지 않음
        return None

    # ------------------------- BaseMode bridging -------------------------
    def ensure_agent_ctx(self, ctx: Dict[int, dict], rid: int) -> dict:
        """BaseMode가 없는 standalone 상황에서도 동작하도록 하는 브릿지."""
        if rid not in ctx:
            ctx[rid] = {}
        return ctx[rid]

    def occupied_from_tags(self, tag_info: dict) -> Set[Cell]:
        """tag_info 에서 'grid_position' 기준으로 점유 셀 수집."""
        occ: Set[Cell] = set()
        for rid, dat in tag_info.items():
            gp = dat.get("grid_position")
            if gp and dat.get("status") == "On":
                occ.add((gp[0], gp[1]))
        return occ

    def result(self, **kwargs):
        """BaseMode.result와 같은 형태를 맞추기 위한 단순 pass-through."""
        return kwargs

    def exit(self, **kwargs):
        pass

    # ------------------------- UI / History 연동 -------------------------
    def export_ui_state(self, clear_new: bool = False):
        """
        ScenarioManager → FrameBus로 넘길 수 있는 UI용 상태 묶음.
        (ORDER LIST/GRID에서 사용)
        """
        state = {
            # 시각화 전용 (UI 애니메이션 분리)
            "order_new": self.order_new,              # 방금 추가(휘발)
            "order_list": list(self.order_list),      # 누적 큐(생성/대기)
            "order_done": list(self.order_done),      # 완료 로그

            # 실행중 상태 (로직+시각화 겸용)
            "order_to_table": list(self.order_to_table),  # (rid, table_dst)
            "order_to_home": list(self.order_to_home),    # (rid, home_dst)
            "home_set": list(self.home_set),              # HOME 대기중

            "next_order_eta": max(0.0, self._next_order_at - time.time()),
        }
        if clear_new:
            self.order_new = None
        return state

    def _seed_initial_orders(self, grid, agents, ctx, tag_info):
        """
        시작 시점에 각 로봇별로 1개씩 주문을 만들어 order_list에 적재.
        - 목적지는 장애물 옆 4방의 빈 칸 중에서 중복 없이 선정(_pick_from_table_meta 사용)
        - 이미 동일 rid의 주문이 있는 경우는 건너뜀(로봇별 1개 제한)
        """
        H, W = grid.shape
        starts = {tuple(a.start) for a in agents if a.start}
        goals = {tuple(a.goal) for a in agents if a.goal}
        homes = {
            tuple(self._home_of(ctx, a.id))
            for a in agents
            if self._home_of(ctx, a.id)
        }
        occ = self.occupied_from_tags(tag_info)
        taken = {dst for _, dst in self.order_list} | {
            dst for _, dst in self.order_start
        }
        forbidden_base = set(starts) | set(goals) | set(occ) | set(homes)

        visible_ids = [a.id for a in agents if a.start]
        for rid in visible_ids:
            if any(r == rid for (r, _) in self.order_list):
                continue
            forbidden = set(forbidden_base) | {
                dst for _, dst in self.order_list
            }
            dst = self._pick_from_table_meta(grid, forbidden | taken)
            if dst is None:
                continue
            self.order_list.append((rid, dst))

    def _load_table_chairs_from_file(self) -> List[Cell]:
        try:
            with self._cfg_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            chairs = [tuple(x) for x in data.get("chairs", [])]
            return chairs
        except FileNotFoundError:
            print(
                f"[RestaurantMode] WARNING: table_coords.json not found at {self._cfg_path}"
            )
            return []
        except Exception as e:
            print(f"[RestaurantMode] ERROR loading table_coords.json: {e}")
            return []

    def _reload_table_config(self, force: bool = False):
        """JSON 파일이 바뀌었으면 self.table_chairs 갱신."""
        try:
            mtime = os.path.getmtime(self._cfg_path)
        except OSError:
            if force:
                self.table_chairs = []
                self._cfg_mtime = None
            return

        if (
            not force
            and self._cfg_mtime is not None
            and mtime <= self._cfg_mtime
        ):
            return  # 변경 없음

        chairs = self._load_table_chairs_from_file()
        self.table_chairs = chairs
        self._cfg_mtime = mtime
        print(
            f"[RestaurantMode] table_coords.json reloaded, {len(chairs)} chairs"
        )

    # ------------------------- History 헬퍼 -------------------------
    def _agent_by_id(self, rid: int):
        """
        현재 tick에서 받은 self.agents 리스트에서 id로 에이전트를 찾는 유틸.
        (지금 코드는 직접 agents를 넘기는 방식이 많아서
         꼭 필요하진 않지만, history 확장 시 쓸 수 있게 준비)
        """
        for a in getattr(self, "agents", []):
            if a.id == rid:
                return a
        return None

    def get_current_order_for(self, rid: int):
        """
        ScenarioManager → UI status 계산 시
        '이 로봇이 지금 수행 중인 주문'을 표현하기 위한 헬퍼.
        - 현재 실행 중(order_start)에 있으면 그걸 우선
        - 아니면 대기 큐(order_list)에 있는 첫 주문
        - 없으면 None
        """
        for (r, dst) in self.order_start:
            if r == rid:
                return (rid, dst)
        for (r, dst) in self.order_list:
            if r == rid:
                return (rid, dst)
        return None

    def _goal_of_order(self, order):
        """
        get_current_order_for가 반환한 order 에서 goal 좌표만 빼는 함수.
        지금 구조에서는 order == (rid, dst) 튜플로 가정.
        """
        if order is None:
            return None
        _rid, dst = order
        return dst

    # (선택) 외부에서 수동으로 주문을 부여하고 싶을 때 사용할 수도 있는 API
    def assign_order_to_robot(self, rid, order_id, goal: Cell):
        """
        외부(UI 등)에서 직접 특정 로봇에 주문을 넣는 경우를 위한 헬퍼.
        - goal: 목적지 셀
        - order_id: UI/로그용 식별자 (string/tuple 아무거나 가능)
        """
        a = self._agent_by_id(rid)
        if not a:
            return None

        a.goal = tuple(goal)

        # history에 'ORDER_RECEIVED' 기록
        if self.manager is not None:
            self.manager.push_order_history(
                rid=rid,
                order_id=order_id,
                goal=tuple(goal),
                status="ORDER_RECEIVED",
            )

        return self.result(replan=True, reason="new_order")
