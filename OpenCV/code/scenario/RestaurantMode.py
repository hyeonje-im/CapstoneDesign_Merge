import random, time
from typing import Dict, List, Optional, Set, Tuple
import json
from pathlib import Path

import os

import numpy as np

import grid

Cell = Tuple[int, int]

class RestaurantMode:
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

    # ---- Base utilities (ScenarioManager가 제공) ----
    # ensure_agent_ctx(self, ctx, rid) -> dict
    # occupied_from_tags(self, tag_info) -> Set[Cell]
    # result(self, *, replan=False, waiters=None, waiter_cells=None,
    #         align_center=None, align_direction=None, ready=None, reason="") -> dict
    # 위 유틸은 RandomMode 설계와 동일한 인터페이스로 사용한다. :contentReference[oaicite:2]{index=2}

    def __init__(self, *, home_provider=None, order_span_sec=(0, 30)):
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

    # ------------------------------ Lifecycle ------------------------------
    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # 상태 리셋 및 HOME 세팅
        self.order_new = None
        self.order_list.clear()
        self.order_start.clear()
        self.order_done.clear()
        self.home_set.clear()
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

        # 1) (필요시) 중앙>방향 정렬로 '수신 준비' 맞춤
        # 2) 각 로봇 goal=HOME 으로 설정하여 HOME 복귀 스타트
        initial_align_ids = [a.id for a in agents if a.start]
        ctx["_init_align_pending"] = set(initial_align_ids)

        replan = False
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not a.start or not s.get("home"):
                continue
            if tuple(a.start) != tuple(s["home"]):
                a.goal = tuple(s["home"])
                replan = True

        # 4) 초기 정렬 펜딩 집합 기록(완료 체크용)
        ctx["_init_align_pending"] = set(initial_align_ids)

        # 5) 반환: 중앙+방향 정렬 요청(필요 시 수행) + 필요하면 replan
        self._seed_initial_orders(grid, agents, ctx, tag_info)
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
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            rr, cc = r+dr, c+dc
            if 0 <= rr < H and 0 <= cc < W:
                yield rr, cc

    # def _pick_table_adjacent(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:
    #     """장애물 셀의 4방 인접 자유칸 중, 금지(forbidden)에 없는 칸 하나를 무작위로."""
    #     H, W = grid.shape
    #     tables_adj = []
    #     for r in range(H):
    #         for c in range(W):
    #             if grid[r, c] == 0:
    #                 continue
    #             adj = [(rr, cc) for (rr, cc) in self._neighbors4(r, c, H, W) if grid[rr, cc] == 0]
    #             if adj:
    #                 tables_adj.append(adj)
    #     if not tables_adj:
    #         return None
    #     random.shuffle(tables_adj)
    #     for adj in tables_adj:
    #         cand = [p for p in adj if p not in forbidden]
    #         if cand:
    #             return random.choice(cand)
    #     return None
    # # ↑ 랜덤모드에서 쓰던 “테이블 옆 4칸” 선택 방식을 그대로 사용. :contentReference[oaicite:3]{index=3}

    def _pick_from_table_meta(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:

        if not self.table_chairs:
            return None

        H, W = grid.shape
        fbd = set(forbidden)
        taken_dsts = {dst for _, dst in self.order_list} | {dst for _, dst in self.order_start}
        fbd |= taken_dsts

        # 유효한 후보만 필터링
        candidates: List[Cell] = []
        for (r, c) in self.table_chairs:
            # 범위 밖이면 스킵
            if not self._in_bounds(r, c, H, W):
                continue
            # 통로(0)가 아니면 스킵
            if grid[r, c] != 0:
                continue
            # 이미 금지/사용 중이면 스킵
            if (r, c) in fbd:
                continue
            candidates.append((r, c))

        if not candidates:
            return None

        import random
        return random.choice(candidates)



    def _home_of(self, ctx: Dict[int, dict], rid: int) -> Optional[Cell]:
        h = self.ensure_agent_ctx(ctx, rid).get("home")
        return tuple(h) if h else None

    def _dir_vec(self, d: str):
        # 행(r)↓, 열(c)→ 기준
        return {"n": (-1,0), "e": (0,1), "s": (1,0), "w": (0,-1)}.get(d.lower(), (0,0))

    def _in_bounds(self, r, c, H, W):
        return 0 <= r < H and 0 <= c < W

    # def _chairs_for_table(self, *, grid: np.ndarray, table_rc, direction: str, chair_count: int):
    #     """
    #     테이블 좌표·방향·의자 수로 '의자 후보 좌표 리스트'를 계산한다.
    #     규칙:
    #     - 1개: 테이블에서 방향으로 1칸
    #     - 2개: (우선) 방향으로 1칸 + 그 좌/우로 1칸 중 가능한 셀
    #         (예: n이면 (r-1,c) 우선, 그 옆 (r-1,c-1), (r-1,c+1) 시도)
    #     grid[r,c]==0 이 통로, 1이 장애물.
    #     """
    #     H, W = grid.shape
    #     tr, tc = table_rc
    #     dr, dc = self._dir_vec(direction)

    #     prim = (tr+dr, tc+dc)
    #     # 좌/우(옆) 벡터: (dr,dc)를 기준으로 시계·반시계 회전
    #     left  = (-dc, dr)
    #     right = (dc, -dr)

    #     candidates = []
    #     # 1순위: 방향으로 1칸
    #     if self._in_bounds(*prim, H, W) and grid[prim[0], prim[1]] == 0:
    #         candidates.append(prim)

    #     # 2개 의자면 좌/우측 한 칸을 추가 후보로 시도
    #     if chair_count >= 2 and candidates:
    #         lr, lc = prim[0]+left[0], prim[1]+left[1]
    #         rr, rc = prim[0]+right[0], prim[1]+right[1]
    #         if self._in_bounds(lr, lc, H, W) and grid[lr, lc] == 0:
    #             candidates.append((lr, lc))
    #         elif self._in_bounds(rr, rc, H, W) and grid[rr, rc] == 0:
    #             candidates.append((rr, rc))

    #     # chair_count가 1인데 prim이 막혔거나,
    #     # 2인데 prim이 막혔을 수 있으니, 보정: prim이 막히면 좌/우부터라도 채움
    #     if not candidates:
    #         # prim이 막혔으면 좌/우 시도
    #         lr, lc = tr+left[0], tc+left[1]
    #         rr, rc = tr+right[0], tc+right[1]
    #         for cand in [(lr,lc), (rr,rc)]:
    #             if self._in_bounds(*cand, H, W) and grid[cand[0], cand[1]] == 0:
    #                 candidates.append(cand)
    #                 if chair_count == 1:
    #                     break

    #     # chair_count 초과로 뽑히지 않도록 제한
    #     return candidates[:chair_count]

    # ------------------------------ Ticking ------------------------------
    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """주문 생성, HOME 상태 유지, 필요 시 CBS 트리거."""
        replan = False
        now = time.time()

        self._reload_table_config(force=False)

        # 1) 주문 생성
        if now >= self._next_order_at:
            self._next_order_at = now + self._rnd_span()

            occ = self.occupied_from_tags(tag_info)
            starts = {tuple(a.start) for a in agents if a.start}
            goals  = {tuple(a.goal) for a in agents if a.goal}
            homes  = {tuple(self._home_of(ctx, a.id)) for a in agents if self._home_of(ctx, a.id)}

            forbidden = set(starts) | set(goals) | set(occ) | set(homes)
            taken_dsts = {dst for _, dst in self.order_list} | {dst for _, dst in self.order_start}
            dst = self._pick_from_table_meta(grid, forbidden | taken_dsts)

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
                self.order_new = (rid, dst)
                self.order_list.append((rid, dst))
            else:
                self.order_new = None

        # 2) HOME 판정
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not s.get("home") or not a.start:
                continue
            if tuple(a.start) != tuple(s["home"]):
                self.home_set.discard(a.id)

        # 3) ★ ready_for_order 계산
        ready_state = {}
        for a in agents:
            rid = a.id
            s = self.ensure_agent_ctx(ctx, rid)
            home = s.get("home")

            if s.get("homeless") or not home or not a.start:
                ready_state[rid] = False
                continue

            if tuple(a.start) == tuple(home) and rid in self.home_set:
                ready_state[rid] = True
            else:
                ready_state[rid] = False

        # 4) ★ ready_for_order 반환
        return self.result(
            replan=False,
            ready_for_order=ready_state,
            reason="tick_ready"
        )
            

    def on_number_key(self, rid: int, *, agents: List, ctx: Dict[int, dict]):
        if rid not in self.home_set:
            return None

        idx = next((i for i,(r,_) in enumerate(self.order_list) if r == rid), None)
        if idx is None:
            return None

        (r, dst) = self.order_list.pop(idx)
        self.order_start.add((r, dst))
        self.order_to_table.add((r, dst))
        self.home_set.discard(rid)

        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None

        a.goal = tuple(dst)

        # ★ 주문 시작했으므로 ready=False
        return self.result(
            replan=True,
            ready_for_order={rid: False},
            reason="order_start"
        )

    # ------------------------------ 콜백 ------------------------------
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """
        목적지 도착
        """
        replan = False

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not a.start:
                continue
            home = self._home_of(ctx, a.id)

            # 1) 주문 목적지에 도착했는가?
            if any((a.id, tuple(a.start)) == (r, dst) for (r, dst) in self.order_start):
                self.order_to_table.discard((a.id, tuple(a.start)))
                # 자동 복귀 기록 (rid, home)
                if home and tuple(a.start) != tuple(home):
                    self.order_to_home.add((a.id, tuple(home)))   # ← 추가
                    a.goal = tuple(home)
                    replan = True
                finished = next(((r, dst) for (r, dst) in self.order_start if r == a.id and tuple(dst)==tuple(a.start)), None)
                if finished:
                    self.order_start.discard(finished)
                    self.order_done.append(finished)
                    # order_list에서 동일 명령 맨 앞 1개 제거
                    rm_idx = next((i for i,(r,dst) in enumerate(self.order_list)
                                   if r==finished[0] and tuple(dst)==tuple(finished[1])), None)
                    if rm_idx is not None:
                        self.order_list.pop(rm_idx)
                    # 자동 HOME 복귀 명령 생성
                    if home and tuple(a.start) != tuple(home):
                        a.goal = tuple(home)
                        replan = True

            # 2) HOME에 도착했는가? → 정렬 요청(출하 준비), 정렬 완료되면 HOME 상태 유지
            if home and tuple(a.start) == tuple(home):
                self.order_to_home = {p for p in self.order_to_home if p[0] != a.id}

        if replan:
            return self.result(replan=True, reason="arrive_home_replan")
        return None

    # RestaurantMode.py — on_alignment_complete(...)
    def on_alignment_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate):
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        home = self._home_of(ctx, rid)

        pending = ctx.get("_init_align_pending")
        if isinstance(pending, set) and rid in pending:
            pending.discard(rid)
            if not pending:
                print("✅ 초기 중앙→방향 정렬 완료 (all robots)")

        # HOME이 아니면 HOME 복귀
        if home and a.start and tuple(a.start) != tuple(home):
            a.goal = tuple(home)
            return self.result(replan=True, reason="align_then_home")

        # ★ HOME이면 HOME 상태
        if home and a.start and tuple(a.start) == tuple(home):
            self.home_set.add(rid)

            # ★ 정렬까지 끝났으므로 ready=True
            return self.result(
                replan=False,
                ready_for_order={rid: True},
                reason="home_align_ready"
            )

        return None                     


    def on_robot_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate):
        a = next((x for x in agents if x.id == rid), None)
        if not a or not a.start:
            return None

        home = self._home_of(ctx, rid)
        at_home  = bool(home and tuple(a.start) == tuple(home))
        at_table = any((rid, tuple(a.start)) == (r, dst) for (r, dst) in self.order_start)
        
        # A) 테이블 도착 → 즉시 HOME 목표 부여 + replan
        if at_table:
            self.order_to_table.discard((rid, tuple(a.start)))
            
            finished = next(((r, dst) for (r, dst) in self.order_start
                             if r == rid and tuple(dst) == tuple(a.start)), None)
            if finished:
                self.order_start.discard(finished)
                self.order_done.append(finished)
                # order_list에서 동일 명령 맨 앞 1개 제거
                rm_idx = next((i for i,(r,dst) in enumerate(self.order_list)
                               if r==finished[0] and tuple(dst)==tuple(finished[1])), None)
                if rm_idx is not None:
                    self.order_list.pop(rm_idx)

            if home and tuple(a.start) != tuple(home):
                a.goal = tuple(home)
                self.order_to_home.add((rid, tuple(home)))
                return self.result(replan=True, reason="table_done_go_home")
            return None  # 홈이 없거나 이미 홈이면 더 할 일 없음
        
        # ★ 정렬 정책 수렴: HOME ‘최초 도착’에서만 중앙+방향 정렬 1회
        if at_home:
            return self.result(
                replan=False,
                align_center={rid},
                align_direction={rid},
                ready_for_order={rid: True},
                reason="home_first_arrival_align",
                align_delay_sec=0.5
            )

        # 그 외(중간 경유지/기타 완료)는 아무 것도 하지 않음
        return None

    # ------------------------- BaseMode bridging -------------------------
    def ensure_agent_ctx(self, ctx: Dict[int, dict], rid: int) -> dict:  # pragma: no cover
        if rid not in ctx:
            ctx[rid] = {}
        return ctx[rid]

    def occupied_from_tags(self, tag_info: dict) -> Set[Cell]:  # pragma: no cover
        return set(tag_info.get("occupied", []))

    def result(self, **kwargs):  # pragma: no cover
        return kwargs

    def exit(self, **kwargs):  # pragma: no cover
        pass

    def export_ui_state(self, clear_new: bool = False):
        import time
        state = {
            # 시각화 전용 (UI 애니메이션 분리)
            "order_new": self.order_new,               # 방금 추가(휘발)
            "order_list": list(self.order_list),       # 누적 큐(생성/대기)
            "order_done": list(self.order_done),       # 방금 완료(제거용)

            # 실행중 상태 (로직+시각화 겸용)
            "order_to_table": list(self.order_to_table),   # (rid, table_dst)
            "order_to_home":   list(self.order_to_home),     # (rid, home_dst)
            "home_set":       list(self.home_set),         # HOME 대기중

            "next_order_eta": max(0.0, self._next_order_at - time.time()),
        }
        if clear_new:
            self.order_new = None
        return state
    
    def _seed_initial_orders(self, grid, agents, ctx, tag_info):
        """
        시작 시점에 각 로봇별로 1개씩 주문을 만들어 order_list에 적재.
        - 목적지는 장애물 옆 4방의 빈 칸 중에서 중복 없이 선정(_pick_table_adjacent 재사용)
        - 이미 동일 rid의 주문이 있는 경우는 건너뜀(중복 방지)
        """
        H, W = grid.shape
        # 금지 셀 구성: 현재 start/goal, 실점유(비전), 기존 주문 목적지, HOME
        starts = {tuple(a.start) for a in agents if a.start}
        goals  = {tuple(a.goal) for a in agents if a.goal}
        homes  = {tuple(self._home_of(ctx, a.id)) for a in agents if self._home_of(ctx, a.id)}
        occ    = self.occupied_from_tags(tag_info)
        taken  = {dst for _, dst in self.order_list} | {dst for _, dst in self.order_start}
        forbidden_base = set(starts) | set(goals) | set(occ) | set(homes)
        # 보이는(=start가 있는) 로봇에 대해 1개씩
        visible_ids = [a.id for a in agents if a.start]
        for rid in visible_ids:
            # rid에 이미 주문이 있으면 건너뜀 (로봇별 1개 제한)
            if any(r == rid for (r, _) in self.order_list):
                continue
            # 각 rid마다 목적지 선정 시, 이미 선택된 목적지도 추가로 금지
            forbidden = set(forbidden_base) | {dst for _, dst in self.order_list}
            dst = self._pick_from_table_meta(grid, forbidden | taken)
            if dst is None:
                # 공간 부족하면 스킵 (다른 로봇은 계속 시도)
                continue
            self.order_list.append((rid, dst))

    def _load_table_chairs_from_file(self) -> List[Cell]:
        try:
            with self._cfg_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            chairs = [tuple(x) for x in data.get("chairs", [])]
            return chairs
        except FileNotFoundError:
            print(f"[RestaurantMode] WARNING: table_coords.json not found at {self._cfg_path}")
            return []
        except Exception as e:
            print(f"[RestaurantMode] ERROR loading table_coords.json: {e}")
            return []

    def _reload_table_config(self, force: bool = False):
        """JSON 파일이 바뀌었으면 self.table_chairs 갱신."""
        try:
            mtime = os.path.getmtime(self._cfg_path)
        except OSError:
            # 파일이 없거나 접근 불가
            if force:
                self.table_chairs = []
                self._cfg_mtime = None
            return

        if not force and self._cfg_mtime is not None and mtime <= self._cfg_mtime:
            return  # 변경 없음

        chairs = self._load_table_chairs_from_file()
        self.table_chairs = chairs
        self._cfg_mtime = mtime
        print(f"[RestaurantMode] table_coords.json reloaded, {len(chairs)} chairs")


    def get_ui_state(self, drain_new=False):
        ui = {}

        for rid in [1, 2, 3]:  # 로봇 ID
            st = self.manager.compute_robot_status(rid, self._last_mode_result) \
                if hasattr(self, "_last_mode_result") else "IDLE"

            pos = self.manager.get_robot_position(rid)  # (row, col)
            goal = self.manager.get_robot_goal(rid)

            ui[rid] = {
                "num": f"#{rid}",
                "pos": str(pos) if pos else "-",
                "goal": str(goal) if goal else "-",
                "status": st
            }

        # 필요한 경우 new-orders drain 기능 처리
        if drain_new:
            self.new_orders = []
        
        return ui