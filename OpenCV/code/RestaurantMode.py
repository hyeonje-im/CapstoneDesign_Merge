import random, time
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

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

    def _pick_table_adjacent(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:
        """장애물 셀의 4방 인접 자유칸 중, 금지(forbidden)에 없는 칸 하나를 무작위로."""
        H, W = grid.shape
        tables_adj = []
        for r in range(H):
            for c in range(W):
                if grid[r, c] == 0:
                    continue
                adj = [(rr, cc) for (rr, cc) in self._neighbors4(r, c, H, W) if grid[rr, cc] == 0]
                if adj:
                    tables_adj.append(adj)
        if not tables_adj:
            return None
        random.shuffle(tables_adj)
        for adj in tables_adj:
            cand = [p for p in adj if p not in forbidden]
            if cand:
                return random.choice(cand)
        return None
    # ↑ 랜덤모드에서 쓰던 “테이블 옆 4칸” 선택 방식을 그대로 사용. :contentReference[oaicite:3]{index=3}

    def _home_of(self, ctx: Dict[int, dict], rid: int) -> Optional[Cell]:
        h = self.ensure_agent_ctx(ctx, rid).get("home")
        return tuple(h) if h else None

    # ------------------------------ Ticking ------------------------------
    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """주문 생성, HOME 상태 유지, 필요 시 CBS 트리거."""
        replan = False
        now = time.time()

        # 1) 신규 주문 생성 타이밍이면 한 개 만든다 (UI: order_new → order_list 뒤에 append 후 휘발)
        if now >= self._next_order_at:
            self._next_order_at = now + self._rnd_span()
            # 목적지 후보 생성(중복 제외)
            occ = self.occupied_from_tags(tag_info)
            starts = {tuple(a.start) for a in agents if a.start}
            goals  = {tuple(a.goal) for a in agents if a.goal}
            homes  = {tuple(self._home_of(ctx, a.id)) for a in agents if self._home_of(ctx, a.id)}
            forbidden = set(starts) | set(goals) | set(occ) | set(homes)
            taken_dsts = {dst for _, dst in self.order_list} | {dst for _, dst in self.order_start}
            dst = self._pick_table_adjacent(grid, forbidden | taken_dsts)
            # 대상 로봇은 일단 무작위(보이는 로봇 중), 로봇별 최대 2개 제한
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
                self.order_new = (rid, dst)           # UI에서 잠깐 보일 버퍼
                self.order_list.append((rid, dst))    # 큐 뒤에 추가
            else:
                self.order_new = None  # 생성 실패(공간 부족/로봇 과다)면 휘발

        # 2) HOME 판정 및 유지
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not s.get("home") or not a.start:
                continue
            if tuple(a.start) != tuple(s["home"]):
                # HOME을 벗어나면 ready 해제
                self.home_set.discard(a.id)

        # 3) 별도 CBS 트리거 없음 (번호키로 소비 시 CBS 필요)
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
        idx = next((i for i,(r,_) in enumerate(self.order_list) if r == rid), None)
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
        return self.result(replan=True, reason="order_start")

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
        # 1) 초기 정렬 완료 추적
        a = next((x for x in agents if x.id == rid), None)
        if not a: 
            return None
        home = self._home_of(ctx, rid)

        # 초기 정렬 펜딩 관리(완료 메시지 출력은 현재 코드 그대로 유지)
        pending = ctx.get("_init_align_pending")
        if isinstance(pending, set) and rid in pending:
            pending.discard(rid)
            if not pending:
                print("✅ 초기 중앙→방향 정렬 완료 (all robots)")

        # ★ HOME이 아니라면: 이제 목표를 HOME으로 주고 replan 요청
        if home and a.start and tuple(a.start) != tuple(home):
            a.goal = tuple(home)
            return self.result(replan=True, reason="align_then_home")

        # HOME 위라면 HOME 상태 진입(현재 코드 그대로)
        if home and a.start and tuple(a.start) == tuple(home):
            self.home_set.add(rid)
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
        forbidden_base = set(starts) | set(goals) | set(occ) | set(homes) | set(taken)

        # 보이는(=start가 있는) 로봇에 대해 1개씩
        visible_ids = [a.id for a in agents if a.start]
        for rid in visible_ids:
            # rid에 이미 주문이 있으면 건너뜀 (로봇별 1개 제한)
            if any(r == rid for (r, _) in self.order_list):
                continue
            # 각 rid마다 목적지 선정 시, 이미 선택된 목적지도 추가로 금지
            forbidden = set(forbidden_base) | {dst for _, dst in self.order_list}
            dst = self._pick_table_adjacent(grid, forbidden)
            if dst is None:
                # 공간 부족하면 스킵 (다른 로봇은 계속 시도)
                continue
            self.order_list.append((rid, dst))
