import random
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# Type aliases (kept loose to fit existing project types)
Cell = Tuple[int, int]

# The surrounding codebase provides these base classes / types:
# - BaseMode: utilities like ensure_agent_ctx, occupied_from_tags, result(...)
# - ModeResult: return envelope consumed by ScenarioManager
# - Agent: carries .id, .start, .goal, .delay
# They are intentionally not re-imported here to avoid circular imports.


class RandomMode:  # inherits BaseMode at runtime in your project
    """
    TestMode와 동일한 구동/콜백 흐름을 유지하면서,
    '목표를 찍는 방식'만 홈↔테이블(장애물) 인접 자유칸 규칙으로 바꾼 모드.

    • 초기/유휴 배정: 홈이면 → 테이블 인접, 아니면 → 홈
    • 도착 직후: on_sequence_complete()가 정렬 요청
    • 정렬 완료: on_alignment_complete()에서 다음 목표 배정(홈이면 테이블 인접 / 아니면 홈)
    • 개별 완료: on_robot_complete()에서도 TestMode와 동일 타이밍으로 다음 목표 배정
    • 홈→테이블로 출발할 때만 a.delay = randint(0,3) 부여 (CBS가 지연을 경로에 반영)
      테이블→홈은 a.delay = 0
    • 홈이 없는 로봇은 homeless=True: 명령 미발송, 그 자리 한 칸 임시 장애물로만 취급
    """

    # ---- Base utilities expected to exist on BaseMode ----
    # ensure_agent_ctx(self, ctx, rid) -> dict
    # occupied_from_tags(self, tag_info) -> Set[Cell]
    # result(self, *, replan=False, waiters=None, waiter_cells=None,
    #         align_center=None, align_direction=None, ready=None, reason="") -> ModeResult

    def __init__(self, *, idle_threshold_frames: int = 12, home_provider: Optional[callable] = None):
        self.idle_thresh = idle_threshold_frames
        self.home_provider = home_provider

    # ----------------------------- Lifecycle -----------------------------
    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            home_from_main = self.home_provider(a.id) if self.home_provider else None
            if home_from_main is not None:
                s["home"] = tuple(home_from_main)
                s["homeless"] = False
            else:
                s["home"] = None
                s["homeless"] = True
                a.goal = None  # 명령 미발송 대상
            # 정렬/검증 상태 초기화
            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["last_pos"] = tuple(a.start) if a.start else None
            s["idle_frames"] = 0
            s["hd_active"] = False   # 현재 홈출발 딜레이 세션 활성화 여부
            s["hd_left"]   = 0

    # ------------------------------ Helpers ------------------------------
    def _neighbors4(self, r: int, c: int, H: int, W: int):
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W:
                yield rr, cc

    def _pick_table_adjacent(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:
        """장애물(=테이블) 셀의 4방 인접 자유칸 중 금지와 겹치지 않는 칸을 하나 고른다."""
        H, W = grid.shape
        tables_adj: List[List[Cell]] = []
        for r in range(H):
            for c in range(W):
                if grid[r, c] == 0:
                    continue  # free는 테이블 아님
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

    def _next_goal_for(self, a, grid: np.ndarray, forbidden: Set[Cell], ctx: Dict[int, dict]) -> Optional[Cell]:
        s = self.ensure_agent_ctx(ctx, a.id)
        if s.get("homeless"):
            return None
        cur = tuple(a.start) if a.start else None
        home = s.get("home")
        if home and cur == tuple(home):
            return self._pick_table_adjacent(grid, forbidden)  # 홈이면 → 테이블 인접
        else:
            return tuple(home) if home else None               # 아니면 → 홈

    # ------------------------------ Ticking ------------------------------
    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        replan = False
        waiters: Set[int] = set()       # 목표 없는(=진짜 waiter) 로봇만
        waiter_cells: Set[Cell] = set() # 해당 로봇의 현재 셀을 막는다

        # 홈리스는 항상 목표 없음 → 차단 셀로만 올림
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless"):
                if a.start:
                    waiter_cells.add(tuple(a.start))
                waiters.add(a.id)

        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(a.start) for a in agents if a.start}
        goals = {tuple(a.goal) for a in agents if a.goal}
        homes = {tuple(self.ensure_agent_ctx(ctx, a.id).get("home"))
                 for a in agents if self.ensure_agent_ctx(ctx, a.id).get("home")}

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless"):
                continue

            cur = tuple(a.start) if a.start else None
            last = s.get("last_pos")
            s["idle_frames"] = (s.get("idle_frames", 0) + 1) if (cur is not None and last == cur) else 0
            s["last_pos"] = cur
            executing = (runstate.get(a.id) or {}).get("executing", None)
            is_idle_now = (executing is False) or (executing is None and s["idle_frames"] >= self.idle_thresh)

            # TestMode와 동일: 도착 직후(=start==goal)에는 tick에서 새 목표를 찍지 않음 (정렬 먼저)
            arrived = (a.start and a.goal and tuple(a.start) == tuple(a.goal))
            if s.get("verifying") or arrived:
                continue

            # 아직 도착 전인데 목표가 없으면 초기/유휴 배정
            if is_idle_now and not a.goal:
                forbidden = set(starts) | set(goals) | set(occ) | set(homes)
                ng = self._next_goal_for(a, grid, forbidden, ctx)
                if ng is not None:
                    is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
                    a.goal = ng
                    at_home = (s.get("home") and cur == tuple(s["home"]))
                    if at_home:
                        # 홈에서 테이블로 '새 목표'를 받는 최초 순간에만 번들 생성
                        if is_new_goal and not s.get("hd_active", False):
                            s["hd_active"] = True
                            s["hd_left"]   = random.randint(0, 3)
                        # 매 계획 시 CBS에 남은 딜레이를 반영
                        a.delay = int(s.get("hd_left", 0))
                    else:
                        # 테이블→홈: 지연 없음, 홈-출발 번들 종료
                        a.delay = 0
                        s["hd_active"] = False
                        s["hd_left"]   = 0
                    goals.add(tuple(ng))
                    replan = True
                
                else:
                    # 목표를 만들 수 없으면(테이블 인접 없음 등) → 임시 차단만
                    if a.start:
                        waiter_cells.add(tuple(a.start))
                        waiters.add(a.id)

        if replan or waiters or waiter_cells:
            return self.result(replan=replan, waiters=waiters, waiter_cells=waiter_cells, reason="idle")
        return None

    # ------------------------------ Callbacks ----------------------------
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """도착 로봇에 정렬을 요청(TestMode와 동일한 타이밍)."""
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless"):
                continue
            if s.get("hd_active") and s.get("hd_left", 0) > 0:
                cur  = tuple(a.start) if a.start else None
                last = s.get("last_pos")
                if cur is not None and last == cur:
                    s["hd_left"] = max(0, int(s["hd_left"]) - 1)
                    a.delay      = int(s["hd_left"])  # 다음 CBS에 반영되도록 최신값 유지

        align_center: Set[int] = set()
        align_direction: Set[int] = set()
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless"):
                continue
            if a.start and a.goal and tuple(a.start) == tuple(a.goal):
                s["verifying"] = True
                s["verify_goal"] = tuple(a.goal)
                align_center.add(a.id)
                align_direction.add(a.id)
        if align_center or align_direction:
            return self.result(replan=False, align_center=align_center, align_direction=align_direction, reason="done")
        return None

    def on_alignment_complete(self, rid: int, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        s = self.ensure_agent_ctx(ctx, rid)
        if s.get("homeless"):
            return None

        # 정렬 검증: 현 위치가 직전에 도착한 목표와 일치해야 함
        if a.start and s.get("verify_goal") and tuple(a.start) == tuple(s["verify_goal"]):
            s["verifying"] = False
            s["verify_goal"] = None

            # 다음 목표 배정 (홈↔테이블 인접 규칙)
            occ = self.occupied_from_tags(tag_info)
            starts = {tuple(x.start) for x in agents if x.start}
            goals = {tuple(x.goal) for x in agents if x.goal}
            homes = {tuple(self.ensure_agent_ctx(ctx, x.id).get("home"))
                     for x in agents if self.ensure_agent_ctx(ctx, x.id).get("home")}
            forbidden = set(starts) | set(goals) | set(occ) | set(homes)

            cur = tuple(a.start) if a.start else None
            at_home = s.get("home") and cur == tuple(s["home"])
            ng = self._pick_table_adjacent(grid, forbidden) if at_home else (tuple(s["home"]) if s.get("home") else None)

            if ng is not None:
                is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
                a.goal = ng
                if at_home:
                    if is_new_goal and not s.get("hd_active", False):
                        s["hd_active"] = True
                        s["hd_left"]   = random.randint(0, 3)
                    a.delay = int(s.get("hd_left", 0))
                else:
                    a.delay = 0
                    s["hd_active"] = False
                    s["hd_left"]   = 0
                return self.result(replan=True, reason="align_ok_next")
            else:
                a.goal = None
                waiters, waiter_cells = set(), set()
                if a.start:
                    waiters.add(a.id)
                    waiter_cells.add(tuple(a.start))
                return self.result(replan=False, waiters=waiters, waiter_cells=waiter_cells, reason="align_ok_no_goal")

        # 검증 불일치: 재정렬 유도(TestMode와 동일)
        s["verifying"] = True
        s["verify_goal"] = tuple(a.goal) if a.goal else None
        return self.result(replan=False, align_center={rid}, align_direction={rid}, reason="align_retry")

    def on_robot_complete(self, rid: int, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """TestMode와 동일: 개별 완료 시 바로 다음 목표를 배정하고 재계획.
        (정렬 완료 콜백에서도 배정되므로, 환경/타이밍에 따라 둘 중 하나만 호출돼도 정상 진행)
        """
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        s = self.ensure_agent_ctx(ctx, rid)
        if s.get("homeless"):
            return None

        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(x.start) for x in agents if x.start}
        goals = {tuple(x.goal) for x in agents if x.goal}
        homes = {tuple(self.ensure_agent_ctx(ctx, x.id).get("home"))
                 for x in agents if self.ensure_agent_ctx(ctx, x.id).get("home")}
        forbidden = set(starts) | set(goals) | set(occ) | set(homes)

        ng = self._next_goal_for(a, grid, forbidden, ctx)
        if ng is not None:
            is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
            a.goal = ng
            cur = tuple(a.start) if a.start else None
            at_home = s.get("home") and cur == tuple(s["home"])
            if at_home:
                if is_new_goal and not s.get("hd_active", False):
                    s["hd_active"] = True
                    s["hd_left"]   = random.randint(0, 3)
                a.delay = int(s.get("hd_left", 0))
            else:
                a.delay = 0
                s["hd_active"] = False
                s["hd_left"]   = 0
            return self.result(replan=True, reason="robot_done")
        else:
            a.goal = None
            return self.result(replan=True, reason="robot_done_no_goal")

    # ------------------------- BaseMode bridging -------------------------
    # NOTE: In your actual codebase, RandomMode inherits BaseMode so these
    # methods are available. If running standalone, you may need to mixin.
    def ensure_agent_ctx(self, ctx: Dict[int, dict], rid: int) -> dict:  # pragma: no cover
        if rid not in ctx:
            ctx[rid] = {}
        return ctx[rid]

    def occupied_from_tags(self, tag_info: dict) -> Set[Cell]:  # pragma: no cover
        return set(tag_info.get("occupied", []))

    def result(self, **kwargs):  # pragma: no cover
        return kwargs


    def exit(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        """
        ScenarioManager가 모드 교체 전에 호출하는 종료 훅.
        상태를 정리하고 외부 부작용은 만들지 않는다.
        """
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            # 정렬/검증 및 홈-출발 딜레이 번들 정리
            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["hd_active"] = False
            s["hd_left"] = 0
            # 모드 종료 시, 딜레이는 CBS에 남기지 않도록 0으로 리셋
            try:
                a.delay = 0
            except Exception:
                pass