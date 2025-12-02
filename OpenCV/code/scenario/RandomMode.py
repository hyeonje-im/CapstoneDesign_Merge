# RandomMode.py

from __future__ import annotations

import random
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# 타입 별칭
Cell = Tuple[int, int]


class RandomMode:
    """
    TestMode와 동일한 구동/콜백 흐름을 유지하면서,
    '목표를 찍는 방식'만 홈↔테이블(장애물) 인접 자유칸 규칙으로 바꾼 모드.

    • 초기/유휴 배정: HOME이면 → 테이블 인접, 아니면 → HOME
    • 도착 직후: on_sequence_complete() 가 정렬 요청
    • 정렬 완료: on_alignment_complete() 에서 다음 목표 배정
    • 개별 완료: on_robot_complete() 에서도 다음 목표 배정
    • HOME→테이블 출발 때만 a.delay = randint(0,3) 부여
      테이블→HOME은 a.delay = 0
    • HOME이 없는 로봇은 homeless=True: 명령 미발송, 그 자리만 임시 장애물 취급
    """

    # ---- Base utilities (BaseMode / ScenarioManager 쪽에서 제공된다고 가정) ----
    # self.ensure_agent_ctx(ctx, rid) -> dict
    # self.occupied_from_tags(tag_info) -> Set[Cell]
    # self.result(... ) -> ModeResult

    def __init__(
        self,
        *,
        idle_threshold_frames: int = 12,
        home_provider: Optional[callable] = None,
    ):
        # 위치가 idle_threshold_frames 프레임 동안 안 움직이면 "놀고 있다"라고 판정
        self.idle_thresh = idle_threshold_frames
        # 외부(ScenarioManager)에서 각 로봇의 HOME 좌표를 받아오는 콜백
        self.home_provider = home_provider

    # ----------------------------- Lifecycle ----------------------------- #
    def enter(self, *, tag_info, grid, agents, ctx, runstate):
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)

            # 1) HOME 설정
            home_from_main = self.home_provider(a.id) if self.home_provider else None
            if home_from_main is not None:
                s["home"] = tuple(home_from_main)
                s["homeless"] = False
            else:
                s["home"] = None
                s["homeless"] = True

            # 2) ★ 기존 목표/딜레이 전부 초기화
            a.goal = None
            try:
                a.delay = 0
            except Exception:
                pass

            # 3) 정렬/검증 상태 및 idle 상태 초기화
            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["last_pos"] = tuple(a.start) if a.start else None
            s["idle_frames"] = 0

            # 4) HOME 출발 지연 번들 상태
            s["hd_active"] = False
            s["hd_left"] = 0

            # 5) ★ TestMode와 동일한 초기화 플래그
            s["init_done"] = False

    # ------------------------------ Helpers ------------------------------ #
    def _neighbors4(self, r: int, c: int, H: int, W: int):
        """4방향 인접 셀 생성기."""
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W:
                yield rr, cc

    def _pick_table_adjacent(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:
        """
        장애물(=테이블) 셀의 4방 인접 자유칸 중, 금지(forbidden)에 없는 칸을 하나 고른다.
        - grid[r,c] != 0 인 셀을 테이블로 보고, 그 4방향 중 grid==0 인 칸을 후보로 사용
        """
        H, W = grid.shape
        tables_adj: List[List[Cell]] = []

        for r in range(H):
            for c in range(W):
                if grid[r, c] == 0:
                    continue  # free는 테이블 아님
                adj = [
                    (rr, cc)
                    for (rr, cc) in self._neighbors4(r, c, H, W)
                    if grid[rr, cc] == 0
                ]
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

    def _next_goal_for(
        self,
        a,
        grid: np.ndarray,
        forbidden: Set[Cell],
        ctx: Dict[int, dict],
    ) -> Optional[Cell]:
        """
        "다음 목표" 규칙:
        - HOME에 있으면: 테이블 인접 칸 중 하나
        - HOME이 아니면: HOME 좌표
        - 홈리스: None
        """
        s = self.ensure_agent_ctx(ctx, a.id)
        if s.get("homeless"):
            return None

        cur = tuple(a.start) if a.start else None
        home = s.get("home")

        if home and cur == tuple(home):
            # HOME이면 테이블 인접 칸으로
            return self._pick_table_adjacent(grid, forbidden)
        else:
            # 그 외에는 HOME으로 복귀
            return tuple(home) if home else None

    # ------------------------------ Ticking ------------------------------ #
    def tick(self, *, tag_info, grid, agents, ctx, runstate):
        replan = False
        waiters: Set[int] = set()
        waiter_cells: Set[Cell] = set()

        # 0) 홈리스는 항상 목표 없음 → 현재 위치를 임시 장애물로만 사용
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") and a.start:
                waiter_cells.add(tuple(a.start))
                waiters.add(a.id)

        # 0-1) ★ 아직 init_done이 안 된 로봇이 있으면 → 초기 목표 부여 모드
        initial_phase = any(
            not (self.ensure_agent_ctx(ctx, a.id).get("init_done", False))
            for a in agents
        )

        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(a.start) for a in agents if a.start}
        goals = {tuple(a.goal) for a in agents if a.goal}
        homes = {
            tuple(self.ensure_agent_ctx(ctx, a.id).get("home"))
            for a in agents
            if self.ensure_agent_ctx(ctx, a.id).get("home")
        }

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)

            if s.get("homeless"):
                continue

            cur = tuple(a.start) if a.start else None
            last = s.get("last_pos")
            s["idle_frames"] = (
                s.get("idle_frames", 0) + 1 if (cur is not None and last == cur) else 0
            )
            s["last_pos"] = cur

            executing = (runstate.get(a.id) or {}).get("executing", None)

            if initial_phase:
                # ★ 초기 단계에서는 idle 여부 상관없이 "새로 시작"이라고 보고 처리
                is_idle_now = True
            else:
                is_idle_now = (executing is False) or (
                    executing is None and s["idle_frames"] >= self.idle_thresh
                )

            arrived = bool(a.start and a.goal and tuple(a.start) == tuple(a.goal))
            if s.get("verifying") or arrived:
                continue

        # ================================
        #     새 목표 생성 조건
        # ================================
            if is_idle_now and not a.goal:
                forbidden = set(starts) | set(goals) | set(occ) | set(homes)
                ng = self._next_goal_for(a, grid, forbidden, ctx)

                if ng is not None:
                    is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
                    a.goal = ng

                    at_home = bool(s.get("home") and cur == tuple(s["home"]))
                    if at_home:
                        if is_new_goal and not s.get("hd_active", False):
                            s["hd_active"] = True
                            s["hd_left"] = random.randint(0, 3)
                        a.delay = int(s.get("hd_left", 0))
                    else:
                        a.delay = 0
                        s["hd_active"] = False
                        s["hd_left"] = 0

                    goals.add(tuple(ng))
                    replan = True
                else:
                    if a.start:
                        waiter_cells.add(tuple(a.start))
                        waiters.add(a.id)

            # ★ 초기 단계였다면, 한 바퀴 돌고 나서 init_done 표시
        if initial_phase:
            for a in agents:
                self.ensure_agent_ctx(ctx, a.id)["init_done"] = True

        if replan or waiters or waiter_cells:
            return self.result(
                replan=replan,
                waiters=waiters,
                waiter_cells=waiter_cells,
                reason="init" if initial_phase else "idle",
            )
        return None

    # ------------------------------ Callbacks ---------------------------- #
    def on_sequence_complete(
        self,
        *,
        tag_info: dict,
        grid: np.ndarray,
        agents: List,
        ctx: Dict[int, dict],
        runstate: Dict[int, dict],
    ):
        """
        시퀀스 한 번 끝났을 때 호출.
        - start == goal 인 로봇들에 대해 1회 중앙 + 방향 정렬을 요청하고,
          alignment 콜백에서 실제 위치를 태그로 검증한 뒤 다음 목표를 부여.
        """
        align_center: Set[int] = set()
        align_direction: Set[int] = set()

        for a in agents:
            if a.start and a.goal and tuple(a.start) == tuple(a.goal):
                s = self.ensure_agent_ctx(ctx, a.id)
                s["verifying"] = True
                s["verify_goal"] = tuple(a.goal)
                align_center.add(a.id)
                align_direction.add(a.id)

        if align_center or align_direction:
            return self.result(
                replan=False,
                align_center=align_center,
                align_direction=align_direction,
                reason="done",
            )
        return None

    def on_alignment_complete(
        self,
        rid: int,
        *,
        tag_info,
        grid,
        agents,
        ctx,
        runstate,
    ):
        """
        특정 로봇(rid)에 대한 정렬(중앙+방향) 완료 콜백.
        - 태그 위치와 verify_goal 을 비교해서 오차가 크면 같은 목표로 재계획.
        - 일치하면 HOME↔테이블 인접 규칙으로 다음 목표를 바로 부여.
        """
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None

        s = self.ensure_agent_ctx(ctx, rid)
        vgoal = s.get("verify_goal")
        if not s.get("verifying") or vgoal is None:
            return None

        gp = tag_info.get(rid, {}).get("grid_position")
        gp = tuple(gp) if gp is not None else None

        # 1) 검증 실패 → 같은 목표로 재계획만
        if gp != vgoal:
            if a.goal != vgoal:
                a.goal = vgoal
            print(
                f"[RandomMode] verify fail: rid={rid} gp={gp} vgoal={vgoal} → replan"
            )
            return self.result(replan=True, reason="verify_fail")

        # 2) 검증 성공 → 다음 목표 배정
        s["verifying"] = False
        s["verify_goal"] = None

        home = s.get("home")
        cur = tuple(a.start) if a.start else None
        at_home = bool(home and cur == tuple(home))

        # forbidden 재계산
        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(x.start) for x in agents if x.start}
        goals = {tuple(x.goal) for x in agents if x.goal}
        homes = {
            tuple(self.ensure_agent_ctx(ctx, x.id).get("home"))
            for x in agents
            if self.ensure_agent_ctx(ctx, x.id).get("home")
        }
        forbidden = set(starts) | set(goals) | set(occ) | set(homes)

        ng = self._next_goal_for(a, grid, forbidden, ctx)
        if ng is not None:
            is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
            a.goal = ng

            if at_home:
                if is_new_goal and not s.get("hd_active", False):
                    s["hd_active"] = True
                    s["hd_left"] = random.randint(0, 3)
                a.delay = int(s.get("hd_left", 0))
            else:
                a.delay = 0
                s["hd_active"] = False
                s["hd_left"] = 0

            print(f"[RandomMode] align done → next goal {ng}, delay={a.delay}")
            return self.result(replan=True, reason="align_next_goal")
        else:
            a.goal = None
            print(f"[RandomMode] align done but no next goal, rid={rid}")
            return self.result(replan=True, reason="align_no_goal")

    def on_robot_complete(
        self,
        rid: int,
        *,
        tag_info,
        grid,
        agents,
        ctx,
        runstate,
    ):
        """
        개별 로봇 완료 콜백.
        - TestMode에서 '한 로봇이 먼저 끝났을 때'와 동일 타이밍으로
          다음 HOME↔테이블 인접 목표를 바로 부여.
        """
        a = next((x for x in agents if x.id == rid), None)
        if not a or not a.start:
            return None

        s = self.ensure_agent_ctx(ctx, rid)
        if s.get("homeless"):
            return None

        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(x.start) for x in agents if x.start}
        goals = {tuple(x.goal) for x in agents if x.goal}
        homes = {
            tuple(self.ensure_agent_ctx(ctx, x.id).get("home"))
            for x in agents
            if self.ensure_agent_ctx(ctx, x.id).get("home")
        }
        forbidden = set(starts) | set(goals) | set(occ) | set(homes)

        ng = self._next_goal_for(a, grid, forbidden, ctx)
        if ng is not None:
            is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
            a.goal = ng

            cur = tuple(a.start) if a.start else None
            at_home = bool(s.get("home") and cur == tuple(s["home"]))
            if at_home:
                if is_new_goal and not s.get("hd_active", False):
                    s["hd_active"] = True
                    s["hd_left"] = random.randint(0, 3)
                a.delay = int(s.get("hd_left", 0))
            else:
                a.delay = 0
                s["hd_active"] = False
                s["hd_left"] = 0

            return self.result(replan=True, reason="robot_done")
        else:
            a.goal = None
            return self.result(replan=True, reason="robot_done_no_goal")

    # ------------------------- BaseMode bridging ------------------------- #
    # NOTE:
    #  - 실제 프로젝트에서는 BaseMode를 상속해서 이 함수들이 이미
    #    정의되어 있을 수 있음.
    #  - 단독 실행/테스트 시에도 돌아가도록 여기서 최소구현을 넣어둠.
    def ensure_agent_ctx(self, ctx: Dict[int, dict], rid: int) -> dict:  # pragma: no cover
        if rid not in ctx:
            ctx[rid] = {}
        return ctx[rid]

    def occupied_from_tags(self, tag_info: dict) -> Set[Cell]:  # pragma: no cover
        # 실제 프로젝트에서는 status=="On" 인 태그 위치만 사용할 수 있도록
        # ScenarioManager 또는 BaseMode 쪽 구현을 쓰는 것이 더 안전함.
        return set(tag_info.get("occupied", []))

    def result(self, **kwargs):  # pragma: no cover
        # ScenarioManager 쪽 ModeResult 규격과 맞게만 쓰면 됨.
        return kwargs

    def exit(self, *, tag_info, grid, agents, ctx, runstate):
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["hd_active"] = False
            s["hd_left"] = 0
            s["init_done"] = False   # 다음에 들어오면 다시 초기화
            try:
                a.delay = 0
                a.goal = None        # ★ 다음 모드가 재설정하게끔 비워줌
            except Exception:
                pass

