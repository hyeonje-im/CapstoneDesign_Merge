from __future__ import annotations
from typing import Dict, List, Tuple, Set, Optional
import numpy as np
from cbs.pathfinder import Agent
from ScenarioManager import BaseMode, ModeResult

Cell = Tuple[int, int]
RobotId = int

class TestMode(BaseMode):
    """
    거동 원칙:
    - 초기/유휴 시 목표가 없으면 무작위 목표를 부여(출발/다른 목표/점유와 비겹침)
    - 도착으로 보이면 정렬(중앙→방향)→비전 단발 검증→불일치면 동일 목표로 복귀 재계획,
      일치면 새 목표 부여 후 재계획
    - CBS 실행/명령 전송은 ScenarioManager 담당 (여기는 replan 신호/웨이터 집합만 반환)
    """

    name = "TestMode"

    def __init__(self, *, idle_threshold_frames: int = 15):
        self.idle_thresh = idle_threshold_frames

    # ---------- IMode 표준 훅 ----------

    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
              ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # per-agent 컨텍스트 초기화
        for a in agents:
            self.ensure_agent_ctx(ctx, a.id)

    def exit(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # 특별 정리 없음
        pass

    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None:
        for a in agents:
            self.ensure_agent_ctx(ctx, a.id)

        # 0) 초기 목표 없으면 한 번에 전부 부여
        if any(not ctx.get(a.id, {}).get("init_done", False) for a in agents):
            self._assign_initial_random_goals(grid, agents, tag_info)
            for a in agents:
                ctx[a.id]["init_done"] = True
            return self.result(replan=True, reason="init")  # 즉시 CBS

        # 1) “명령 없음 + 정지(idle)” 로봇에 새 목표 부여
        occ = self.occupied_from_tags(tag_info)                     # 비전 점유 셀  :contentReference[oaicite:3]{index=3}
        starts = {tuple(a.start) for a in agents if a.start}
        goals  = {tuple(a.goal)  for a in agents if a.goal}

        replan = False
        waiters: Set[int] = set()
        waiter_cells: Set[Cell] = set()

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            # 위치 기반 idle 프레임 누적
            cur = tuple(a.start) if a.start else None
            last = s.get("last_pos")
            s["idle_frames"] = (s.get("idle_frames", 0) + 1) if (cur is not None and last == cur) else 0
            s["last_pos"] = cur

            executing = (runstate.get(a.id) or {}).get("executing", None)
            is_idle_now = (executing is False) or (executing is None and s["idle_frames"] >= self.idle_thresh)

            if is_idle_now and (not a.goal or (a.start and a.goal and tuple(a.start) == tuple(a.goal))):
                # 금지 셀: 모든 start/goal/점유(안전)
                forbidden = set(starts) | set(goals) | set(occ)
                ng = self.sample_free_goal(grid, forbidden)         # 무작위 유효 셀  :contentReference[oaicite:4]{index=4}
                if ng is not None:
                    a.goal = ng
                    goals.add(tuple(ng))
                    replan = True
                else:
                    if a.start:
                        waiters.add(a.id)
                        waiter_cells.add(tuple(a.start))

        if replan or waiters or waiter_cells:
            return self.result(replan=replan, waiters=waiters, waiter_cells=waiter_cells, reason="idle")
        return None

    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
                             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None:
        # 도착 로봇 선별 → 해당 로봇만 정렬(중앙/방향)
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
            return self.result(replan=False, align_center=align_center, align_direction=align_direction, reason="done")
        return None

    def on_alignment_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
        # 단발 검증: 정렬 직후 “정말 그 칸인가?”
        
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        s = self.ensure_agent_ctx(ctx, rid)
        vgoal = s.get("verify_goal")
        if not s.get("verifying") or vgoal is None:
            return None

        gp = tag_info.get(rid, {}).get("grid_position")
        gp = tuple(gp) if gp is not None else None

        if gp != vgoal:
            # 불일치 → 동일 목표로 복귀 재계획
            if a.goal != vgoal:
                a.goal = vgoal
            return self.result(replan=True, reason="verify_miss")

        # 일치 → 검증 종료 후 새 목표 부여
        s["verifying"] = False
        s["verify_goal"] = None

        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(x.start) for x in agents if x.start}
        goals  = {tuple(x.goal)  for x in agents if x.goal}
        forbidden = starts | goals | occ

        ng = self.sample_free_goal(grid, forbidden)
        if ng is not None:
            a.goal = ng
            return self.result(replan=True, reason="verified")
        else:
            # 목표를 못 만들면 웨이터로 (임시 장애물)
            a.goal = None
            waiters, waiter_cells = set(), set()
            if a.start:
                waiters.add(a.id); waiter_cells.add(tuple(a.start))
            return self.result(replan=False, waiters=waiters, waiter_cells=waiter_cells, reason="no_goal")

    def on_robot_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
        # 정책: 개별 완주 즉시 새 목표 부여 시도 → 전체 재계획(ready 생략)
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return None
        s = self.ensure_agent_ctx(ctx, rid)
        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(x.start) for x in agents if x.start}
        goals  = {tuple(x.goal)  for x in agents if x.goal}
        ng = self.sample_free_goal(grid, starts | goals | occ)
        if ng is not None:
            a.goal = ng
            return self.result(replan=True, reason="robot_done")
        else:
            a.goal = None
            # 웨이터 편입은 on_sequence_complete 경계에서 자동 처리됨
            return self.result(replan=True, reason="robot_done_no_goal")

    # ---------- 내부 유틸 ----------

    def _assign_initial_random_goals(self, grid: np.ndarray, agents: List[Agent], tag_info: dict) -> None:
        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(a.start) for a in agents if a.start}
        goals: Set[Cell] = set()
        for a in agents:
            ng = self.sample_free_goal(grid, starts | goals | occ)
            a.goal = ng if ng is not None else a.start  # 실패하면 대기(= start)
            if ng is not None:
                goals.add(tuple(ng))
