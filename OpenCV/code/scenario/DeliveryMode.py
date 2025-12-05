# DeliveryMode.py
from __future__ import annotations
import numpy as np
import random
from typing import Dict, Tuple, Set, List, Optional

from OpenCV.code.ui_bridge import FrameBus


Cell = Tuple[int, int]
RobotId = int


class DeliveryMode:
    def __init__(self):
        self._candidate_goals: Set[Cell] = set()
        self._assigned_goals: Dict[RobotId, Cell] = {}

    # ------------------------------------------------------------
    # 1) 모드 초기화
    # ------------------------------------------------------------
    def init(self, grid_state: np.ndarray, home_positions: Dict[int, Cell]):
        self._candidate_goals = self._compute_candidate_goals(grid_state)
        FrameBus.set_delivery_candidate_goals(self._candidate_goals)

        print(f"[DeliveryMode] 후보 목적지 개수 = {len(self._candidate_goals)}")


    
    def _compute_candidate_goals(self, grid: np.ndarray) -> Set[Cell]:
        H, W = grid.shape
        cand = set()

        for r in range(H):
            for c in range(W):

                # 장애물만 기준
                if grid[r][c] != 1:
                    continue

                # 4방향으로 빈칸이면 후보
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < H and 0 <= cc < W and grid[rr][cc] == 0:
                        cand.add((rr, cc))
        
        return cand


    # ------------------------------------------------------------
    # 3) 매 tick 로봇 상태 검사
    # ------------------------------------------------------------
    def tick(self, agent_states: Dict[int, Cell],
             goal_positions: Dict[int, Optional[Cell]],
             runstate: Dict[int, dict]) -> Dict:

        waiters: Set[int] = set()
        ready: Set[int] = set()

        for rid, state in runstate.items():

            executing = state.get("executing", False)
            has_goal  = state.get("has_goal", False)
            goal      = goal_positions.get(rid)

            # ▷ 이동 중이면 패스
            if executing:
                continue

            # ▷ 목적지 있는데 아직 도착 전
            if has_goal and goal is not None:
                continue

            # ════════════════════════
            # ▷ 여기서 IDLE → 새 목적지 지정
            # ════════════════════════
            new_goal = self._pick_random_goal(rid)
            if new_goal:
                ready.add(rid)
                goal_positions[rid] = new_goal
                print(f"[DeliveryMode] Robot{rid} 목표 = {new_goal}")

        # tick() 반환값은 ScenarioManager에 전달됨
        return {
            "waiters": waiters,
            "ready": ready,
        }


    # ------------------------------------------------------------
    # 4) 목표지 선정
    # ------------------------------------------------------------
    def _pick_random_goal(self, rid: int) -> Optional[Cell]:
        if not self._candidate_goals:
            return None
        return random.choice(list(self._candidate_goals))


    # ------------------------------------------------------------
    # 5) 정렬 완료
    # ------------------------------------------------------------
    def align_center_done(self, rid: int):
        pass

    def align_direction_done(self, rid: int):
        pass

    # ------------------------------------------------------------
    # 6) 목적지 도착 후
    # ------------------------------------------------------------
    def robot_arrived(self, rid: int):
        pass

    # ------------------------------------------------------------
    # 7) 시퀀스 종료 처리
    # ------------------------------------------------------------
    def sequence_complete(self, rid: int):
        """도착 시 다음 목표로 바로 돌아가도록"""
        pass
