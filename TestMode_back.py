# test_mode.py
from __future__ import annotations
import random
from typing import Dict, List, Tuple, Set, TypedDict, Optional
import time
import numpy as np
from cbs.pathfinder import Agent  # 프로젝트의 Agent 사용

Cell = Tuple[int, int]
RobotId = int

# ScenarioManager의 ModeResult와 동일 구조로 맞춤
class ModeResult(TypedDict, total=False):
    replan: bool
    reason: str                  # "done" | "timer" | "idle" | "init"
    waiters: Set[RobotId]
    ready: Set[RobotId]
    waiter_cells: Set[Cell]
    align_center: Set[RobotId]
    align_direction: Set[RobotId]

class TestMode:
    """
    요구사항:
    - 시작 시 모든 로봇에 대해 장애물이 아닌 셀 중에서 무작위 목표를 부여 (출발지/다른 목표와 중복 금지)
    - 어떤 로봇이 목표에 도착하면 0~3 step 딜레이 후 새 무작위 목표 부여 (CBS 재실행)
    - 명령이 없는 상태에서 몇 프레임 동안 정지해 있으면 새 목표 부여 (CBS 재실행)
    - CBS 실행/명령 전송은 ScenarioManager가 담당 (여기는 replan 신호/웨이터 집합만 반환)
    """

    def __init__(self, *, frames_per_step: int = 10, idle_threshold_frames: int = 15, delay_max_steps: int = 3):
        self.frames_per_step = frames_per_step
        self.idle_thresh = idle_threshold_frames
        self.delay_max_steps = delay_max_steps

    # --------- IMode 표준 메서드들 ---------

    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
              ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # per-agent 컨텍스트 기본 필드 초기화
        for a in agents:
            s = ctx.setdefault(a.id, {})
            s.setdefault("init_done", False)
            s.setdefault("idle_frames", 0)
            s.setdefault("last_pos", None)
            if getattr(a, "delay", None) is None:
                a.delay = 0
            s.setdefault("verifying", False)
            s.setdefault("verify_goal", None)
            s.setdefault("verify_ok_since", None) 

    def exit(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # 필요한 정리는 없음
        pass

    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult:
        for a in agents:
            s = ctx.setdefault(a.id, {})
            s.setdefault("init_done", False)
            s.setdefault("idle_frames", 0)
            s.setdefault("last_pos", None)
        
        waiters: Set[int] = set()
        waiter_cells: Set[Cell] = set()
        replan = False
        reason = "idle"

        # 0) 초기 목표 없으면 한 번에 전부 부여
        init_needed = False
        for a in agents:
            s = ctx[a.id]
            if not s.get("init_done", False):
                init_needed = True
                break
        if init_needed:
            self._assign_initial_random_goals(grid, agents, tag_info)
            for a in agents:
                ctx[a.id]["init_done"] = True
            return ModeResult(replan=True, reason="init")

        # 2) “명령 없음 + 정지” 감지 → 새 목표 부여
        # runstate[rid]["executing"] 이 None이면 컨트롤러가 정보를 안 준 것이고,
        # 이 경우 태그 위치 기반 idle만으로 판단.
        occupied_now = _occupied_from_tags(tag_info)
        starts_set = {tuple(a.start) for a in agents if a.start}
        goals_set = {tuple(a.goal) for a in agents if a.goal}

        for a in agents:
            s = ctx[a.id]

            # 위치 기반 idle 측정
            current_pos = tuple(a.start) if a.start else None
            last_pos = s.get("last_pos")
            if current_pos is not None and last_pos is not None and current_pos == last_pos:
                s["idle_frames"] += 1
            else:
                s["idle_frames"] = 0
            s["last_pos"] = current_pos

            executing = runstate.get(a.id, {}).get("executing", None)
            no_cmd_or_idle = (executing is False) or (executing is None and s["idle_frames"] >= self.idle_thresh)

            # “명령 없음 + 정지”이고, 목표가 없거나(또는 start==goal) 사실상 멈춰 있다면 새 목표 부여
            if no_cmd_or_idle and (not a.goal or (a.start and a.goal and tuple(a.start) == tuple(a.goal))):
                new_goal = _sample_random_free_goal(grid, starts_set, goals_set | occupied_now)
                if new_goal is not None:
                    a.goal = new_goal
                    # 목표 갱신되면 재계획
                    replan = True
                    reason = "idle"
                    goals_set.add(tuple(new_goal))
                else:
                    if a.start:
                        waiters.add(a.id)
                        waiter_cells.add(tuple(a.start))

        out: ModeResult = {"replan": replan, "reason": reason}
        if waiters:
            out["waiters"] = waiters
        if waiter_cells:
            out["waiter_cells"] = waiter_cells
        return out

    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
                             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult:
        """
        컨트롤러 DONE 수신 시:
        - 목표에 도착한 로봇(start==goal)들에 대해 0~3 step 딜레이 부여
        - 딜레이 후 새 무작위 목표를 뽑도록 준비 (대기 만료 틱에서 replan)
        """
        waiters: Set[int] = set()
        waiter_cells: Set[Cell] = set()
        align_center: Set[int] = set()
        align_direction: Set[int] = set()

        occupied_now = _occupied_from_tags(tag_info)
        starts_set = {tuple(a.start) for a in agents if a.start}
        goals_set = {tuple(a.goal) for a in agents if a.goal}

        # 도착한 로봇 찾아 딜레이 부여
        for a in agents:
            s = ctx[a.id]
            if a.start and a.goal and tuple(a.start) == tuple(a.goal):
                # 검증 모드로 진입
                s["verifying"] = True
                s["verify_goal"] = tuple(a.goal)
                # (2) 해당 로봇 '만' 정렬 명령을 요청 (중앙 → 방향)
                align_center.add(a.id)
                align_direction.add(a.id)

        out: ModeResult = {"replan": False, "reason": "done"}
        if waiters:
            out["waiters"] = waiters
        if waiter_cells:
            out["waiter_cells"] = waiter_cells
        if align_center:
            out["align_center"] = align_center
        if align_direction: 
            out["align_direction"] = align_direction
        return out

    # --------- 내부 유틸 ---------

    def _assign_initial_random_goals(self, grid: np.ndarray, agents: List[Agent], tag_info: dict) -> None:
        occupied_now = _occupied_from_tags(tag_info)
        starts_set = {tuple(a.start) for a in agents if a.start}
        goals_set: Set[Cell] = set()

        for a in agents:
            g = _sample_random_free_goal(grid, starts_set, goals_set | occupied_now)
            a.goal = g if g is not None else a.start  # 실패하면 대기
            if g is not None:
                goals_set.add(tuple(g))

    def on_alignment_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate) -> ModeResult:
        # 1) 대상 agent / 검증 상태 확인
        a = next((x for x in agents if x.id == rid), None)
        if not a: return {"replan": False, "reason": "align_complete"}
        s = ctx.get(rid, {})
        vgoal = s.get("verify_goal")
        if not s.get("verifying") or vgoal is None:
            return {"replan": False, "reason": "align_complete"}

        # 2) 비전 좌표와 목표칸 비교 (이 타이밍 단 한 번만)
        gp = tag_info.get(rid, {}).get("grid_position")
        gp = tuple(gp) if gp is not None else None

        waiters, waiter_cells = set(), set()
        starts_set = {tuple(x.start) for x in agents if x.start}
        goals_set  = {tuple(x.goal)  for x in agents if x.goal}

        if gp != vgoal:
            # ── 회복: 동일 목표로 CBS 재계획
            if a.goal != vgoal:
                a.goal = vgoal
            return {"replan": True, "reason": "verify_miss"}
        else:
            # ── 진짜 도착 확정: 검증 종료 후 새 목표
            s["verifying"] = False
            s["verify_goal"] = None
            new_goal = _sample_random_free_goal(grid, starts_set, goals_set | _occupied_from_tags(tag_info))
            if new_goal is not None:
                a.goal = new_goal
                return {"replan": True, "reason": "verified"}
            else:
                # 목적지 못 만들면 waiter (임시 장애물)
                a.goal = None
                if a.start:
                    waiters.add(a.id); waiter_cells.add(tuple(a.start))
                return {"replan": False, "reason": "no_goal", "waiters": waiters, "waiter_cells": waiter_cells}

        # TestMode.py
    def on_robot_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate):
        a = next((x for x in agents if x.id == rid), None)
        if not a:
            return {"replan": False, "reason": "no-agent"}

        # 정책: 완주한 로봇에게는 새 목표를 즉시 부여하되,
        # 동기화를 위해 '전체 재계획'을 요청
        starts = {tuple(x.start) for x in agents if x.start}
        goals  = {tuple(x.goal)  for x in agents if x.goal}
        new_goal = _sample_random_free_goal(grid, starts, goals | _occupied_from_tags(tag_info))
        if new_goal is not None:
            a.goal = new_goal
            return {"replan": True, "reason": "robot_done"}   # ready 미지정 → 전체
        else:
            # 새 목표 못 만들면 일단 보류
            a.goal = None
            return {"replan": True, "reason": "robot_done_no_goal"}  # waiter로 자동 편입됨


# ---- 모드 내부에서 쓰는 작은 헬퍼들 ----

def _occupied_from_tags(tag_info: dict) -> Set[Cell]:
    occ: Set[Cell] = set()
    for _, data in tag_info.items():
        gp = data.get("grid_position")
        if gp:
            occ.add(tuple(gp))
    return occ

def _sample_random_free_goal(grid: np.ndarray, starts: Set[Cell], blocked: Set[Cell]) -> Optional[Cell]:
    """grid==0인 모든 좌표 중에서 (starts ∪ blocked)에 없는 셀을 무작위로 하나 선택."""
    H, W = grid.shape
    candidates: List[Cell] = []
    for r in range(H):
        for c in range(W):
            if grid[r, c] == 0:
                cell = (r, c)
                if cell not in starts and cell not in blocked:
                    candidates.append(cell)
    if not candidates:
        return None
    return random.choice(candidates)