from __future__ import annotations
from typing import Dict, List, Tuple, Set, Optional
import numpy as np
from OpenCV.code.cbs.pathfinder import Agent
from OpenCV.code.scenario.ScenarioManager import BaseMode, ModeResult

Cell = Tuple[int, int]
RobotId = int

class TestMode(BaseMode):
    name = "TestMode"
   

    name = "TestMode"

    def __init__(self, *, idle_threshold_frames: int = 15):
        self.idle_thresh = idle_threshold_frames

 

    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
              ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
      
        for a in agents:
            self.ensure_agent_ctx(ctx, a.id)

    def exit(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
     
        pass

    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None:
        for a in agents:
            self.ensure_agent_ctx(ctx, a.id)

      
        if any(not ctx.get(a.id, {}).get("init_done", False) for a in agents):
            self._assign_initial_random_goals(grid, agents, tag_info)
            for a in agents:
                ctx[a.id]["init_done"] = True
            return self.result(replan=True, reason="init") 

      
        occ = self.occupied_from_tags(tag_info)                     
        starts = {tuple(a.start) for a in agents if a.start}
        goals  = {tuple(a.goal)  for a in agents if a.goal}

        replan = False
        waiters: Set[int] = set()
        waiter_cells: Set[Cell] = set()

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            
            cur = tuple(a.start) if a.start else None
            last = s.get("last_pos")
            s["idle_frames"] = (s.get("idle_frames", 0) + 1) if (cur is not None and last == cur) else 0
            s["last_pos"] = cur

            executing = (runstate.get(a.id) or {}).get("executing", None)
            is_idle_now = (executing is False) or (executing is None and s["idle_frames"] >= self.idle_thresh)

            if is_idle_now and (not a.goal or (a.start and a.goal and tuple(a.start) == tuple(a.goal))):
               
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
      
            if a.goal != vgoal:
                a.goal = vgoal
            return self.result(replan=True, reason="verify_miss")

        
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
          
            a.goal = None
            waiters, waiter_cells = set(), set()
            if a.start:
                waiters.add(a.id); waiter_cells.add(tuple(a.start))
            return self.result(replan=False, waiters=waiters, waiter_cells=waiter_cells, reason="no_goal")

    def on_robot_complete(self, rid: int, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
      
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
        
            return self.result(replan=True, reason="robot_done_no_goal")

    

    def _assign_initial_random_goals(self, grid: np.ndarray, agents: List[Agent], tag_info: dict) -> None:
        occ = self.occupied_from_tags(tag_info)
        starts = {tuple(a.start) for a in agents if a.start}
        goals: Set[Cell] = set()
        for a in agents:
            ng = self.sample_free_goal(grid, starts | goals | occ)
            a.goal = ng if ng is not None else a.start 
            if ng is not None:
                goals.add(tuple(ng))

    
    def export_ui_state(self, agents, ctx, runstate):
       
        state = {}

        for a in agents:
            rid = a.id

            pos  = tuple(a.start) if a.start else None
            goal = tuple(a.goal) if a.goal else None

            status = self.compute_robot_status(rid, a, ctx, runstate)

            state[rid] = {
                "num": "-",  
                "pos": pos,
                "goal": goal,
                "status": status,
            }

        return state
    
    
    def compute_robot_status(self, rid, agent, ctx, runstate):
        rs = runstate.get(rid) or {}

        executing = rs.get("executing", None)
        start = agent.start
        goal  = agent.goal

        
        if goal is None:
            return "IDLE"

       
        if start and goal and tuple(start) == tuple(goal):
            return "ARRIVED"

        
        if executing:
            return "MOVING"

        
        return "MOVING"

