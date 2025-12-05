
from __future__ import annotations

import random
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# 타입
Cell = Tuple[int, int]


class RandomMode:
    def __init__(
        self,
        *,
        idle_threshold_frames: int = 12,
        home_provider: Optional[callable] = None,
    ):
        self.idle_thresh = idle_threshold_frames
        self.home_provider = home_provider
        # 주문 저장용
        self.order_seq = {1: 0, 2: 0, 3: 0}   
        self.orders = {1: [], 2: [], 3: []}  

    # ----------------------------- Lifecycle ----------------------------- #
    def enter(self, *, tag_info, grid, agents, ctx, runstate):
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)

            # HOME 설정
            home_from_main = self.home_provider(a.id) if self.home_provider else None
            if home_from_main is not None:
                s["home"] = tuple(home_from_main)
                s["homeless"] = False
            else:
                s["home"] = None
                s["homeless"] = True


            a.goal = None
            try:
                a.delay = 0
            except Exception:
                pass


            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["last_pos"] = tuple(a.start) if a.start else None
            s["idle_frames"] = 0

            
            s["hd_active"] = False
            s["hd_left"] = 0

            
            s["init_done"] = False

    # ------------------------------ Helpers ------------------------------ #
    def _neighbors4(self, r: int, c: int, H: int, W: int):
        
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W:
                yield rr, cc

    def _pick_table_adjacent(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:
        H, W = grid.shape
        tables_adj: List[List[Cell]] = []

        for r in range(H):
            for c in range(W):
                if grid[r, c] == 0:
                    continue 
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
        s = self.ensure_agent_ctx(ctx, a.id)
        if s.get("homeless"):
            return None

        cur = tuple(a.start) if a.start else None
        home = s.get("home")

        if home and cur == tuple(home):
            
            return self._pick_table_adjacent(grid, forbidden)
        else:
            
            return tuple(home) if home else None

    
    def tick(self, *, tag_info, grid, agents, ctx, runstate):
        replan = False
        waiters: Set[int] = set()
        waiter_cells: Set[Cell] = set()

       
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") and a.start:
                waiter_cells.add(tuple(a.start))
                waiters.add(a.id)

       
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
                
                is_idle_now = True
            else:
                is_idle_now = (executing is False) or (
                    executing is None and s["idle_frames"] >= self.idle_thresh
                )

            arrived = bool(a.start and a.goal and tuple(a.start) == tuple(a.goal))
            if s.get("verifying") or arrived:
                continue

        
            if is_idle_now and not a.goal:
                forbidden = set(starts) | set(goals) | set(occ) | set(homes)
                ng = self._next_goal_for(a, grid, forbidden, ctx)

                if ng is not None:
                    is_new_goal = (tuple(ng) != tuple(a.goal)) if a.goal else True
                    a.goal = ng
                    self._record_order(a, start=cur, goal=ng, status="ASSIGNED")
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

   
    def on_sequence_complete(
        self,
        *,
        tag_info: dict,
        grid: np.ndarray,
        agents: List,
        ctx: Dict[int, dict],
        runstate: Dict[int, dict],
    ):
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
            print(
                f"[RandomMode] verify fail: rid={rid} gp={gp} vgoal={vgoal} → replan"
            )
            return self.result(replan=True, reason="verify_fail")

        
        s["verifying"] = False
        s["verify_goal"] = None

        home = s.get("home")
        cur = tuple(a.start) if a.start else None
        at_home = bool(home and cur == tuple(home))

        
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

    # BaseMode#
    def ensure_agent_ctx(self, ctx: Dict[int, dict], rid: int) -> dict:
        if rid not in ctx:
            ctx[rid] = {}
        return ctx[rid]

    def occupied_from_tags(self, tag_info: dict) -> Set[Cell]:
        return set(tag_info.get("occupied", []))

    def result(self, **kwargs):  
        return kwargs

    def exit(self, *, tag_info, grid, agents, ctx, runstate):
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            s.pop("verifying", None)
            s.pop("verify_goal", None)
            s["hd_active"] = False
            s["hd_left"] = 0
            s["init_done"] = False   
            try:
                a.delay = 0
                a.goal = None       
            except Exception:
                pass

    def _record_order(self, a, start, goal, status="ASSIGNED"):
        rid = a.id
        self.order_seq[rid] += 1

        self.orders[rid].append({
            "order_id": self.order_seq[rid],
            "start": start,
            "goal": goal,
            "status": status
        })

    

