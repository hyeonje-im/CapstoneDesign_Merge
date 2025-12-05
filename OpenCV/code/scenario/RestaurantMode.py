import random, time
from typing import Dict, List, Optional, Set, Tuple
import json
from pathlib import Path

import os

import numpy as np

import grid

Cell = Tuple[int, int]

class RestaurantMode:
   
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

   
    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None:
        # 상태 리셋
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

        ctx["_init_align_pending"] = set(initial_align_ids)

        self._seed_initial_orders(grid, agents, ctx, tag_info)
        return self.result(
            replan=False,
            align_center=set(initial_align_ids),
            align_direction=set(initial_align_ids),
            reason="enter_init_align_only"
        )

    def _rnd_span(self) -> float:
        return random.uniform(self.span_min, self.span_max)

    def _neighbors4(self, r: int, c: int, H: int, W: int):
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            rr, cc = r+dr, c+dc
            if 0 <= rr < H and 0 <= cc < W:
                yield rr, cc

    def _pick_from_table_meta(self, grid: np.ndarray, forbidden: Set[Cell]) -> Optional[Cell]:

        if not self.table_chairs:
            return None

        H, W = grid.shape
        fbd = set(forbidden)
        taken_dsts = {dst for _, dst in self.order_list} | {dst for _, dst in self.order_start}
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

        import random
        return random.choice(candidates)



    def _home_of(self, ctx: Dict[int, dict], rid: int) -> Optional[Cell]:
        h = self.ensure_agent_ctx(ctx, rid).get("home")
        return tuple(h) if h else None

    def _dir_vec(self, d: str):
       
        return {"n": (-1,0), "e": (0,1), "s": (1,0), "w": (0,-1)}.get(d.lower(), (0,0))

    def _in_bounds(self, r, c, H, W):
        return 0 <= r < H and 0 <= c < W

   
    #ticking
    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        """주문 생성, HOME 상태 유지, 필요 시 CBS 트리거."""
        replan = False
        now = time.time()

        self._reload_table_config(force=False)

        # 주문 생성
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

        # HOME 판정
        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not s.get("home") or not a.start:
                continue
            if tuple(a.start) != tuple(s["home"]):
                self.home_set.discard(a.id)

        
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

        # ready=False
        return self.result(
            replan=True,
            ready_for_order={rid: False},
            reason="order_start"
        )

    #콜백
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List, ctx: Dict[int, dict], runstate: Dict[int, dict]):
        replan = False

        for a in agents:
            s = self.ensure_agent_ctx(ctx, a.id)
            if s.get("homeless") or not a.start:
                continue
            home = self._home_of(ctx, a.id)


            if any((a.id, tuple(a.start)) == (r, dst) for (r, dst) in self.order_start):
                self.order_to_table.discard((a.id, tuple(a.start)))
               
                if home and tuple(a.start) != tuple(home):
                    self.order_to_home.add((a.id, tuple(home)))   # ← 추가
                    a.goal = tuple(home)
                    replan = True
                finished = next(((r, dst) for (r, dst) in self.order_start if r == a.id and tuple(dst)==tuple(a.start)), None)
                if finished:
                    self.order_start.discard(finished)
                    self.order_done.append(finished)
                   
                    rm_idx = next((i for i,(r,dst) in enumerate(self.order_list)
                                   if r==finished[0] and tuple(dst)==tuple(finished[1])), None)
                    if rm_idx is not None:
                        self.order_list.pop(rm_idx)
                    # 자동 HOME 복귀 명령 생성
                    if home and tuple(a.start) != tuple(home):
                        a.goal = tuple(home)
                        replan = True

            # HOME에 도착했는가? → 정렬 요청
            if home and tuple(a.start) == tuple(home):
                self.order_to_home = {p for p in self.order_to_home if p[0] != a.id}

        if replan:
            return self.result(replan=True, reason="arrive_home_replan")
        return None

   
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

       
        if home and a.start and tuple(a.start) != tuple(home):
            a.goal = tuple(home)
            return self.result(replan=True, reason="align_then_home")

      
        if home and a.start and tuple(a.start) == tuple(home):
            self.home_set.add(rid)

          
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
        
        
        if at_table:
            self.order_to_table.discard((rid, tuple(a.start)))
            
            finished = next(((r, dst) for (r, dst) in self.order_start
                             if r == rid and tuple(dst) == tuple(a.start)), None)
            if finished:
                self.order_start.discard(finished)
                self.order_done.append(finished)
             
                rm_idx = next((i for i,(r,dst) in enumerate(self.order_list)
                               if r==finished[0] and tuple(dst)==tuple(finished[1])), None)
                if rm_idx is not None:
                    self.order_list.pop(rm_idx)

            if home and tuple(a.start) != tuple(home):
                a.goal = tuple(home)
                self.order_to_home.add((rid, tuple(home)))
                return self.result(replan=True, reason="table_done_go_home")
            return None 
        
       
        if at_home:
            return self.result(
                replan=False,
                align_center={rid},
                align_direction={rid},
                ready_for_order={rid: True},
                reason="home_first_arrival_align",
                align_delay_sec=0.5
            )

       
        return None

    # BaseMode bridging 
    def ensure_agent_ctx(self, ctx: Dict[int, dict], rid: int) -> dict: 
        if rid not in ctx:
            ctx[rid] = {}
        return ctx[rid]

    def occupied_from_tags(self, tag_info: dict) -> Set[Cell]:  
        return set(tag_info.get("occupied", []))

    def result(self, **kwargs):  
        return kwargs

    def exit(self, **kwargs):  
        pass

    def export_ui_state(self, clear_new: bool = False):
        import time
        state = {
            # 시각화 전용
            "order_new": self.order_new,               
            "order_list": list(self.order_list),       
            "order_done": list(self.order_done),       

            # 실행중 상태 
            "order_to_table": list(self.order_to_table),   
            "order_to_home":   list(self.order_to_home),    
            "home_set":       list(self.home_set),        

            "next_order_eta": max(0.0, self._next_order_at - time.time()),
        }
        if clear_new:
            self.order_new = None
        return state
    
    def _seed_initial_orders(self, grid, agents, ctx, tag_info):
        
        H, W = grid.shape
        
        starts = {tuple(a.start) for a in agents if a.start}
        goals  = {tuple(a.goal) for a in agents if a.goal}
        homes  = {tuple(self._home_of(ctx, a.id)) for a in agents if self._home_of(ctx, a.id)}
        occ    = self.occupied_from_tags(tag_info)
        taken  = {dst for _, dst in self.order_list} | {dst for _, dst in self.order_start}
        forbidden_base = set(starts) | set(goals) | set(occ) | set(homes)
       
        visible_ids = [a.id for a in agents if a.start]
        for rid in visible_ids:
            
            if any(r == rid for (r, _) in self.order_list):
                continue
          
            forbidden = set(forbidden_base) | {dst for _, dst in self.order_list}
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
            print(f"[RestaurantMode] WARNING: table_coords.json not found at {self._cfg_path}")
            return []
        except Exception as e:
            print(f"[RestaurantMode] ERROR loading table_coords.json: {e}")
            return []

    def _reload_table_config(self, force: bool = False):
       
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

            pos = self.manager.get_robot_position(rid) 
            goal = self.manager.get_robot_goal(rid)

            ui[rid] = {
                "num": f"#{rid}",
                "pos": str(pos) if pos else "-",
                "goal": str(goal) if goal else "-",
                "status": st
            }

        
        if drain_new:
            self.new_orders = []
        
        return ui