# ScenarioManager.py (FULL PATCHED VERSION)
from __future__ import annotations
from typing import Protocol, TypedDict, Dict, List, Set, Tuple, Optional, Callable
import numpy as np
from OpenCV.code.cbs.pathfinder import PathFinder, Agent
from OpenCV.code.ui_bridge import FrameBus
import threading
import time, random

Cell = Tuple[int, int]
RobotId = int


# ---- ModeResult 표준 ----
class ModeResult(TypedDict, total=False):
    replan: bool
    reason: str
    waiters: Set[RobotId]
    ready: Set[RobotId]
    waiter_cells: Set[Cell]
    align_center: Set[RobotId]
    align_direction: Set[RobotId]


# ---- 모드 인터페이스 ----
class IMode(Protocol):
    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
              ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None: ...
    def exit(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> None: ...
    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None: ...
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None: ...


# =============== ScenarioManager (Fixed) ==================
class ScenarioManager:
    """
    단일 책임:
    - 태그/맵 읽어서 모드 tick 전달
    - 모드가 replan 요청하면 CBS 실행
    - 컨트롤러 콜백 관리
    - UI에 로봇 상태 패킷 전달
    """

    def __init__(
        self,
        *,
        controller,
        agents_ref: List[Agent],
        paths_ref: List[List[Cell]],
        get_grid: Callable[[], np.ndarray],
        get_tag_info: Callable[[], dict],
        path_to_commands: Callable[[List[Cell], int], List[dict]],
        get_initial_hd: Callable[[RobotId], int],
        pathfinder_factory: Optional[Callable[[np.ndarray], PathFinder]] = None,
        mode: IMode
    ):
        self.controller = controller
        self.agents_ref = agents_ref
        self.paths_ref = paths_ref
        self.get_grid = get_grid
        self.get_tag_info = get_tag_info
        self.path_to_commands = path_to_commands
        self.get_initial_hd = get_initial_hd
        self.pathfinder_factory = pathfinder_factory or (lambda grid: PathFinder(grid))

        self.controller.set_alignment_completion_callback(self.on_align_complete)
        self.controller.set_robot_completion_callback(self.on_robot_complete)
        self.controller.set_sequence_completion_callback(self.on_sequence_complete)

        self.mode: IMode = mode
        self.enabled: bool = False
        self.ctx: Dict[int, dict] = {}

        self._sync_starts_from_tags()
        rs = self._build_runstate()
        self.mode.enter(tag_info=self.get_tag_info(), grid=self.get_grid(),
                        agents=self.agents_ref, ctx=self.ctx, runstate=rs)

        self._active_step_plan: Dict[int, Dict[str, Dict[str, Cell]]] = {}
        self._active_step_count: int = 0
        self._initial_hd_hint: Dict[int, int] = {}

        self._replan_requested = False
        self._replanning = False

        # ⭐ ARRIVED 감지 플래그 + 최신 runstate 저장
        self._sequence_done_flag: Dict[int, bool] = {}
        self._last_runstate: Dict[int, dict] = {}

        # ⭐ 주문 히스토리 패킷 관리
        self._pending_orders = []


    # --------------------------------------------------------
    def set_mode(self, mode: IMode):
        rs = self._build_runstate()
        self.mode.exit(tag_info=self.get_tag_info(), grid=self.get_grid(),
                       agents=self.agents_ref, ctx=self.ctx, runstate=rs)
        self.mode = mode
        self.ctx.clear()
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        self.mode.enter(tag_info=self.get_tag_info(), grid=self.get_grid(),
                        agents=self.agents_ref, ctx=self.ctx, runstate=rs)

    # --------------------------------------------------------
    def set_enabled(self, on: bool):
        self.enabled = bool(on)
        print(f"[Scenario] {'ENABLED' if self.enabled else 'PAUSED'}")

    # --------------------------------------------------------
    def toggle_enabled(self):
        self.set_enabled(not self.enabled)

    # --------------------------------------------------------
    def on_align_complete(self, rid: str):
        if not self.enabled:
            return
        self._sync_starts_from_tags()
        rs = self._build_runstate()

        if hasattr(self.mode, "on_alignment_complete"):
            res = self.mode.on_alignment_complete(int(rid),
                        tag_info=self.get_tag_info(), grid=self.get_grid(),
                        agents=self.agents_ref, ctx=self.ctx, runstate=rs)
            if res and res.get("replan"):
                self._plan_and_send(self.get_grid(), res)

    # --------------------------------------------------------
    def tick(self):
        if not self.enabled:
            return

        self._sync_starts_from_tags()
        grid = self.get_grid()
        tag = self.get_tag_info()

        # ⭐ runstate 저장 (status 계산용)
        runstate = self._build_runstate()
        self._last_runstate = runstate

        res = self.mode.tick(tag_info=tag, grid=grid,
                             agents=self.agents_ref, ctx=self.ctx,
                             runstate=runstate)

        if res:
            ac = res.get("align_center") or set()
            ad = res.get("align_direction") or set()
            targets = sorted(list(ac | ad))

            if targets:
                self.controller.run_align_sequence(targets, do_release=False)

            if res.get("replan"):
                if getattr(self.controller, "active", False):
                    self.controller.request_pause_on_step_boundary()
                    self._replan_requested = True
                    print("[Scenario] replan requested → deferred")
                else:
                    print("[Scenario] replan requested → immediate")
                    self._sync_starts_from_tags()
                    self._plan_and_send(self.get_grid(), res)

            # ⭐ UI 패킷 생성
            self.export_robot_ui_state(res)

        # ⭐ ARRIVED 플래그 1프레임 유지 후 즉시 리셋
        for rid in list(self._sequence_done_flag.keys()):
            self._sequence_done_flag[rid] = False

    # --------------------------------------------------------
    def on_sequence_complete(self, info=None):
        if not self.enabled:
            return

        self._sync_starts_from_tags()
        rs = self._build_runstate()

        res = self.mode.on_sequence_complete(
            tag_info=self.get_tag_info(), grid=self.get_grid(),
            agents=self.agents_ref, ctx=self.ctx, runstate=rs
        ) or {}

        ac = res.get("align_center") or set()
        ad = res.get("align_direction") or set()
        targets = sorted(list(ac | ad))
        if targets:
            self.controller.run_align_sequence(targets, do_release=False)

        should_replan = self._replan_requested or res.get("replan", False)
        if not should_replan:
            self._replan_requested = False
            return

        if self._replanning:
            return

        self._replanning = True

        def _deferred_replan(grid_snapshot, res_snapshot):
            try:
                self._sync_starts_from_tags()
                last_plan = getattr(self, "_active_step_plan", {}) or {}
                max_step = max(last_plan.keys()) if last_plan else -1

                def _last_dst_for(r):
                    r_s = str(r)
                    for s in range(max_step, -1, -1):
                        info = (last_plan.get(s, {}).get(r_s)) or (last_plan.get(s, {}).get(r))
                        if info and info.get("dst") is not None:
                            return tuple(info["dst"])
                    return None

                tag_info = self.get_tag_info()
                for a in self.agents_ref:
                    visible = ("grid_position" in (tag_info.get(a.id) or {})) and \
                              (tag_info.get(a.id, {}).get("status") == "On")
                    if not visible:
                        ld = _last_dst_for(a.id)
                        if ld:
                            a.start = ld

                self._plan_and_send(grid_snapshot, res_snapshot)

            finally:
                self._replan_requested = False
                self._replanning = False

        grid_now = self.get_grid()
        print("[Scenario] sequence complete → 1.0s delayed replan")
        threading.Timer(1.0, _deferred_replan, args=[grid_now, res]).start()

    # --------------------------------------------------------
    def _sync_starts_from_tags(self):
        tag = self.get_tag_info()
        for a in self.agents_ref:
            data = tag.get(a.id)
            if data and "grid_position" in data:
                a.start = tuple(data["grid_position"])

    # --------------------------------------------------------
    def _plan_and_send(self, grid, res):
        waiters_ids = set(res.get("waiters", set()))
        ready_ids = res.get("ready")
        waiter_cells = set(res.get("waiter_cells", set()))

        inferred_waiters = set()
        for a in self.agents_ref:
            bad = (not a.start) or (not a.goal) or (a.start == a.goal)
            if bad:
                inferred_waiters.add(a.id)
                if a.start:
                    waiter_cells.add(a.start)

        waiters_ids |= inferred_waiters

        aug = grid.copy()
        for (r, c) in waiter_cells:
            if 0 <= r < aug.shape[0] and 0 <= c < aug.shape[1]:
                aug[r, c] = 1

        moving = [a for a in self.agents_ref if (
            a.id not in waiters_ids
            and a.start and a.goal
            and a.start != a.goal
            and (ready_ids is None or a.id in ready_ids)
        )]

        if not moving:
            print("[Scenario] No moving agents.")
            return

        try:
            pf = self.pathfinder_factory(aug)
            solved_agents = pf.compute_paths(moving)
        except Exception as e:
            print(f"[CBS] Error: {e}")
            return

        if not solved_agents:
            print("[CBS] No paths")
            return

        self.paths_ref.clear()
        for sa in solved_agents:
            p = sa.get_final_path()
            if p:
                self.paths_ref.append(p)

        cmd_map: Dict[str, List[str]] = {}
        step_cell_plan: Dict[int, Dict[str, Dict[str, Cell]]] = {}

        for sa in solved_agents:
            path = sa.get_final_path()
            if not path or len(path) < 2:
                continue

            hd0 = self._initial_hd_hint.pop(sa.id, None)
            if hd0 is None:
                hd0 = self.get_initial_hd(sa.id)
            cmd_objs = self.path_to_commands(path, hd0)
            cmds = [c["command"] for c in cmd_objs]

            rid = str(sa.id)
            cmd_map[rid] = cmds

            for i in range(len(path) - 1):
                step_cell_plan.setdefault(i, {})
                step_cell_plan[i][rid] = {"src": tuple(path[i]),
                                          "dst": tuple(path[i+1])}

        if cmd_map:
            print("[Scenario] Sending commands:", {k:v for k,v in cmd_map.items() if v})
            self._active_step_plan = step_cell_plan
            self._active_step_count = (max(step_cell_plan.keys()) + 1)
            self.controller.start_sequence(cmd_map, step_cell_plan=step_cell_plan)

    # --------------------------------------------------------
    def _build_runstate(self) -> Dict[int, dict]:
        tag = self.get_tag_info()
        rs: Dict[int, dict] = {}
        has_is_exec = hasattr(self.controller, "is_executing")

        for a in self.agents_ref:
            rid = a.id
            if has_is_exec:
                try:
                    executing = self.controller.is_executing(rid)
                except Exception:
                    executing = None
            else:
                executing = None

            info = tag.get(rid, {})
            rs[rid] = {
                "executing": executing,
                "has_goal": bool(a.goal),
                "start": a.start,
                "goal": a.goal,
                "tag": info,
                "dir_aligned_recent": (
                    hasattr(self.controller, "aligned_recently")
                    and self.controller.aligned_recently(rid, within_sec=0.3)
                ),
            }
        return rs

    # --------------------------------------------------------
    def on_robot_complete(self, rid: str):
        if not self.enabled:
            return

        rid_int = int(rid)

        # ⭐ 도착 플래그 ON
        self._sequence_done_flag[rid_int] = True

        self._sync_starts_from_tags()
        rs = self._build_runstate()

        if hasattr(self.mode, "on_robot_complete"):
            res = self.mode.on_robot_complete(
                rid_int,
                tag_info=self.get_tag_info(), grid=self.get_grid(),
                agents=self.agents_ref, ctx=self.ctx, runstate=rs
            )
        else:
            res = {"replan": False}

        if res.get("replan"):
            self.controller.request_pause_on_step_boundary()
            self._replan_requested = True

    # ============================================================
    #  UI STATUS 계산 (Status column)
    # ============================================================
    def compute_robot_status(self, rid: int, mode_result: dict) -> str:
        rs = self._last_runstate
        rstate = rs.get(rid, {})

        if mode_result.get("replan", False):
            return "REPLANNING"

        if rid in (mode_result.get("align_center", set()) or set()):
            return "ALIGNING"
        if rid in (mode_result.get("align_direction", set()) or set()):
            return "ALIGNING"

        if rid in (mode_result.get("waiters", set()) or set()):
            return "WAITING"

        if self._sequence_done_flag.get(rid, False):
            return "ARRIVED"

        executing = rstate.get("executing")
        has_goal  = rstate.get("has_goal")
        start     = rstate.get("start")
        goal      = rstate.get("goal")

        if executing and has_goal and start and goal and (tuple(start) != tuple(goal)):
            return "MOVING"

        # RETURNING / AT_HOME optional modes
        if hasattr(self.mode, "order_to_home"):
            if (rid, goal) in (self.mode.order_to_home or set()):
                return "RETURNING"

        if hasattr(self.mode, "home_set"):
            if rid in self.mode.home_set:
                return "AT_HOME"

        return "IDLE"

    # ============================================================
    #  UI PACKET EXPORT
    # ============================================================
    def export_robot_ui_state(self, mode_result: dict):
        from OpenCV.code.ui_bridge import FrameBus

        agents = self.agents_ref
        agent_pos = FrameBus.get_agent_states()
        agent_goal = FrameBus.get_goal_positions()

        ui_state = {}

        for a in agents:
            rid = a.id

            if hasattr(self.mode, "get_current_order_for"):
                num = self.mode.get_current_order_for(rid)
            else:
                num = "-"

            pos  = agent_pos.get(rid, None)
            goal = agent_goal.get(rid, None)
            status = self.compute_robot_status(rid, mode_result)

            ui_state[rid] = {
                "num": num,
                "pos": pos,
                "goal": goal,
                "status": status,
            }

        FrameBus.set_robot_ui_state(ui_state)

    # ============================================================
    def push_order_history(self, rid: int, order_id, goal, status: str):
        ts = time.time()
        self._pending_orders.append({
            "rid": rid,
            "order_id": order_id,
            "goal": goal,
            "status": status,
            "timestamp": ts,
        })

    # ============================================================
    #  UI → 호출용 상태 패키지
    # ============================================================
    def get_mode_ui_state(self, *, drain_new: bool = False) -> dict:

        state = {
            "agents": {},
            "scenario_status": "RUNNING" if self.enabled else "PAUSED",
            "mode": self.mode.name if self.mode else None,
        }

        for a in self.agents_ref:
            rid = a.id
            cur = tuple(a.start) if a.start else None
            goal = tuple(a.goal) if a.goal else None
            rs = self._last_runstate.get(rid, {})
            executing = rs.get("executing")

            state["agents"][rid] = {
                "pos": cur,
                "goal": goal,
                "status": self.compute_robot_status(rid, {}),
            }

        if drain_new:
            state["new_orders"] = list(self._pending_orders)
            self._pending_orders.clear()
        else:
            state["new_orders"] = []

        return state





# ============================================================
# BaseMode
# ============================================================
class BaseMode:
    name = "Base"

    def enter(self, *, tag_info, grid, agents, ctx, runstate): pass
    def exit(self, *, tag_info, grid, agents, ctx, runstate): pass
    def tick(self, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
        return None
    def on_sequence_complete(self, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
        return None
    def on_robot_complete(self, rid, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
        return None
    def on_alignment_complete(self, rid, *, tag_info, grid, agents, ctx, runstate) -> ModeResult | None:
        return None

    def get_agent_ctx(self, ctx: dict, rid: int) -> dict:
        ctx.setdefault("agents", {})
        return ctx["agents"].setdefault(str(rid), {})

    def set_agent_phase(self, ctx: dict, rid: int, phase: str):
        self.get_agent_ctx(ctx, rid)["phase"] = phase

    def is_idle(self, runstate: dict, rid: int, frames: int = 8) -> bool:
        st = runstate.get(rid) or runstate.get(str(rid))
        if not st: return False
        return (st.get("executing") is False) and (st.get("idle_frames", 999) >= frames)

    def occupied_from_tags(self, tag_info: dict) -> set[Tuple[int,int]]:
        occ = set()
        for rid, dat in tag_info.items():
            gp = dat.get("grid_position")
            if gp and dat.get("status") == "On":
                occ.add((gp[0], gp[1]))
        return occ

    def collect_forbidden_cells(self, agents, tag_info) -> set[Tuple[int,int]]:
        forb = set()
        for a in agents:
            if a.start:
                forb.add(tuple(a.start))
            if a.goal:
                forb.add(tuple(a.goal))
        forb |= self.occupied_from_tags(tag_info)
        return forb

    def sample_free_goal(self, grid, forbidden: set[Tuple[int,int]]):
        H, W = len(grid), len(grid[0])
        candidates = [(r, c) for r in range(H) for c in range(W)
                      if grid[r][c] == 0 and (r, c) not in forbidden]
        return random.choice(candidates) if candidates else None

    def result(self, *, replan=False, ready=None, waiters=None, waiter_cells=None,
               align_center=None, align_direction=None, reason=None) -> ModeResult:
        r = ModeResult()
        if replan: r["replan"] = True
        if ready: r["ready"] = list(ready)
        if waiters: r["waiters"] = list(waiters)
        if waiter_cells: r["waiter_cells"] = list(waiter_cells)
        if align_center: r["align_center"] = set(align_center)
        if align_direction: r["align_direction"] = set(align_direction)
        if reason: r["reason"] = reason
        return r

    def ensure_agent_ctx(self, ctx: dict, rid: int) -> dict:
        s = ctx.setdefault(rid, {})
        s.setdefault("init_done", False)
        s.setdefault("idle_frames", 0)
        s.setdefault("last_pos", None)
        s.setdefault("verifying", False)
        s.setdefault("verify_goal", None)
        return s
