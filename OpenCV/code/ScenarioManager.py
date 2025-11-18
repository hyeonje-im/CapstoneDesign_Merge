# ScenarioManager.py (with full FrameBus integration)
from __future__ import annotations
from typing import Protocol, TypedDict, Dict, List, Set, Tuple, Optional, Callable
import numpy as np
import threading
import time, random

from OpenCV.code.cbs.pathfinder import PathFinder, Agent
from OpenCV.code.ui_bridge import FrameBus   # ★ UI 연동 핵심

Cell = Tuple[int, int]
RobotId = int

# ---- 모드가 반환하는 표준 결과 ----
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
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray,
             agents: List[Agent], ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None: ...


class ScenarioManager:

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

        self.mode = mode
        self.enabled = False
        self.ctx: Dict[int, dict] = {}

        self.controller.set_alignment_completion_callback(self.on_align_complete)
        self.controller.set_robot_completion_callback(self.on_robot_complete)
        self.controller.set_sequence_completion_callback(self.on_sequence_complete)

        self._replanning = False
        self._replan_requested = False
        self._initial_hd_hint: Dict[int, int] = {}

        self._active_step_plan = {}
        self._active_step_count = 0

        # 최초 UI → start/home/goal 업데이트
        self._sync_starts_from_tags()

        rs = self._build_runstate()
        self.mode.enter(
            tag_info=self.get_tag_info(),
            grid=self.get_grid(),
            agents=self.agents_ref,
            ctx=self.ctx,
            runstate=rs
        )

    # -----------------------
    # enable / disable
    # -----------------------
    def set_enabled(self, on: bool):
        self.enabled = bool(on)
        print(f"[Scenario] {'ENABLED' if self.enabled else 'PAUSED'}")

    def toggle_enabled(self):
        self.set_enabled(not self.enabled)

    # -----------------------
    # ALIGN DONE 이벤트
    # -----------------------
    def on_align_complete(self, rid: str):
        if not self.enabled:
            return

        self._sync_starts_from_tags()
        rs = self._build_runstate()

        if hasattr(self.mode, "on_alignment_complete"):
            res = self.mode.on_alignment_complete(
                int(rid),
                tag_info=self.get_tag_info(),
                grid=self.get_grid(),
                agents=self.agents_ref,
                ctx=self.ctx,
                runstate=rs
            )
            if res and res.get("replan"):
                self._plan_and_send(self.get_grid(), res)

    # ================================================================
    # ========================== TICK =================================
    # ================================================================
    def tick(self):
        if not self.enabled:
            return

        # 태그 기반 위치 동기화 → UI에 위치 업데이트 전달
        self._sync_starts_from_tags()

        # ① GRID UI 업데이트
        grid = self.get_grid()
        FrameBus.set_grid_state(grid)

        # ② TAG = heading 정보 포함
        tag = self.get_tag_info()
        headings = {rid: info.get("heading_deg") for rid, info in tag.items()
                    if isinstance(info, dict) and ("heading_deg" in info)}
        FrameBus.set_headings(headings)

        # ③ (선택) 딜레이: controller에서 구현 시 자동 반영
        if hasattr(self.controller, "get_delays"):
            FrameBus.set_delays(self.controller.get_delays())

        # ④ 모드 tick
        runstate = self._build_runstate()
        res = self.mode.tick(tag_info=tag, grid=grid,
                             agents=self.agents_ref, ctx=self.ctx, runstate=runstate)

        if not res:
            return

        # ALIGN 요청 처리
        ac = res.get("align_center") or set()
        ad = res.get("align_direction") or set()
        targets = sorted(list(ac | ad))
        if targets:
            self.controller.run_align_sequence(targets, do_release=False)

        # REPLAN 처리
        if res.get("replan"):
            if getattr(self.controller, "active", False):
                self.controller.request_pause_on_step_boundary()
                self._replan_requested = True
                print("[Scenario] replan deferred to step boundary")
            else:
                print("[Scenario] replan immediately")
                self._sync_starts_from_tags()
                self._plan_and_send(self.get_grid(), res)

    # ================================================================
    # ================ UPDATE START / HOME / GOAL ====================
    # ================================================================
    def _sync_starts_from_tags(self):
        tag = self.get_tag_info()

        for a in self.agents_ref:
            dat = tag.get(a.id)
            if dat and "grid_position" in dat:
                a.start = tuple(dat["grid_position"])

        # UI 업데이트: agent_states / home / goal
        FrameBus.set_agent_states({a.id: a.start for a in self.agents_ref if a.start})
        FrameBus.set_home_positions({a.id: a.start for a in self.agents_ref if a.start})
        FrameBus.set_goal_positions({a.id: a.goal for a in self.agents_ref if a.goal})

    # ================================================================
    # ================= CBS 실행 + 명령 전송 ==========================
    # ================================================================
    def _plan_and_send(self, grid, res):
        # A) 모드 결과 정규화
        waiters_ids = set(res.get("waiters", set()))
        ready_ids = res.get("ready")
        waiter_cells = set(res.get("waiter_cells", set()))

        # B) start/goal 부적격 로봇 → waiter 처리
        inferred_waiters = set()
        for a in self.agents_ref:
            bad = (not a.start) or (not a.goal) or (a.start == a.goal)
            if bad:
                inferred_waiters.add(a.id)
                if a.start:
                    waiter_cells.add(a.start)

        waiters_ids |= inferred_waiters

        # C) waiter 장애물 증강
        aug = grid.copy()
        for (r, c) in waiter_cells:
            if 0 <= r < aug.shape[0] and 0 <= c < aug.shape[1]:
                aug[r, c] = 1

        # D) CBS 대상 산출
        moving = [
            a for a in self.agents_ref
            if (a.id not in waiters_ids
                and a.start and a.goal
                and a.start != a.goal
                and (ready_ids is None or a.id in ready_ids))
        ]
        if not moving:
            print("[Scenario] 움직일 로봇 없음")
            return

        # E) CBS 실행
        try:
            pf = self.pathfinder_factory(aug)
            solved_agents = pf.compute_paths(moving)
        except Exception as e:
            print(f"[CBS] 예외 발생: {e}")
            return
        if not solved_agents:
            print("[CBS] 경로 없음")
            return

        # F) 시각화용 paths_ref 업데이트
        self.paths_ref.clear()
        for sa in solved_agents:
            p = sa.get_final_path()
            if p:
                self.paths_ref.append(p)

        # G) 명령 생성 + step_cell_plan 구성
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

            if cmds:
                rid = str(sa.id)
                cmd_map[rid] = cmds
                for i in range(len(path) - 1):
                    step_cell_plan.setdefault(i, {})
                    step_cell_plan[i][rid] = {
                        "src": tuple(path[i]),
                        "dst": tuple(path[i+1])
                    }

        # H) 실제 로봇에게 명령 전송
        if cmd_map:
            print("[Scenario] 로봇들에게 경로 전송:", cmd_map)
            self._active_step_plan = step_cell_plan or {}
            self._active_step_count = (max(step_cell_plan.keys()) + 1) if step_cell_plan else 0

            self.controller.start_sequence(cmd_map, step_cell_plan=step_cell_plan)
        else:
            print("[Scenario] 유효 명령 없음")

        # ★ UI에 CBS 경로 전달
        FrameBus.set_paths({sa.id: sa.get_final_path() for sa in solved_agents})


    # ================================================================
    # ====================== 런타임 상태 생성 ========================
    # ================================================================
    def _build_runstate(self) -> Dict[int, dict]:
        tag = self.get_tag_info()
        rs: Dict[int, dict] = {}
        has_is_exec = hasattr(self.controller, "is_executing")

        for a in self.agents_ref:
            rid = a.id
            if has_is_exec:
                try:
                    executing = self.controller.is_executing(rid)
                except:
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
                )
            }
        return rs


    # ================================================================
    # ====================== 로봇 1개 완료 이벤트 =====================
    # ================================================================
    def on_robot_complete(self, rid: str):
        if not self.enabled:
            return

        self._sync_starts_from_tags()
        rs = self._build_runstate()

        if hasattr(self.mode, "on_robot_complete"):
            res = self.mode.on_robot_complete(
                int(rid),
                tag_info=self.get_tag_info(),
                grid=self.get_grid(),
                agents=self.agents_ref,
                ctx=self.ctx,
                runstate=rs
            )
        else:
            res = {"replan": False}

        if res.get("replan"):
            self.controller.request_pause_on_step_boundary()
            self._replan_requested = True


    # ================================================================
    # =================== 시퀀스 전체 완료 이벤트 =====================
    # ================================================================
    def on_sequence_complete(self, info=None):
        if not self.enabled:
            return

        self._sync_starts_from_tags()
        rs = self._build_runstate()

        res = self.mode.on_sequence_complete(
            tag_info=self.get_tag_info(),
            grid=self.get_grid(),
            agents=self.agents_ref,
            ctx=self.ctx,
            runstate=rs
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

        def _deferred(grid_snapshot, res_snapshot):
            try:
                self._sync_starts_from_tags()
                self._plan_and_send(grid_snapshot, res_snapshot)
            finally:
                self._replanning = False
                self._replan_requested = False

        threading.Timer(1.0, _deferred, args=[self.get_grid(), res]).start()


    # =================================================================
    # ===================== UI 모드 상태 전달 =========================
    # =================================================================
    def _push_ui_state(self, state: dict):
        if not hasattr(self, "_ui_state_queue"):
            self._ui_state_queue = []
        self._ui_state_queue.append(state)

    def get_mode_ui_state(self, drain_new: bool = True) -> dict:
        if not hasattr(self, "_ui_state_queue"):
            self._ui_state_queue = []

        if not self._ui_state_queue:
            return {}

        if drain_new:
            merged = {}
            for s in self._ui_state_queue:
                merged.update(s)
            self._ui_state_queue.clear()
            return merged
        else:
            return self._ui_state_queue[-1]


# =====================================================================
# ============================= BaseMode ===============================
# =====================================================================
class BaseMode:
    name = "Base"

    def enter(self, *, tag_info, grid, agents, ctx, runstate): pass
    def exit(self, *, tag_info, grid, agents, ctx, runstate): pass
    def tick(self, *, tag_info, grid, agents, ctx, runstate): return None
    def on_sequence_complete(self, *, tag_info, grid, agents, ctx, runstate): return None
    def on_robot_complete(self, rid, *, tag_info, grid, agents, ctx, runstate): return None
    def on_alignment_complete(self, rid, *, tag_info, grid, agents, ctx, runstate): return None

    # ------- 공통 유틸 -------
    def get_agent_ctx(self, ctx: dict, rid: int) -> dict:
        ctx.setdefault("agents", {})
        return ctx["agents"].setdefault(str(rid), {})

    def set_agent_phase(self, ctx: dict, rid: int, phase: str):
        self.get_agent_ctx(ctx, rid)["phase"] = phase

    def is_idle(self, runstate: dict, rid: int, frames: int = 8) -> bool:
        st = runstate.get(rid) or runstate.get(str(rid))
        if not st:
            return False
        return (st.get("executing") is False) and (st.get("idle_frames", 999) >= frames)

    def occupied_from_tags(self, tag_info: dict) -> set:
        occ = set()
        for rid, dat in tag_info.items():
            gp = dat.get("grid_position")
            if gp and dat.get("status") == "On":
                occ.add(tuple(gp))
        return occ

    def collect_forbidden_cells(self, agents, tag_info):
        forb = set()
        for a in agents:
            if a.start:
                forb.add(tuple(a.start))
            if a.goal:
                forb.add(tuple(a.goal))
        forb |= self.occupied_from_tags(tag_info)
        return forb

    def sample_free_goal(self, grid, forbidden):
        H, W = len(grid), len(grid[0])
        cand = [(r, c) for r in range(H) for c in range(W)
                if grid[r][c] == 0 and (r, c) not in forbidden]
        return random.choice(cand) if cand else None

    def result(self, *, replan=False, ready=None, waiters=None,
               waiter_cells=None, align_center=None, align_direction=None, reason=None):
        r = ModeResult()
        if replan: r["replan"] = True
        if ready: r["ready"] = list(ready)
        if waiters: r["waiters"] = list(waiters)
        if waiter_cells: r["waiter_cells"] = list(waiter_cells)
        if align_center: r["align_center"] = set(align_center)
        if align_direction: r["align_direction"] = set(align_direction)
        if reason: r["reason"] = reason
        return r
