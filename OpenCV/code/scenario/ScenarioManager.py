# ScenarioManager.py (FINAL WITH UI-LINK)
from __future__ import annotations
from typing import Protocol, TypedDict, Dict, List, Set, Tuple, Optional, Callable
import numpy as np
import time, random
import threading

from OpenCV.code.cbs.pathfinder import PathFinder, Agent
from OpenCV.code.ui_bridge import FrameBus   # ★ UI 연동 추가

Cell = Tuple[int, int]
RobotId = int


# -------------------------------------------------------------------
# ModeResult (모드 → 매니저로 전달되는 신호 구조)
# -------------------------------------------------------------------
class ModeResult(TypedDict, total=False):
    replan: bool
    reason: str
    waiters: Set[RobotId]
    ready: Set[RobotId]
    waiter_cells: Set[Cell]
    align_center: Set[RobotId]
    align_direction: Set[RobotId]
    ready_for_order: Dict[RobotId, bool]


# -------------------------------------------------------------------
# 모드 인터페이스
# -------------------------------------------------------------------
class IMode(Protocol):
    def enter(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
              ctx: Dict[int, dict], runstate: Dict[int, dict]): ...
    def exit(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]): ...
    def tick(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None: ...
    def on_sequence_complete(self, *, tag_info: dict, grid: np.ndarray, agents: List[Agent],
             ctx: Dict[int, dict], runstate: Dict[int, dict]) -> ModeResult | None: ...


# ===================================================================
#                        ScenarioManager
# ===================================================================
class ScenarioManager:
    """
    - 매 프레임 태그/맵을 읽어 현재 모드에 전달
    - 모드가 replan 요청 → CBS 실행 및 명령 전송
    - 정렬, 콜백, 번호키 처리
    - ★ UI 연동 (FrameBus) 상태 생성
    """

    # ---------------------------------------------------------------
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
        pathfinder_factory=None,
        mode: IMode,
    ):
        self.controller = controller
        self.agents_ref = agents_ref
        self.paths_ref = paths_ref
        self.get_grid = get_grid
        self.get_tag_info = get_tag_info
        self.path_to_commands = path_to_commands
        self.get_initial_hd = get_initial_hd
        self.pathfinder_factory = pathfinder_factory or (lambda grid: PathFinder(grid))

        self.mode: IMode = mode
        self.enabled: bool = False
        self.ctx: Dict[int, dict] = {}

        self._delayed_align_jobs = []
        self._initial_hd_hint: Dict[int, int] = {}
        self._replan_requested = False
        self._replanning = False

        self._active_step_plan = {}
        self._active_step_count = 0
        self._last_mode_result: dict = {}
        self._last_runstate: Dict[int, dict] = {}
        self.cbs_index = 0
        
        # Controller 콜백 등록
        controller.set_alignment_completion_callback(self.on_align_complete)
        controller.set_robot_completion_callback(self.on_robot_complete)
        controller.set_sequence_completion_callback(self.on_sequence_complete)

        # 최초 진입
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        self.mode.enter(
            tag_info=self.get_tag_info(), grid=self.get_grid(),
            agents=self.agents_ref, ctx=self.ctx, runstate=rs
        )
    def set_mode(self, mode: IMode):
        """모드 교체 (상태 초기화 간단화)"""
        # 1) 현재 runstate를 만든 뒤, 기존 모드에 exit(runstate=...) 전달
        rs = self._build_runstate()
        self.mode.exit(
            tag_info=self.get_tag_info(),
            grid=self.get_grid(),
            agents=self.agents_ref,
            ctx=self.ctx,
            runstate=rs,
        )
        # 2) 모드 교체 및 컨텍스트 초기화
        self.mode = mode
        self.ctx.clear()
        # 3) 최신 상태 동기화 후 새 모드 enter
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        self.mode.enter(
            tag_info=self.get_tag_info(),
            grid=self.get_grid(),
            agents=self.agents_ref,
            ctx=self.ctx,
            runstate=rs,
        )

    # ---------------------------------------------------------------
    def set_enabled(self, on: bool):
        self.enabled = bool(on)
        print(f"[Scenario] {'ENABLED' if on else 'PAUSED'}")

        if self.enabled:
            self._sync_starts_from_tags()
            rs = self._build_runstate()
            res = self.mode.enter(
                tag_info=self.get_tag_info(),
                grid=self.get_grid(),
                agents=self.agents_ref,
                ctx=self.ctx,
                runstate=rs,
            )
            # 정렬/재계획 처리
            if res:
                ac = set(res.get("align_center") or [])
                ad = set(res.get("align_direction") or [])
                targets = sorted(list(ac | ad))
                if targets:
                    self.ctx.setdefault("_align_gate", set()).update(targets)
                    self.controller.run_align_sequence(targets, do_release=False)

                if res.get("replan") and not self.ctx.get("_align_gate"):
                    if getattr(self.controller, "active", False):
                        self.controller.request_pause_on_step_boundary()
                        self._replan_requested = True
                    else:
                        self._sync_starts_from_tags()
                        self._plan_and_send(self.get_grid(), res)

    def toggle_enabled(self):
        self.set_enabled(not self.enabled)

    # ---------------------------------------------------------------
    def tick(self):
        if not self.enabled:
            return

        self._sync_starts_from_tags()

        # 딜레이 정렬 처리
        now = time.time()
        if self._delayed_align_jobs:
            due, later = [], []
            for (t, targets, reason) in self._delayed_align_jobs:
                (due if now >= t else later).append((t, targets, reason))
            self._delayed_align_jobs = later
            for _, targets, _ in due:
                targets = sorted(list(set(targets)))
                self.ctx.setdefault("_aligning", set()).update(targets)
                self.controller.run_align_sequence(targets, do_release=False)

        # 모드 tick
        grid = self.get_grid()
        tag = self.get_tag_info()
        rs = self._build_runstate()
        res = self.mode.tick(
            tag_info=tag, grid=grid, agents=self.agents_ref,
            ctx=self.ctx, runstate=rs
        )
        self._last_mode_result = res or {}

        # replan 처리
        if res and res.get("replan"):
            if getattr(self.controller, "active", False):
                self.controller.request_pause_on_step_boundary()
                self._replan_requested = True
            else:
                self._sync_starts_from_tags()
                self._plan_and_send(self.get_grid(), res)

        # ★ UI 업데이트 추가
        self._update_ui_state(rs)

    # ---------------------------------------------------------------
    # align 콜백
    def on_align_complete(self, rid: str):
        aligning = self.ctx.get("_aligning", set())
        aligning.discard(int(rid))

        res = None
        if hasattr(self.mode, "on_alignment_complete"):
            res = self.mode.on_alignment_complete(
                int(rid),
                tag_info=self.get_tag_info(),
                grid=self.get_grid(),
                agents=self.agents_ref,
                ctx=self.ctx,
                runstate=self._build_runstate(),
            )

        if res and res.get("replan"):
            if getattr(self.controller, "active", False):
                self.controller.request_pause_on_step_boundary()
                self._replan_requested = True
            else:
                self._plan_and_send(self.get_grid(), res or {})

    # ---------------------------------------------------------------
    # sequence 콜백
    def on_sequence_complete(self, info=None):
        if not self.enabled:
            return

        self._sync_starts_from_tags()
        rs = self._build_runstate()
        res = self.mode.on_sequence_complete(
            tag_info=self.get_tag_info(), grid=self.get_grid(),
            agents=self.agents_ref, ctx=self.ctx, runstate=rs
        ) or {}
        self._last_mode_result = res or {}

        should_replan = self._replan_requested or res.get("replan", False)
        if not should_replan:
            self._replan_requested = False
            return

        if self._replanning:
            return
        self._replanning = True

# 1. 지연 실행될 내부 함수 정의
        def _deferred_replan(grid_snapshot, res_snapshot):
            try:
                # 1) 태그 다시 동기화 (1초 뒤 최신 위치 반영)
                self._sync_starts_from_tags()
                
                # (옵션: 업로드 파일에 있던 'last dst fallback' 로직을 여기에 추가할 수도 있음)
                
                # 2) 계획 및 전송
                self._plan_and_send(grid_snapshot, res_snapshot)
                
                # 3) 지연 실행 후 상태 UI 업데이트 (필요 시)
                self._update_ui_state(self._build_runstate())
                
            finally:
                self._replanning = False
                self._replan_requested = False

        # 2. 1.0초 타이머 시작
        print(f"[Scenario] 경로 완료. 다음 계획까지 1.0초 대기...")
        grid_now = self.get_grid()  # 현재 그리드 캡처
        threading.Timer(1.0, _deferred_replan, args=[grid_now, res]).start()

        # ★ UI 업데이트 (대기 상태 즉시 반영)
        self._update_ui_state(rs)

    # ---------------------------------------------------------------
    def _sync_starts_from_tags(self):
        tag = self.get_tag_info()
        for a in self.agents_ref:
            info = tag.get(a.id)
            if info and "grid_position" in info:
                a.start = tuple(info["grid_position"])

    # ---------------------------------------------------------------
    # ---- CBS 실행 + 명령 전송 (단일 트리거 지점) ----
    def _plan_and_send(self, grid, res):
        # A) 모드 결과 정규화
        print("cbs_index",self.cbs_index)
        self.cbs_index += 1
        waiters_ids  = set(res.get("waiters", set()))
        waiters_ids |= set(self.ctx.get("_aligning", set()))
        ready_ids    = res.get("ready")  # 없으면 전체 start!=goal 대상
        waiter_cells = set(res.get("waiter_cells", set()))

        # B) 목표 누락/부적격 에이전트는 자동 waiter로 편입
        inferred_waiters = set()
        for a in self.agents_ref:
            bad = (not a.start) or (not a.goal) or (a.start == a.goal)
            if bad:
                inferred_waiters.add(a.id)
                if a.start: waiter_cells.add(a.start)

        waiters_ids |= inferred_waiters

        # C) 그리드 증강
        aug = grid.copy()
        for (r, c) in waiter_cells:
            if 0 <= r < aug.shape[0] and 0 <= c < aug.shape[1]:
                aug[r, c] = 1

        # D) CBS 대상(moving) 확정: ready가 있으면 그 집합만, 없으면 (start!=goal) 전체에서 waiter 제외
        def movable(a):
            if a.id in waiters_ids and (not a.goal or a.start == a.goal):
                return False
            if not (a.start and a.goal): return False
            if a.start == a.goal: return False
            if ready_ids is not None and a.id not in ready_ids: return False
            return True

        reasons = {}
        for a in self.agents_ref:
            r = []
            if a.id in waiters_ids: r.append("waiter")
            if not a.start:         r.append("no-start")
            if not a.goal:          r.append("no-goal")
            if a.start and a.goal and a.start == a.goal: r.append("start==goal")
            if ready_ids is not None and a.id not in ready_ids: r.append("not-in-ready")
            reasons[a.id] = {"start": a.start, "goal": a.goal, "flags": r}

        # moving 산출 후, 비었으면 상세 이유 출력
        moving = [a for a in self.agents_ref if (
            a.id not in waiters_ids and a.start and a.goal and a.start != a.goal and
            (ready_ids is None or a.id in ready_ids)
        )]
        if not moving:
            print("[Scenario] 움직일 로봇 없음 / reasons=", reasons)
            return

        # 3) CBS 실행
        try:
            pf = self.pathfinder_factory(aug)
            solved_agents = pf.compute_paths(moving)
        except Exception as e:
            print(f"[CBS] 예외 발생: {e}. 이번 턴을 스킵합니다.")
            return
        if not solved_agents:
            print("[CBS] 경로가 생성되지 않았습니다(None/empty).")
            return

        # 4) 시각화용 paths 갱신
        self.paths_ref.clear()
        for sa in solved_agents:
            p = sa.get_final_path()
            if p:
                self.paths_ref.append(p)

        # 5) 명령 생성 + 전송
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
                    step_cell_plan[i][rid] = {"src": tuple(path[i]), "dst": tuple(path[i+1])}

        if cmd_map:
            print("[Scenario] 계산된 경로를 로봇에게 전송:", {k: v for k, v in cmd_map.items() if v})
            self._active_step_plan = step_cell_plan or {}
            self._active_step_count = (max(step_cell_plan.keys()) + 1) if step_cell_plan else 0
            self.controller.start_sequence(cmd_map, step_cell_plan=step_cell_plan)
        else:
            print("[Scenario] 유효한 명령이 없습니다.")

    # ---------------------------------------------------------------
    # runstate 빌드
    def _build_runstate(self) -> Dict[int, dict]:
        tag = self.get_tag_info()
        rs = {}
        has_exec = hasattr(self.controller, "is_executing")

        for a in self.agents_ref:
            rid = a.id
            if has_exec:
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
            }
        self._last_runstate = rs
        return rs

    # ---------------------------------------------------------------
    def on_robot_complete(self, rid: str):
        if not self.enabled:
            return
        self._sync_starts_from_tags()
        rs = self._build_runstate()

        if hasattr(self.mode, "on_robot_complete"):
            res = self.mode.on_robot_complete(
                int(rid),
                tag_info=self.get_tag_info(), grid=self.get_grid(),
                agents=self.agents_ref, ctx=self.ctx, runstate=rs
            )
        else:
            res = None
        
        self._last_mode_result = res or {}
        
        if res and res.get("replan"):
            self.controller.request_pause_on_step_boundary()
            self._replan_requested = True

        # ★ UI 업데이트
        self._update_ui_state(rs)

    # ---------------------------------------------------------------
    # 📌 번호키
    def on_number_key(self, rid: int):
        """UI에서 숫자키가 눌렸을 때 호출된다.
        현재 모드가 on_number_key()를 지원하면 그걸 호출해 주문을 로봇에게 배정한다.
        """
        if not self.enabled:
            return

        # 1) 최신 위치 반영
        self._sync_starts_from_tags()
        rs = self._build_runstate()

        # 2) 현재 모드가 숫자키를 지원하는지 확인
        res = None
        if hasattr(self.mode, "on_number_key"):
            # RestaurantMode는 여기로 들어온다
            res = self.mode.on_number_key(
                rid,
                agents=self.agents_ref,
                ctx=self.ctx
            )
        elif hasattr(self.mode, "on_home_key"):
            # 예전 레거시 지원
            res = self.mode.on_home_key(
                rid,
                agents=self.agents_ref,
                ctx=self.ctx
            )

        # 3) 모드가 replan 요구하면 CBS 실행
        if res and res.get("replan"):
            if getattr(self.controller, "active", False):
                # 실행 중이면 일단 스텝 경계에서 멈추고 재계획 예약
                self.controller.request_pause_on_step_boundary()
                self._replan_requested = True
            else:
                # 즉시 재계획 가능
                self._sync_starts_from_tags()
                self._plan_and_send(self.get_grid(), res)

        # 4) UI 업데이트
        self._last_mode_result = res or {}
        self._update_ui_state(rs)


    def _update_ui_state(self, runstate: Dict[int, dict]):
        """FrameBus로 로봇 상태 + 시나리오 주문 상태를 모두 내려보내는 함수."""
        ui_dict = {}

        # ★★★ 최근 모드 결과 가져오기
        mode_result = getattr(self, "_last_mode_result", {})
        ready_map = mode_result.get("ready_for_order", {}) 
        for a in self.agents_ref:
            rid = a.id
            st = runstate.get(rid, {})

            # ★★★ 여기서 상태 계산 함수를 사용
            status = self.compute_robot_status(rid, mode_result, runstate)

            ui_dict[rid] = {
                "num": f"#{rid}",
                "pos": st.get("start"),
                "goal": st.get("goal"),
                "status": status,
                "ready_for_order": ready_map.get(rid, False),
            }

        # 로봇 상태 UI로 전송
        FrameBus.set_robot_ui_state(ui_dict)

        # RestaurantMode 주문 상태도 같이 전송
        order_state = self.get_mode_ui_state(drain_new=True)
        FrameBus.set_scenario_order_state(order_state)

    # ---------------------------------------------------------------
    def _compute_status_ui(self, rid: int, st: dict) -> str:
        """UI에 표시될 readable 로봇 상태 결정."""
        if st.get("executing"):
            return "MOVING"
        if st.get("goal") and st.get("start") == st.get("goal"):
            return "ARRIVED"
        if not st.get("goal"):
            return "IDLE"
        return "WAITING"
    def get_robot_position(self, rid: int):
        """로봇의 현재 시작 위치(start)를 UI용으로 반환."""
        rs = self._build_runstate()
        st = rs.get(rid)
        if not st:
            return None
        return st.get("start")

    def get_robot_goal(self, rid: int):
        """로봇의 현재 목표(goal)를 UI용으로 반환."""
        rs = self._build_runstate()
        st = rs.get(rid)
        if not st:
            return None
        return st.get("goal")

    def compute_robot_status(self, rid: int, mode_result: dict, runstate: Dict[int, dict]) -> str:
        """RestaurantMode에서 오는 mode_result + runstate 기준으로 상태 계산"""

        st = runstate.get(rid, {})


        # 1) ALIGNING 우선
        if mode_result:
            if rid in (mode_result.get("align_center", set()) or set()):
                return "ALIGNING"
            if rid in (mode_result.get("align_direction", set()) or set()):
                return "ALIGNING"

        # 2) WAITING
        if mode_result and rid in (mode_result.get("waiters", set()) or set()):
            return "WAITING"

        # 3) 도착 상태
        start = st.get("start")
        goal = st.get("goal")
        if start and goal and start == goal:
            return "ARRIVED"

        # 4) 실행 중
        if st.get("executing"):
            return "MOVING"

        # 5) 기본값
        return "IDLE"
    def is_ready_for_order(self, rid: int, mode_result: dict, runstate: Dict[int, dict]) -> bool:
        """
        UI용 '다음 명령 받을 준비 되었는지' 플래그.
        1순위: mode_result["ready_for_order"][rid]
        2순위: runstate[rid]["ready_for_order"] (있다면)
        """
        if mode_result:
            rfo = mode_result.get("ready_for_order")
            if isinstance(rfo, dict) and rid in rfo:
                return bool(rfo[rid])

        st = runstate.get(rid, {})
        return bool(st.get("ready_for_order", False))

    # ---------------------------------------------------------------
    def get_mode_ui_state(self, *, drain_new: bool = True):
        """RestaurantMode 전용 UI export"""
        try:
            from OpenCV.code.scenario.RestaurantMode import RestaurantMode
            if isinstance(self.mode, RestaurantMode):
                return self.mode.export_ui_state(clear_new=drain_new)
        except Exception as e:
            print(f"[Scenario] get_mode_ui_state ERROR: {e}")
        return {}
    
    def _handle_mode_result(self, ret, rs):
        if not ret:
            return

        # align_center / align_direction 처리
        if ret.get("align_center"):
            self.controller.run_center_align(ret["align_center"], do_release=False)

        if ret.get("align_direction"):
            self.controller.run_direction_align(ret["align_direction"], do_release=False)

        # CBS 재계획 요청
        if ret.get("replan", False):
            self._trigger_replan(rs)



# ===================================================================
# BaseMode 
# ===================================================================


class BaseMode:
    name = "Base"

    # === 표준 훅: 필요 없으면 아무 것도 안 함 ===
    def enter(self, *, tag_info, grid, agents, ctx, runstate): pass
    def exit(self, *, tag_info, grid, agents, ctx, runstate): pass
    def tick(self, *, tag_info, grid, agents, ctx, runstate) -> ModeResult|None: return None
    def on_sequence_complete(self, *, tag_info, grid, agents, ctx, runstate) -> ModeResult|None: return None
    def on_robot_complete(self, rid, *, tag_info, grid, agents, ctx, runstate) -> ModeResult|None: return None
    def on_alignment_complete(self, rid, *, tag_info, grid, agents, ctx, runstate) -> ModeResult|None: return None

    # === 공통 유틸 ===
    def get_agent_ctx(self, ctx: dict, rid: int) -> dict:
        ctx.setdefault("agents", {})
        return ctx["agents"].setdefault(str(rid), {})

    def set_agent_phase(self, ctx: dict, rid: int, phase: str):
        self.get_agent_ctx(ctx, rid)["phase"] = phase

    def is_idle(self, runstate: dict, rid: int, frames: int = 8) -> bool:
        st = runstate.get(rid) or runstate.get(str(rid))
        if not st: return False
        # idle_frames가 없으면 executing만으로 판단(필요 시 나중에 누적 프레임을 넣어도 됨)
        return (st.get("executing") is False) and (st.get("idle_frames", 999) >= frames)

    def occupied_from_tags(self, tag_info: dict) -> set[tuple[int,int]]:
        occ = set()
        for rid, dat in tag_info.items():
            gp = dat.get("grid_position")
            if gp and dat.get("status") == "On":
                occ.add((gp[0], gp[1]))
        return occ

    def collect_forbidden_cells(self, agents, tag_info) -> set[tuple[int,int]]:
        forb = set()
        for a in agents:
            if a.start: forb.add(tuple(a.start))
            if a.goal:  forb.add(tuple(a.goal))
        forb |= self.occupied_from_tags(tag_info)
        return forb

    def sample_free_goal(self, grid, forbidden: set[tuple[int,int]]):
        # grid: 0 통과 가능
        H, W = len(grid), len(grid[0])
        candidates = [(r,c) for r in range(H) for c in range(W) if grid[r][c] == 0 and (r,c) not in forbidden]
        return random.choice(candidates) if candidates else None

    def result(self, *, replan=False, ready=None, waiters=None, waiter_cells=None,
               align_center=None, align_direction=None, reason=None) -> ModeResult:
        r = ModeResult()
        if replan: r["replan"] = True
        if ready:  r["ready"] = list(ready)
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