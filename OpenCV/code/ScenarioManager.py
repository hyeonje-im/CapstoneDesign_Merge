# ScenarioManager.py (minimal)
from __future__ import annotations
from typing import Protocol, TypedDict, Dict, List, Set, Tuple, Optional, Callable
import numpy as np
from cbs.pathfinder import PathFinder, Agent  # 기존 PathFinder를 Planner로 활용
import threading
import time, random

Cell = Tuple[int, int]
RobotId = int

# ---- 모드가 반환하는 표준 결과 ----
class ModeResult(TypedDict, total=False):
    replan: bool
    reason: str                   # "done" | "timer" | "vision" ...
    waiters: Set[RobotId]         # 이번 턴 멈춰야 할 로봇들
    ready: Set[RobotId]           # 이번 턴 CBS에 넣을 로봇들(부분 재계획)
    waiter_cells: Set[Cell]       # (선택) 대기 로봇의 셀 — 장애물로 올릴 때 사용
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
    
class ScenarioManager:
    """
    단일 책임:
    - 매 프레임 태그/맵을 읽어 현재 모드에 전달
    - 모드가 replan을 요청하면 CBS 실행 + 명령 전송
    - 컨트롤러 done 콜백 라우팅
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

        self.mode: IMode = mode
        self.enabled: bool = False
        self.ctx: Dict[int, dict] = {}  # per-agent 상태(모드 전용)

        # 컨트롤러 done 콜백 → 시나리오 매니저
        self.controller.set_sequence_completion_callback(self.on_sequence_complete)

        # 최초 진입 준비
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        self.mode.enter(tag_info=self.get_tag_info(), grid=self.get_grid(),agents=self.agents_ref, ctx=self.ctx, runstate=rs)

        self._active_step_plan: Dict[int, Dict[str, Dict[str, Cell]]] = {}
        self._active_step_count: int = 0

        self._initial_hd_hint: Dict[int, int] = {}
        self._replan_requested = False
        self._replanning = False

        

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
    def set_enabled(self, on: bool):
        self.enabled = bool(on)
        print(f"[Scenario] {'ENABLED' if self.enabled else 'PAUSED'}")

    def toggle_enabled(self):
        self.set_enabled(not self.enabled)

    def on_align_complete(self, rid: str):
        if not self.enabled: return
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        # 모드에게 "정렬 끝" 이벤트 전달 → 모드가 필요시 replan
        if hasattr(self.mode, "on_alignment_complete"):
            res = self.mode.on_alignment_complete(int(rid),
                        tag_info=self.get_tag_info(), grid=self.get_grid(),
                        agents=self.agents_ref, ctx=self.ctx, runstate=rs)
            if res and res.get("replan"):
                self._plan_and_send(self.get_grid(), res)

    # ---- 메인 루프에서 호출 ----
    def tick(self):
        if not self.enabled:
            return
        self._sync_starts_from_tags()
        grid = self.get_grid()
        tag = self.get_tag_info()
        runstate = self._build_runstate()
        res = self.mode.tick(tag_info=tag, grid=grid, agents=self.agents_ref,
                             ctx=self.ctx, runstate=runstate)
        if res:
            ac = res.get("align_center") or set()
            ad = res.get("align_direction") or set()
            targets = sorted(list(ac | ad))
            if targets:
                self.controller.run_align_sequence(targets, do_release=False)

            # 2) replan 처리
            if res.get("replan"):
                if getattr(self.controller, "active", False):
                    # 진행 중이면: 스텝 경계에서 1회만 재계획
                    self.controller.request_pause_on_step_boundary()
                    self._replan_requested = True
                    print("[Scenario] replan requested → deferred to step boundary")
                else:
                    # 유휴면: 즉시 재계획 & 전송
                    print("[Scenario] replan requested → run immediately (idle)")
                    self._sync_starts_from_tags()
                    self._plan_and_send(self.get_grid(), res)

    # ---- 컨트롤러 done 콜백 ----
    def on_sequence_complete(self, info=None):
        if not self.enabled:
            return
        # 시퀀스가 끝난 '경계'에서만 재계획을 1회 실행
        # 모드 훅 호출은 그대로 두되, 실행 여부는 플래그/결과를 합산해 판단
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        res = self.mode.on_sequence_complete(
            tag_info=self.get_tag_info(), grid=self.get_grid(),
            agents=self.agents_ref, ctx=self.ctx, runstate=rs
        ) or {}

        # 정렬 요청은 그대로 반영
        ac = res.get("align_center") or set()
        ad = res.get("align_direction") or set()
        targets = sorted(list(ac | ad))
        if targets:
            self.controller.run_align_sequence(targets, do_release=False)

        # 최종 replan 여부: (플래그 OR 모드 응답)
        should_replan = self._replan_requested or res.get("replan", False)
        if not should_replan:
            self._replan_requested = False
            return

        if self._replanning:
            return  # 혹시 중복 호출 가드
        self._replanning = True
        try:
            # 1) 태그 동기화
            self._sync_starts_from_tags()
            # 2) 태그 가려진 로봇은 last dst 폴백
            last_plan = getattr(self, "_active_step_plan", {}) or {}
            max_step = max(last_plan.keys()) if last_plan else -1
            def _last_dst_for(r):
                r_s = str(r)
                for s in range(max_step, -1, -1):
                    info = (last_plan.get(s, {}).get(r_s)) or (last_plan.get(s, {}).get(r))
                    if info and info.get("dst") is not None:
                        return tuple(info["dst"])
                return None
            tag = self.get_tag_info()
            for a in self.agents_ref:
                visible = ("grid_position" in (tag.get(a.id) or {})) and (tag.get(a.id, {}).get("status")=="On")
                if not visible:
                    ld = _last_dst_for(a.id)
                    if ld: a.start = ld

            # 3) (선택) 재시작 대상(ready)에게 초기 heading 힌트 주입
            #    - 필요 시 기존 로직을 함수화해서 호출해도 OK
            # 4) CBS 실행 + 전송
            grid = self.get_grid()
            self._plan_and_send(grid, res)
        finally:
            self._replan_requested = False
            self._replanning = False

    # ---- 태그 → Agent.start 동기화 (공통) ----
    def _sync_starts_from_tags(self):
        tag = self.get_tag_info()
        moving = bool(getattr(self.controller, "active", False))  # 컨트롤러가 실행 중인지

        for a in self.agents_ref:
            data = tag.get(a.id)
            if data and "grid_position" in data:
                a.start = tuple(data["grid_position"])

    # ---- CBS 실행 + 명령 전송 (단일 트리거 지점) ----
    def _plan_and_send(self, grid, res):
        # A) 모드 결과 정규화
        waiters_ids  = set(res.get("waiters", set()))
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


        # --- 2) 공용 런타임 뷰 생성 -----------------------------------------
    def _build_runstate(self) -> Dict[int, dict]:
        """모든 모드가 공통으로 쓰는 현재 실행 상태 뷰."""
        tag = self.get_tag_info()
        rs: Dict[int, dict] = {}
        # 컨트롤러가 per-robot 실행 여부를 제공하면 사용
        has_is_exec = hasattr(self.controller, "is_executing")
        for a in self.agents_ref:
            rid = a.id
            # executing: 컨트롤러가 있으면 그것을, 없으면 마지막 송신 기록을 사용
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
                "dir_aligned_recent": (hasattr(self.controller, "aligned_recently") and self.controller.aligned_recently(rid, within_sec=0.3)),
            }
        return rs

    def on_robot_complete(self, rid: str):
        if not self.enabled:
            return
        # 모드에게 물어봄: 이 로봇 완료 시점에 재계획/새 목적지 필요?
        self._sync_starts_from_tags()
        rs = self._build_runstate()
        if hasattr(self.mode, "on_robot_complete"):
            res = self.mode.on_robot_complete(
                int(rid),
                tag_info=self.get_tag_info(), grid=self.get_grid(),
                agents=self.agents_ref, ctx=self.ctx, runstate=rs
            )
        else:
            res = {"replan": False, "reason": "nohook"}

        # 재계획 필요 없다 → 컨트롤러는 평소대로 다음 스텝 진행
        if res and res.get("replan"):
            # 경계에서 정지만 예약, 실제 CBS는 on_sequence_complete에서 1회만 실행
            self.controller.request_pause_on_step_boundary()
            self._replan_requested = True
        return

    def _predict_logical_cell(self, rid: int) -> Optional[Cell]:
        """
        이동 중(active=True)일 때, CBS 전송에 사용했던 step_cell_plan을 바탕으로
        해당 로봇의 '현재 또는 직전 스텝의 dst'를 논리적 현재 셀로 추정.
        """
        plan = getattr(self, "_active_step_plan", None)
        if not plan or not getattr(self.controller, "active", False):
            return None

        # 컨트롤러가 가진 현재 스텝 번호 (0-indexed)
        cur = int(getattr(self.controller, "current_step", 0))

        # 1) 현재 스텝 dst 우선
        step = plan.get(cur, {})
        rid_s = str(rid)
        info = step.get(rid_s) or step.get(rid) or {}
        if "dst" in info and info["dst"] is not None:
            return tuple(info["dst"])

        # 2) 없으면 직전 스텝에서 뒤로 훑으며 마지막 dst 사용
        for s in range(cur - 1, -1, -1):
            info = (plan.get(s, {}).get(rid_s)) or (plan.get(s, {}).get(rid))
            if info and info.get("dst") is not None:
                return tuple(info["dst"])

        return None


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