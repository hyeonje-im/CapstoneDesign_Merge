from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple
import math
import time
from config import corridor_width

from enum import Enum

class StopReason(Enum):
    HARDLOCK = "hardlock(Rc')"                             # 로봇-로봇 현재시점 하드락
    BREACH_WITHIN_STEP = "breach_within_step"              # 로봇-로봇 스텝 내 CCD 진입
    OBSTACLE_HARDLOCK = "hardlock_obstacle(Rc')"           # 로봇-장애물 현재시점 하드락
    OBSTACLE_WITHIN_STEP = "breach_within_step_obstacle"   # 로봇-장애물 스텝 내 CCD 진입

Vec2 = Tuple[float, float]


@dataclass
class GuardConfig:
    # 비상 정지용
    step_cm: float = 15.0              # one MOVE step distance
    collision_radius_cm: float = 6.0  # target stop-before-collision distance (R_c)
    eps_step_cm: float = 1.0           # tolerance on step window 
    vmin_cmps: float = 3.0             # ignore very small speeds (noise floor)
    max_pairs: int = 1000
    tau_latency_s: float = 0.0        # comms + firmware reaction latency
    
    arm_dist_cm: float = 15.0          # 이 거리 밖이면 CCD 자체를 스킵(장애물/로봇 공통)
    vr_min_cmps: float = 5.0           # 방사 접근속도 하한(이보다 작으면 CCD 스킵)

    # --- 장애물(정지 원판) ---
    obstacle_radius_cm: float = 5        # 10x10 장애물
    max_obstacles: int = 200               # 프레임당 검사할 최대 장애물 수(성능 가드)

    

class CollisionGuard:
    """
    Frame-synchronous emergency stop guard (no decel; only im_S).

    - Destination/path agnostic; relies on relative kinematics (CPA) only.
    - Triggers immediate stop (im_S) when a pair is predicted to breach R_c'
    within the current MOVE step window.

    Usage:
        guard = CollisionGuard(
            stop_fn=lambda ids: immediate_stop(client, ids),
            config=GuardConfig(...)
        )

        guard.tick(tag_info, PRESET_IDS)
    """


    def __init__(self, stop_fn: Callable[[Sequence[int]], None], *, config: Optional[GuardConfig] = None) -> None:
        self.stop_fn = stop_fn
        self.cfg = config or GuardConfig()
        self._latched: Set[int] = set()
        # corridor dwell timers (seconds) per robot
        self._last_moved_at: Dict[int, float] = {}
        self._suppress_until: Dict[int, float] = {}
        self._goal_provider: Optional[Callable[[int], Optional[Vec2]]] = None

    # ------------------------ public API ------------------------
    def set_goal_provider(self, fn: Callable[[int], Optional[Vec2]]) -> None:
        """rid -> (gx, gy) cm or None"""
        self._goal_provider = fn

    def tick(self, tag_info: Dict, ids: Iterable[int]) -> None:
        """Call once per frame after tag_info is updated.
        - Evaluates new hazards -> issues im_S (and latches IDs)
        """

        ids = [int(r) for r in ids]
        # 1) Immediate stop checks (new hazards only).
        new_targets = self._detect_hard_stop_targets(tag_info, ids)
        new_targets = [r for r in new_targets if r not in self._latched]
        
        # tag_info 업데이트 직후, visible id들에 대해
        now = time.time()
        expired = [r for r,t in self._suppress_until.items() if t <= now]
        for r in expired: self._suppress_until.pop(r, None)
        
        for rid in ids:
            v = self._get_vel(tag_info, rid) or (0.0, 0.0)
            if self._norm2(v) >= self.cfg.vmin_cmps:
                self._last_moved_at[rid] = now
            else:
                # 초기값이 없을 수 있으니 기본값을 과거로 깔아준다
                self._last_moved_at.setdefault(rid, now)

        if new_targets:
            self.stop_fn(sorted(new_targets))
            self._latched.update(new_targets)

    # --------------------- core detection logic ---------------------
    def _detect_hard_stop_targets(self, tag_info: Dict, ids: Sequence[int]) -> List[int]:
        cfg = self.cfg
        ids = [r for r in ids if self._get_pos(tag_info, r) is not None]
        n = len(ids)
        if n == 0:
            return []
        # O(N^2) 가드
        if n * (n - 1) // 2 > cfg.max_pairs:
            ids = ids[: int((math.sqrt(1 + 8 * cfg.max_pairs) - 1) // 2)]
            n = len(ids)

        to_stop: Set[int] = set()
        pair_reasons: Dict[tuple[int,int], str] = {}

        # --- 로봇-로봇 ---
        for i in range(n):
            for j in range(i + 1, n):
                a, b = ids[i], ids[j]
                pa, va = self._get_pos(tag_info, a), self._get_vel(tag_info, a) or (0.0, 0.0)
                pb, vb = self._get_pos(tag_info, b), self._get_vel(tag_info, b) or (0.0, 0.0)

                if self._norm2(va) < cfg.vmin_cmps and self._norm2(vb) < cfg.vmin_cmps:
                    continue

                # 목적지 유무 확인
                has_goal_a = (self._goal_provider is not None and self._goal_provider(a) is not None)
                has_goal_b = (self._goal_provider is not None and self._goal_provider(b) is not None)

                if not (has_goal_a and has_goal_b):
                    # CCD 생략: 하드락만 검사
                    # Rc' = Rc + ||u||*tau
                    rx, ry = (pb[0] - pa[0], pb[1] - pa[1])
                    vx, vy = (vb[0] - va[0], vb[1] - va[1])
                    dist_now = math.hypot(rx, ry)
                    vr = - ((rx * vx + ry * vy) / dist_now) if dist_now > 1e-6 else 0.0  # (+)면 접근
                    R_hl = self.cfg.collision_radius_cm
                    if dist_now <= R_hl and vr >= self.cfg.vmin_cmps:
                        if not self._is_suppressed(a): to_stop.add(a)
                        if not self._is_suppressed(b): to_stop.add(b)
                        pair_reasons[(min(a,b), max(a,b))] = StopReason.HARDLOCK.value
                    continue  # 이 쌍은 CCD 패스

                ok, reason = self._pair_breaches_within_step(pa, va, pb, vb)
                pair_reasons[(min(a,b), max(a,b))] = reason
                if ok:
                    if self._is_suppressed(a) or self._is_suppressed(b):
                        continue
                    to_stop.add(a); to_stop.add(b)

        if to_stop:
            pairs = [
                f"{i}-{j}:{pair_reasons[(i,j)]}"
                for (i,j) in sorted(pair_reasons.keys())
                if i in to_stop or j in to_stop
            ]
            if pairs:
                print(f"[CG][STOP][pairs] {', '.join(pairs)}")

        # --- 로봇-장애물 ---
        obstacles = self._extract_obstacles(tag_info)  # [(ox, oy, r), ...]
        if obstacles:
            for rid in ids:
                pa = self._get_pos(tag_info, rid)
                va = self._get_vel(tag_info, rid) or (0.0, 0.0)
                if pa is None or self._norm2(va) < cfg.vmin_cmps:
                    continue
                hit = False
                for k, oc in enumerate(obstacles):
                    has_goal = (self._goal_provider is not None and self._goal_provider(rid) is not None)
                    if not has_goal:
                        # CCD 생략: 하드락만 검사
                        ox, oy, r_obs = oc
                        rx, ry = (ox - pa[0], oy - pa[1])
                        dist_now = math.hypot(rx, ry)
                        vr = ((rx * va[0] + ry * va[1]) / dist_now) if dist_now > 1e-6 else 0.0  # (+)면 장애물로 접근
                        R_hl = self.cfg.collision_radius_cm + r_obs
                        if dist_now <= R_hl and vr >= self.cfg.vmin_cmps:
                            if not self._is_suppressed(rid):
                                to_stop.add(rid)
                                print(f"[CG][STOP][obs] r={rid} vs obs: {StopReason.OBSTACLE_HARDLOCK.value}")
                            hit = True
                            break
                        else:
                            continue  # 이 장애물(및 나머지) 대해 CCD 패스

                    ok, reason = self._agent_hits_obstacle_within_step(pa, va, oc)
                    if ok:
                        if self._is_suppressed(rid):
                            continue  # 관리창 억제: 하드락 포함 STOP 무시
                        to_stop.add(rid)
                        print(f"[CG][STOP][obs] r={rid} vs obs#{k}: {reason}")
                        hit = True
                        break

                if hit:
                    continue

        return sorted(to_stop)


    # 기존: bool 리턴 → (bool, str) 리턴으로 변경
    def _pair_breaches_within_step(self, pa: Vec2, va: Vec2, pb: Vec2, vb: Vec2) -> tuple[bool, str]:
        """
        단일 CCD(연속충돌) 로직:
        - 하드락: dist_now <= Rc' (Rc' = Rc + ||u||*tau) → 즉시 정지
        - 아니면 시간창 H = min( (step+eps)/|va|, (step+eps)/|vb| ) 내에서
            ∃ t∈[0,H] s.t. ||s0 + u t|| <= R (R = Rc + ||u||*tau) 이면 정지
        """
        cfg = self.cfg

        # 상대 위치/속도
        rx, ry = (pb[0] - pa[0], pb[1] - pa[1])
        vx, vy = (vb[0] - va[0], vb[1] - va[1])
        u2 = vx * vx + vy * vy
        u = math.sqrt(u2)

        # ---- 하드락 ----
        Rc_prime_now = cfg.collision_radius_cm + u * cfg.tau_latency_s
        dist_now = math.hypot(rx, ry)
        if dist_now > 1e-6:
            # 방사 접근속도: (+)면 접근
            vr = - (rx * vx + ry * vy) / dist_now
        else:
            vr = 0.0

        # 하드락 반경은 지연항 없이 '기본 Rc'만 사용 (접선 통과 억제)
        R_hl = self.cfg.collision_radius_cm

        if dist_now <= R_hl and vr >= self.cfg.vmin_cmps:
            return True, StopReason.HARDLOCK.value

        # ---- 시간창 H ----
        speed_a = max(self._norm2(va), cfg.vmin_cmps)
        speed_b = max(self._norm2(vb), cfg.vmin_cmps)
        H_a = (cfg.step_cm + cfg.eps_step_cm) / speed_a
        H_b = (cfg.step_cm + cfg.eps_step_cm) / speed_b
        H = min(H_a, H_b)

        if dist_now > 1e-6:
            vr = - (rx*vx + ry*vy) / dist_now   # (+)면 접근
            if vr < cfg.vr_min_cmps:
                return False, StopReason.BREACH_WITHIN_STEP.value

        # ---- CCD: [0,H] 내 교차 여부 ----
        R = cfg.collision_radius_cm + u * cfg.tau_latency_s
        R2 = R * R

        # 상대속도 거의 0 → 거리 상수
        if u2 <= 1e-9:
            if dist_now <= R:
                info = f"{StopReason.BREACH_WITHIN_STEP.value}(H={H:.3f},R={R:.2f},dmin={dist_now:.2f},t_enter=0.000)"
                return True, info
            else:
                return False, StopReason.BREACH_WITHIN_STEP.value

        # 일반 케이스: 2차식 교차
        # D(t) = ||s0 + u t||^2 <= R^2  ⇔  A t^2 + B t + C <= 0
        A = u2
        B = 2.0 * (rx * vx + ry * vy)
        C = (rx * rx + ry * ry) - R2
        disc = B * B - 4.0 * A * C

        if disc < 0.0:
            return False, StopReason.BREACH_WITHIN_STEP.value

        sqrt_disc = math.sqrt(disc) if disc > 0.0 else 0.0
        t_enter = (-B - sqrt_disc) / (2.0 * A)
        t_exit  = (-B + sqrt_disc) / (2.0 * A)

        if t_enter <= H and t_exit >= 0.0:
            t_star = max(0.0, min(H, - (rx * vx + ry * vy) / A))
            dmin = math.hypot(rx + vx * t_star, ry + vy * t_star)
            info = f"{StopReason.BREACH_WITHIN_STEP.value}(H={H:.3f},R={R:.2f},dmin={dmin:.2f},t_enter={max(0.0,t_enter):.3f})"
            return True, info

        return False, StopReason.BREACH_WITHIN_STEP.value


    # --------------------- helpers ---------------------
    def _clusters_under_threshold(self, tag_info: Dict, ids: Iterable[int], threshold_cm: float) -> List[List[int]]:
        ids = [int(r) for r in ids if self._get_pos(tag_info, r) is not None]
        if len(ids) < 2:
            return []
        adj = {r: set() for r in ids}
        for i, a in enumerate(ids):
            pa = self._get_pos(tag_info, a)
            for b in ids[i + 1:]:
                pb = self._get_pos(tag_info, b)
                if pa is None or pb is None:
                    continue
                if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) <= threshold_cm:
                    adj[a].add(b); adj[b].add(a)
        clusters: List[List[int]] = []
        seen: Set[int] = set()
        for r in ids:
            if r in seen:
                continue
            stack = [r]
            comp: List[int] = []
            while stack:
                u = stack.pop()
                if u in seen:
                    continue
                seen.add(u)
                comp.append(u)
                stack.extend(v for v in adj[u] if v not in seen)
            if len(comp) >= 2:
                clusters.append(sorted(comp))
        return clusters

    def _get_pos(self, tag_info: Dict, rid: int) -> Optional[Vec2]:
        d = tag_info.get(int(rid), {})
        if d.get("status") != "On":
            return None
        # Try common position keys in cm
        if "corrected_center" in d:
            return tuple(d["corrected_center"])  # (x_cm, y_cm)
        if "smoothed_coordinates_cm" in d:
            return tuple(d["smoothed_coordinates_cm"])  # fallback
        if "center_cm" in d:
            return tuple(d["center_cm"])  # generic
        return None

    def _get_vel(self, tag_info: Dict, rid: int) -> Optional[Vec2]:
        d = tag_info.get(int(rid), {})
        v = d.get("velocity_cmps")
        if v is not None:
            return tuple(v)
        # alternative field names (robustness)
        v = d.get("velocity") or d.get("motion_velocity_cmps")
        return tuple(v) if v is not None else None
    
    def _extract_obstacles(self, tag_info) -> list[tuple[float, float, float]]:
        """
        반환: [(ox, oy, r), ...]
        우선순위:
        - tag_info['obstacle_circles_cm']  # Vision이 제공: [(cx, cy, r), ...]
        """
        obs: list[tuple[float, float, float]] = []
        if 'obstacle_circles_cm' in tag_info:
            raw = tag_info['obstacle_circles_cm']
            for o in raw:
                if isinstance(o, dict):
                    ox, oy, r = float(o['x']), float(o['y']), float(o.get('r', self.cfg.obstacle_radius_cm))
                else:
                    # (x, y, r) 또는 (x, y) 튜플
                    if len(o) == 3:
                        ox, oy, r = float(o[0]), float(o[1]), float(o[2])
                    else:
                        ox, oy, r = float(o[0]), float(o[1]), float(self.cfg.obstacle_radius_cm)
                obs.append((ox, oy, r))
        return obs[: self.cfg.max_obstacles]

    
    def _agent_hits_obstacle_within_step(self, pa: Vec2, va: Vec2, oc: tuple[float,float,float]) -> tuple[bool, str]:
        """
        로봇-장애물(정지 원판) CCD 판정.
        - 장애물 속도 0
        - 유효 반경 R = (Rc + r_obs) + ||va||*tau
        - 시간창 H = (step+eps) / max(|va|, vmin)
        """
        cfg = self.cfg
        ox, oy, r_obs = float(oc[0]), float(oc[1]), float(oc[2])

        # 상대 위치/속도 (장애물은 정지)
        rx, ry = (ox - pa[0], oy - pa[1])
        vx, vy = (-va[0], -va[1])     # u = -va
        u2 = vx * vx + vy * vy
        u = math.sqrt(u2)

        # 하드락
        R_sum = cfg.collision_radius_cm + r_obs
        Rc_prime_now = R_sum + self._norm2(va) * cfg.tau_latency_s
            # ---- 하드락 (즉시 충돌만) ----
        dist_now = math.hypot(rx, ry)
        if dist_now > 1e-6:
            # 장애물은 정지 → 방사 접근속도: (+)면 장애물 쪽으로 전진
            vr = (rx * va[0] + ry * va[1]) / dist_now
        else:
            vr = 0.0

        # 하드락 반경은 지연항 없이 'Rc + r_obs'만 사용
        R_hl = self.cfg.collision_radius_cm + r_obs

        if dist_now <= R_hl and vr >= self.cfg.vmin_cmps:
            return True, StopReason.OBSTACLE_HARDLOCK.value


        # [완화 게이트 #1] Arm 거리: 멀면 CCD 스킵
        if dist_now > self.cfg.arm_dist_cm:
            return False, StopReason.OBSTACLE_WITHIN_STEP.value

        # [완화 게이트 #2] 방사 접근속도 하한
        if dist_now > 1e-6:
            # 장애물은 정지 → 상대속도 u = -va
            vr = - (rx * (-va[0]) + ry * (-va[1])) / dist_now  # (+)면 접근
            if vr < self.cfg.vr_min_cmps:
                return False, StopReason.OBSTACLE_WITHIN_STEP.value


        # 시간창
        speed = max(self._norm2(va), cfg.vmin_cmps)
        H = (cfg.step_cm + cfg.eps_step_cm) / speed

        # CCD
        R = R_sum + self._norm2(va) * cfg.tau_latency_s
        R2 = R * R

        if u2 <= 1e-9:
            if dist_now <= R:
                info = f"{StopReason.OBSTACLE_WITHIN_STEP.value}(H={H:.3f},R={R:.2f},dmin={dist_now:.2f},t_enter=0.000)"
                return True, info
            else:
                return False, StopReason.OBSTACLE_WITHIN_STEP.value

        A = u2
        B = 2.0 * (rx * vx + ry * vy)
        C = (rx * rx + ry * ry) - R2
        disc = B * B - 4.0 * A * C
        if disc < 0.0:
            return False, StopReason.OBSTACLE_WITHIN_STEP.value

        sqrt_disc = math.sqrt(disc) if disc > 0.0 else 0.0
        t_enter = (-B - sqrt_disc) / (2.0 * A)
        t_exit  = (-B + sqrt_disc) / (2.0 * A)

        if t_enter <= H and t_exit >= 0.0:
            t_star = max(0.0, min(H, - (rx * vx + ry * vy) / A))
            dmin = math.hypot(rx + vx * t_star, ry + vy * t_star)
            info = f"{StopReason.OBSTACLE_WITHIN_STEP.value}(H={H:.3f},R={R:.2f},dmin={dmin:.2f},t_enter={max(0.0,t_enter):.3f})"
            return True, info

        return False, StopReason.OBSTACLE_WITHIN_STEP.value

    def suppress_stop_for(self, rid: int, duration_s: float) -> None:
        """rid 에 대해 duration_s 동안 STOP(하드락 포함)을 무시한다."""
        self._suppress_until[rid] = max(self._suppress_until.get(rid, 0.0), time.time() + duration_s)

    def _is_suppressed(self, rid: int) -> bool:
        return time.time() < self._suppress_until.get(rid, 0.0)

    @staticmethod
    def _norm2(v: Vec2) -> float:
        return math.hypot(v[0], v[1])

    @staticmethod
    def _has_forward_velocity(v: Vec2) -> bool:
        return math.hypot(v[0], v[1]) > 1e-6


