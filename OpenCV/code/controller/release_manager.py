# release_manager.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Set
import math, time

Vec2 = Tuple[float, float]

@dataclass
class ReleasePolicy:
    arming_delay_s: float = 1.0          # 안정화 대기
    manage_window_s: float = 1.25        # 1대가 움직일 수 있는 관리창(하드락 무시 포함)
    re_spacing_s: float = 0.10           # RE → goal align 사이 딜레이
    re_batch_gap_s: float = 0.05         # 다음 토큰 주기 전 소폭 간격
    corridor_half_w_cm: Optional[float] = None  # None이면 guard.cfg.collision_radius_cm 사용
    goal_reach_eps_cm: float = 3.0       # 목표 도달 판정 여유

class ReleaseManager:
    """
    '사건(incident)' 단위로 래치된 로봇을 하나씩 출하하는 관리자.
    - 회랑 검사로 통과 가능 로봇을 고르고
    - 항상 한 대씩 RE (관리창 동안 STOP 억제; 하드락 포함)
    - 계속 재평가하여 가능한 로봇을 순차 출하
    """

    def __init__(self, guard, controller,client, policy: Optional[ReleasePolicy] = None):
        self.guard = guard                      # CollisionGuard 인스턴스
        self.controller = controller            # RobotController (goal 좌표 얻기용)
        self.client = client
        self.policy = policy or ReleasePolicy()
        # incident state
        self._active: bool = False
        self._cluster: List[int] = []
        self._armed_at: float = 0.0
        self._token_holder: Optional[int] = None
        self._token_started_at: float = 0.0
        self._last_progress_pos: Dict[int, Vec2] = {}  # 진행 감시
        self._last_seen_latched: set[int] = set()   # ← 최근 래치 집합 스냅샷

    # --------- 외부에서 호출 ---------
    def start_incident(self, cluster: Sequence[int], tag_info: Dict) -> None:
        """새 래치 클러스터가 생겼을 때 1회 호출."""
        self._active = True
        self._cluster = sorted(set(int(r) for r in cluster))
        self._armed_at = time.time() + self.policy.arming_delay_s
        self._token_holder = None
        self._token_started_at = 0.0
        self._last_progress_pos = {}
        # 초기 위치 저장
        for r in self._cluster:
            p = self._get_pos(tag_info, r)
            if p: self._last_progress_pos[r] = p
        print(f"[RM] incident started cluster={self._cluster} arm_until={self._armed_at:.2f}")

    def stop_incident(self) -> None:
        self._active = False
        self._cluster.clear()
        self._token_holder = None
        self._token_started_at = 0.0
        self._last_progress_pos.clear()
        print(f"[RM] incident cleared")

    def tick(self, tag_info: Dict, visible_ids: Iterable[int]) -> None:
        """매 프레임 호출(guard.tick 이후)."""
        vis_set = {int(r) for r in visible_ids}
        latched_now = {r for r in getattr(self.guard, "_latched", set()) if r in vis_set}

        # 래치 집합 변화 감지 → 사건 시작/재무장
        if not self._active and latched_now:
            # (옵션) 클러스터링으로 사건 묶기
            try:
                clusters = self.guard._clusters_under_threshold(tag_info, latched_now,
                                                            threshold_cm=self.guard.cfg.collision_radius_cm * 2.0)
            except TypeError:
                # _clusters_under_threshold 시그니처가 (tag_info, ids, threshold) 인 경우
                clusters = self.guard._clusters_under_threshold(tag_info, list(latched_now),
                                                            self.guard.cfg.collision_radius_cm * 2.0)

            seed = clusters[0] if clusters else sorted(latched_now)
            self.start_incident(seed, tag_info)

        # 래치 스냅샷 업데이트(옵션)
        self._last_seen_latched = latched_now
        
        
        if not self._active:
            return
        now = time.time()
        # 0) 클러스터가 비거나 모두 래치 해제되면 종료
        latched = [r for r in self._cluster if r in self.guard._latched]
        if not latched:
            self.stop_incident(); return
        self._cluster = latched

        # 1) 안정화 창
        if now < self._armed_at:
            return

        # 2) 토큰 보유자가 달리는 중인가?
        if self._token_holder is not None:
            self._manage_window(tag_info, now)
            return

        # 3) 회랑 검사로 후보 선정
        passers, blockers = self._corridor_selection(tag_info, self._cluster)
        if not passers:
            # 아직 길이 안 열렸으면 일단 유지 (미래 2단계: 우회 전략)
            return

        # 4) 우선순위(기본: id 오름차순)로 1대만 출하
        rid = min(passers)
        self._issue_token(rid, tag_info, now)

        # 5) 다음 토큰 전 아주 짧은 간격 유지
        # (실제로는 다음 tick에서 manage_window가 동작)
        return

    # --------- 내부 구현 ---------
    def _manage_window(self, tag_info: Dict, now: float) -> None:
        rid = self._token_holder
        if rid is None:
            return
        # 관리창 동안 STOP 억제(하드락 포함)
        self.guard.suppress_stop_for(rid, 0.20)  # 매 프레임 갱신: 끊기지 않게

        # 종료 조건: (a) 시간 만료, (b) 목표 근접, (c) 클러스터 이탈/랭킹 변화(옵션)
        if now - self._token_started_at >= self.policy.manage_window_s:
            print(f"[RM] token timeout rid={rid}")
            self._token_holder = None
            return

        # 목표 도달(스텝 목표 근접) 체크
        goal = self._get_goal_pos(rid)
        pos = self._get_pos(tag_info, rid)
        if goal and pos:
            if self._dist(goal, pos) <= self.policy.goal_reach_eps_cm:
                print(f"[RM] token done rid={rid}")
                self._token_holder = None
                return

        # 진행 감시(정체 시 추후 전략 교체 가능)
        if pos:
            self._last_progress_pos[rid] = pos

    def _corridor_selection(self, tag_info: Dict, ids: Sequence[int]) -> Tuple[List[int], List[int]]:
        """회랑 검사: 내 위치→목표까지 'collision_radius_cm' 원이 통과 가능한지."""
        cfg = self.guard.cfg
        half_w = self.policy.corridor_half_w_cm or cfg.collision_radius_cm
        passers, blockers = [], []
        # 다른 로봇 & 장애물 데이터 뽑기
        robots = [(r, self._get_pos(tag_info, r)) for r in ids]
        robots = [(r,p) for r,p in robots if p is not None]
        obstacles = self.guard._extract_obstacles(tag_info)  # [(ox,oy,r), ...]

        for r, pr in robots:
            goal = self._get_goal_pos(r)
            if not goal:
                blockers.append(r); continue
            if self._corridor_clear(pr, goal, half_w, robots, obstacles, r):
                passers.append(r)
            else:
                blockers.append(r)
        return passers, blockers

    def _issue_token(self, rid: int, tag_info: Dict, now: float) -> None:
        self._token_holder = rid
        self._token_started_at = now
        # 억제 먼저 건다(하드락 포함)
        self.guard.suppress_stop_for(rid, self.policy.manage_window_s + 0.2)

        # 1) RE 직접 송신 (수동 개입과 동일 채널 사용)
        self.client.publish(f"robot/{rid}/cmd", "RE")
        print(f"[RM] RE → Robot_{rid}")

        # 2) 짧은 딜레이 후 스텝 목표 정렬
        time.sleep(self.policy.re_spacing_s)
        try:
            self.controller.go_to_step_goal([str(rid)])
            self.guard._latched.discard(rid)
        except Exception as e:
            print(f"[RM] go_to_step_goal failed rid={rid}: {e}")

    # ---------- 기하/헬퍼 ----------
    def _corridor_clear(self, p: Vec2, g: Vec2, half_w: float,
                        robots: List[Tuple[int, Vec2]],
                        obstacles: List[Tuple[float,float,float]],
                        self_id: int) -> bool:
        """선분 p→g 의 '캡슐(half_w)'이 다른 로봇/장애물과 겹치지 않는지."""
        # 로봇 충돌: 반폭 합 = half_w + other_half (other_half ~= guard.cfg.collision_radius_cm)
        for rid, q in robots:
            if rid == self_id: 
                continue
            if q is None: 
                continue
            if not self._capsules_disjoint(p, g, q, q, half_w, self.guard.cfg.collision_radius_cm):
                return False
        # 장애물: 반폭 합 = half_w + r_obs
        for (ox, oy, r_obs) in obstacles:
            if not self._capsule_circle_disjoint(p, g, (ox, oy), half_w, r_obs):
                return False
        return True

    @staticmethod
    def _dist(a: Vec2, b: Vec2) -> float:
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def _get_pos(self, tag_info: Dict, rid: int) -> Optional[Vec2]:
        return self.guard._get_pos(tag_info, rid)

    def _get_goal_pos(self, rid: int) -> Optional[Vec2]:
        # RobotController가 스텝 목표 좌표를 제공한다고 가정 (없으면 None)
        try:
            return tuple(self.controller.get_step_goal_cm(rid))
        except Exception:
            return None

    # --- 캡슐-기하 ---
    def _capsules_disjoint(self, a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2,
                           half_w_a: float, half_w_b: float) -> bool:
        # 두 선분의 최소거리 > 합 반폭 ?
        def dot(x, y): return x[0]*y[0]+x[1]*y[1]
        def sub(x, y): return (x[0]-y[0], x[1]-y[1])
        def add(x, y): return (x[0]+y[0], x[1]+y[1])
        def mul(x, s): return (x[0]*s, x[1]*s)
        def seg_min_dist(A,B,C,D):
            u=sub(B,A); v=sub(D,C); w=sub(A,C)
            a=dot(u,u); b=dot(u,v); c=dot(v,v); d=dot(u,w); e=dot(v,w)
            Dn=a*c-b*b; SMALL=1e-8
            if Dn<SMALL:
                sN=0.0; sD=1.0; tN=e; tD=c if c>SMALL else 1.0
            else:
                sN=(b*e-c*d); tN=(a*e-b*d)
                if sN<0.0: sN=0.0; tN=e; tD=c
                elif sN>a: sN=a; tN=e+b; tD=c
            if tN<0.0:
                tN=0.0; sN=max(0.0, min(a, -d)); sD=a if a>SMALL else 1.0
            elif tN>tD:
                tN=tD; sN=max(0.0, min(a, -d+b)); sD=a if a>SMALL else 1.0
            sc=0.0 if sD==0 else sN/sD
            tc=0.0 if tD==0 else tN/tD
            P=add(A, mul(u, sc)); Q=add(C, mul(v, tc))
            dx=P[0]-Q[0]; dy=P[1]-Q[1]
            return math.hypot(dx, dy)
        mind = seg_min_dist(a0, a1, b0, b1)
        return mind > (half_w_a + half_w_b)

    @staticmethod
    def _capsule_circle_disjoint(a0: Vec2, a1: Vec2, center: Vec2, half_w: float, r: float) -> bool:
        # 선분-점 최소거리 > 합 반지름 ?
        def clamp(x, lo, hi): return lo if x<lo else hi if x>hi else x
        ax, ay = a0; bx, by = a1; cx, cy = center
        abx, aby = (bx-ax, by-ay)
        ab2 = abx*abx + aby*aby
        if ab2 <= 1e-9:
            # 점-원
            return math.hypot(ax-cx, ay-cy) > (half_w + r)
        t = ((cx-ax)*abx + (cy-ay)*aby) / ab2
        t = clamp(t, 0.0, 1.0)
        px, py = (ax + abx*t, ay + aby*t)
        return math.hypot(px-cx, py-cy) > (half_w + r)
