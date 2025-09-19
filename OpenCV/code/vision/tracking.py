# tracking.py
import collections, math, time
from collections import deque

class MovingWindowTracker:
    def __init__(self, window_sec=0.25, max_speed_cmps=200.0, zero_snap_thr=2.0, ema_tau=0.15):
        self.window_sec = window_sec
        self.points = deque()  # (t, x_cm, y_cm)
        self.max_speed_cmps = max_speed_cmps
        self.zero_snap_thr = zero_snap_thr
        self.vx_ema = 0.0
        self.vy_ema = 0.0
        self.ema_tau = ema_tau
        self._t_prev = None
        self._x_prev = None
        self._y_prev = None

    def _trim(self, t_now):
        # 시간 기반 윈도우: t_now - t0 > window_sec 인 것들 제거
        w = self.window_sec
        while self.points and (t_now - self.points[0][0] > w):
            self.points.popleft()

    def _update_ema(self, vx, vy, dt):
        if dt <= 0: return
        beta = 1 - math.exp(-dt / max(self.ema_tau, 1e-3))  # 0~1
        self.vx_ema = (1 - beta) * self.vx_ema + beta * vx
        self.vy_ema = (1 - beta) * self.vy_ema + beta * vy

    def update(self, x_cm, y_cm, t):
        # 아웃라이어 점프 가드
        if self._t_prev is not None:
            dt = max(0.01, min(0.20, t - self._t_prev))
            dx = x_cm - self._x_prev
            dy = y_cm - self._y_prev
            if math.hypot(dx, dy) > self.max_speed_cmps * dt * 1.5:
                # 비정상 점프면 샘플 무시
                return
        # 샘플 저장
        self.points.append((t, float(x_cm), float(y_cm)))
        self._t_prev, self._x_prev, self._y_prev = t, x_cm, y_cm
        self._trim(t)

    def get_smoothed_position(self):
        if not self.points:
            return 0.0, 0.0
        sx = sum(p[1] for p in self.points)
        sy = sum(p[2] for p in self.points)
        n = len(self.points)
        return sx / n, sy / n

    def get_velocity_ols(self):
        """시간 기반 최소자승으로 vx, vy 추정"""
        n = len(self.points)
        if n < 2: return 0.0, 0.0
        t0 = self.points[0][0]
        # 수치 안정 위해 t를 0 기준으로 이동
        ts = [p[0] - t0 for p in self.points]
        xs = [p[1] for p in self.points]
        ys = [p[2] for p in self.points]
        st = sum(ts); stt = sum(tt*tt for tt in ts)
        sx = sum(xs); sxx = sum(x*x for x in xs)  # sxx는 안 쓰지만 남김
        sy = sum(ys)
        n = float(n)
        denom = (n*stt - st*st)
        if abs(denom) < 1e-6:
            return 0.0, 0.0
        vx = (n*sum(t*x for t, x in zip(ts, xs)) - st*sx) / denom
        vy = (n*sum(t*y for t, y in zip(ts, ys)) - st*sy) / denom
        # EMA 업데이트(선택)
        dt = (self.points[-1][0] - self.points[-2][0]) if len(self.points) >= 2 else 0.033
        self._update_ema(vx, vy, dt)
        return vx, vy

    def get_velocity(self, use_ema=True):
        vx, vy = self.get_velocity_ols()
        if use_ema:
            speed_inst = math.hypot(vx, vy)
            # 정지 스냅
            if speed_inst < self.zero_snap_thr:
                self.vx_ema = 0.0; self.vy_ema = 0.0
            return self.vx_ema, self.vy_ema
        else:
            # 정지 스냅
            if math.hypot(vx, vy) < self.zero_snap_thr:
                return 0.0, 0.0
            return vx, vy

class TrackingManager:
    def __init__(self, window_sec=0.25, max_speed_cmps=200.0, zero_snap_thr=2.0, ema_tau=0.15):
        self.trackers = {}
        self.cfg = dict(window_sec=window_sec, max_speed_cmps=max_speed_cmps,
                        zero_snap_thr=zero_snap_thr, ema_tau=ema_tau)

    def update_all(self, tag_info, current_time):
        for tid, d in tag_info.items():
            if d.get("status") != "On": 
                continue
            coords = d.get("coordinates")
            if not coords: 
                continue
            x_cm, y_cm = coords
            tr = self.trackers.get(tid)
            if tr is None:
                tr = self.trackers[tid] = MovingWindowTracker(**self.cfg)
            tr.update(x_cm, y_cm, float(current_time))

            avg_x, avg_y = tr.get_smoothed_position()
            vx, vy = tr.get_velocity(use_ema=True)
            speed = math.hypot(vx, vy)
            ux, uy = (vx/speed, vy/speed) if speed > 1e-6 else (0.0, 0.0)

            d["smoothed_coordinates_cm"] = (avg_x, avg_y)
            d["velocity_cmps"]           = (vx, vy)
            d["speed_cmps"]              = speed
            d["motion_dir_unit"]         = (ux, uy)
            d["last_tracking_ts"]        = current_time
