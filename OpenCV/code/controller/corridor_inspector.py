from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, Iterable

import math
from controller.collision_guard import GuardConfig

Vec2 = Tuple[float, float]

def _unit(v: Vec2) -> Vec2:
    n = math.hypot(v[0], v[1])
    if n <= 1e-6:
        return (0.0, 0.0)
    return (v[0]/n, v[1]/n)

def _rot(v: Vec2, deg: float) -> Vec2:
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    return (v[0]*c - v[1]*s, v[0]*s + v[1]*c)

def _point_in_forward_corridor(p: Vec2, u: Vec2, q: Vec2, half_w: float, length: float) -> bool:
    dx, dy = (q[0]-p[0], q[1]-p[1])
    along = dx*u[0] + dy*u[1]
    perp = abs(-dy*u[0] + dx*u[1])
    return (0.0 <= along <= length) and (perp < half_w)

@dataclass
class CorridorInspector:
    cfg: GuardConfig

    def _pose(self, rid: str, tag_info: Dict) -> Optional[Tuple[Vec2, Vec2]]:
        d = tag_info.get(int(rid)) or tag_info.get(str(rid)) or {}
        pos = d.get("center") or d.get("position_cm") or d.get("pos") or None
        if not pos or not isinstance(pos, (tuple, list)) or len(pos) < 2:
            return None
        x, y = float(pos[0]), float(pos[1])

        if "forward_vec" in d and isinstance(d["forward_vec"], (tuple, list)) and len(d["forward_vec"]) >= 2:
            fx, fy = float(d["forward_vec"][0]), float(d["forward_vec"][1])
            u = _unit((fx, fy))
        else:
            h = d.get("heading_deg") or d.get("theta_deg") or d.get("heading") or 0.0
            rad = math.radians(float(h))
            u = (math.cos(rad), math.sin(rad))
        if math.hypot(u[0], u[1]) <= 1e-6:
            return None
        return ((x, y), _unit(u))

    def _corridor_length(self, rid: str, tag_info: Dict) -> float:
        d = tag_info.get(int(rid)) or tag_info.get(str(rid)) or {}
        v = float(d.get("speed_cmps", 20.0))
        return float(self.cfg.step_cm + self.cfg.eps_step_cm + v * self.cfg.tau_latency_s)

    def _apply_rotation_if_two_stage(self, u: Vec2, command_set: Iterable[dict]) -> Vec2:
        if not command_set:
            return u
        try:
            first = next(iter(command_set))
            c = first.get("command")
            if isinstance(c, str) and c.endswith("_modeOnly") and (c.startswith("L") or c.startswith("R")):
                sign = 1.0 if c.startswith("L") else -1.0
                angle = float(c[1:].split("_")[0])
                return _unit(_rot(u, sign * angle))
        except Exception:
            pass
        return u

    def is_clear_for_move(self, rid: str, command_set: Iterable[dict], tag_info: Dict) -> bool:
        pose = self._pose(rid, tag_info)
        if pose is None:
            return True
        p, u0 = pose
        u = self._apply_rotation_if_two_stage(u0, command_set)
        L = self._corridor_length(rid, tag_info)
        half_w = float(self.cfg.corridor_half_cm)
        for tid, data in tag_info.items():
            try:
                if str(tid) == str(rid):
                    continue
                if data.get("status") and data.get("status") != "On":
                    continue
                pos = data.get("center") or data.get("position_cm") or data.get("pos") or None
                if not pos:
                    continue
                q = (float(pos[0]), float(pos[1]))
                if _point_in_forward_corridor(p, u, q, half_w, L):
                    return False
            except Exception:
                continue
        return True

    def is_clear_for_release(self, rid: str, tag_info: Dict) -> bool:
        pose = self._pose(rid, tag_info)
        if pose is None:
            return True
        p, u = pose
        L = self._corridor_length(rid, tag_info)
        half_w = float(self.cfg.corridor_half_cm)
        for tid, data in tag_info.items():
            try:
                if str(tid) == str(rid):
                    continue
                if data.get("status") and data.get("status") != "On":
                    continue
                pos = data.get("center") or data.get("position_cm") or data.get("pos") or None
                if not pos:
                    continue
                q = (float(pos[0]), float(pos[1]))
                if _point_in_forward_corridor(p, u, q, half_w, L):
                    return False
            except Exception:
                continue
        return True
