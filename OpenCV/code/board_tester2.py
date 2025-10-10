# board_tester.py
# Corner-first board detector with BLACK TAPE preference
# - ROI 두 번 클릭(LMB)로 설정
# - 'r' 리셋, 'f' 재탐색, 'a' 오토 스윕 토글, 'd' 디버그, 'q' 종료
# - 오토(Adaptive) 이진화 + 신뢰도 누적(20/18) + 코너복원
# - 검은 테이프 전용: Black-hat 기반 선 강화 + 테이프 폴라리티 검사(3/4 변 통과 허용)

import cv2
import numpy as np
from collections import deque
from typing import Tuple, List, Dict

# ---------- Optional project imports (graceful fallback) ----------
try:
    from vision.camera import camera_open, Undistorter
except Exception:
    from camera import camera_open, Undistorter

try:
    from config import camera_cfg, board_width_cm, board_height_cm, grid_row, grid_col, board_margin
except Exception:
    camera_cfg = {"type":"opencv", "matrix":None, "dist":None, "size":(1280,720)}
    board_width_cm, board_height_cm = 60.0, 40.0
    grid_row, grid_col = 6, 9
    board_margin = 0.0


# =================== ROI 선택 ===================
roi_tl = None
roi_br = None
selecting_roi = False

def mouse_callback(event, x, y, flags, param):
    global roi_tl, roi_br, selecting_roi
    if event == cv2.EVENT_LBUTTONDOWN:
        if not selecting_roi:
            roi_tl = (x, y); selecting_roi = True
            print(f"[ROI] Top-left set: {roi_tl}")
        else:
            roi_br = (x, y); selecting_roi = False
            print(f"[ROI] Bottom-right set: {roi_br}")


# =================== 유틸 ===================
def clamp(v, a, b): return max(a, min(b, v))
def odd(n):
    n = int(round(n)); return n if n % 2 == 1 else n + 1
def ksize(n):
    n = int(round(n)); n = max(1, n); return (n, n) if n % 2 == 1 else (n+1, n+1)


# =================== 신뢰도 누적 ===================
class ReliabilityAccumulator:
    def __init__(self, window=20, threshold=18):
        self.window = int(window)
        self.threshold = int(threshold)
        self.buf = deque()
        self.sum_img = None
        self.shape = None

    def reset(self):
        self.buf.clear(); self.sum_img = None; self.shape = None

    def set_shape(self, shape: Tuple[int,int]):
        if self.shape != shape:
            self.reset()
            self.shape = shape
            self.sum_img = np.zeros(shape, dtype=np.uint16)

    def push(self, bin_img: np.ndarray):
        if bin_img is None or bin_img.size == 0: return
        h, w = bin_img.shape[:2]
        if self.shape != (h, w):
            self.set_shape((h, w))
        mask01 = (bin_img > 0).astype(np.uint8)
        self.buf.append(mask01)
        self.sum_img += mask01
        if len(self.buf) > self.window:
            oldest = self.buf.popleft()
            self.sum_img -= oldest

    def enough(self) -> bool:
        return len(self.buf) >= min(self.window, 3)

    def get_mask(self) -> np.ndarray:
        if self.sum_img is None: return None
        k = min(self.threshold, self.window)
        rel = (self.sum_img >= k).astype(np.uint8) * 255
        return rel


# =================== 자동 이진화 필터 ===================
class AutoROIFilter:
    def __init__(self):
        self.auto_params = {
            "offset": 0, "clip": 2.0, "block": 21, "C": 10,
            "close": (5,5), "open": (3,3), "invert": True
        }

    def autotune_params(self, roi_gray: np.ndarray):
        H, W = roi_gray.shape[:2]
        S = min(H, W)

        block = odd(clamp(S/20, 15, 51))
        close_k = ksize(clamp(S/100, 3, 7))
        open_k  = (3,3)

        mean = float(np.mean(roi_gray))
        std  = float(np.std(roi_gray))
        p98  = float(np.percentile(roi_gray, 98))

        # 조명 불균일성
        tile = 32 if S >= 320 else 16
        th, tw = max(1, H//tile), max(1, W//tile)
        small = cv2.resize(roi_gray, (tw, th), interpolation=cv2.INTER_AREA)
        uneven = float(np.max(small) - np.min(small)) > 25.0

        # 패턴/에지 밀도
        edges = cv2.Canny(roi_gray, 50, 150)
        edge_density = float(np.count_nonzero(edges)) / float(edges.size)
        pattern_noise = edge_density > 0.08

        # 글레어
        glare = (p98 - mean) > 40.0

        # CLAHE clip
        clip = clamp(1.0 + (20.0 / (std + 1e-6)), 1.2, 3.0)
        if glare: clip = min(clip, 1.8)
        if pattern_noise: clip = max(1.2, clip * 0.9)

        # Brightness offset
        offset = 0
        if mean < 90: offset = +15
        elif mean > 165: offset = -15

        # C
        C = 10
        if uneven: C += 4
        if pattern_noise: C += 4
        if glare: C -= 4
        C = int(clamp(C, 3, 20))

        self.auto_params = {
            "offset": int(offset),
            "clip": float(round(clip,2)),
            "block": int(block),
            "C": int(C),
            "close": close_k,
            "open": open_k,
            "invert": True
        }

    def binarize_with(self, roi_bgr, ap: Dict):
        if roi_bgr.ndim == 3:
            gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi_bgr.copy()

        if ap.get("offset", 0) != 0:
            gray = cv2.convertScaleAbs(gray, alpha=1.0, beta=int(ap["offset"]))

        clahe = cv2.createCLAHE(clipLimit=float(ap["clip"]), tileGridSize=(8,8))
        gray = clahe.apply(gray)

        th = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, int(ap["block"]), int(ap["C"])
        )
        th = cv2.medianBlur(th, 3)
        if ap.get("invert", True):
            th = cv2.bitwise_not(th)

        closed = cv2.morphologyEx(
            th, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, ap["close"])
        )
        opened = cv2.morphologyEx(
            closed, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, ap["open"])
        )
        return opened


# =================== 자동 파라미터 스윕 ===================
class AutoParamSearch:
    def __init__(self):
        self.candidates: List[Dict] = []
        self.idx = 0
        self.active = False

    def _mk_ap(self, base, **over):
        ap = dict(base)
        for k,v in over.items():
            ap[k] = v
        ap["block"] = odd(clamp(ap["block"], 15, 51))
        ap["C"]     = int(clamp(int(ap["C"]), 3, 20))
        ap["clip"]  = float(clamp(float(ap["clip"]), 1.2, 3.0))
        ap["offset"]= int(clamp(int(ap["offset"]), -40, 40))
        ap["close"] = ksize(clamp(ap.get("close", (5,5))[0], 3, 7))
        ap["open"]  = ksize(clamp(ap.get("open", (3,3))[0], 3, 5))
        ap["invert"]= bool(ap.get("invert", True))
        return ap

    def prepare(self, base_ap: Dict):
        cand = []
        # 1st: block & C
        for db in [0, +8, -8, +12, -12]:
            for dC in [0, +4, -4, +8, -8]:
                cand.append(self._mk_ap(base_ap, block=base_ap["block"]+db, C=base_ap["C"]+dC))
        # 2nd: clip/offset
        for dclip in [0.0, +0.5, -0.5]:
            for doff in [0, +15, -15]:
                cand.append(self._mk_ap(base_ap, clip=base_ap["clip"]+dclip, offset=base_ap["offset"]+doff))
        # 3rd: invert flip
        cand.append(self._mk_ap(base_ap, invert=False))

        seen = set(); uniq = []
        for ap in cand:
            key = (ap["block"], ap["C"], ap["clip"], ap["offset"], ap["invert"])
            if key not in seen:
                uniq.append(ap); seen.add(key)

        self.candidates = uniq
        self.idx = 0
        self.active = True

    def next(self, n=3) -> List[Dict]:
        if not self.active or self.idx >= len(self.candidates):
            return []
        j = min(self.idx + n, len(self.candidates))
        return self.candidates[self.idx:j]

    def advance(self, k: int):
        self.idx = min(self.idx + k, len(self.candidates))
        if self.idx >= len(self.candidates):
            self.active = False


# =================== 코너 기반 BoardDetector (검은 테이프 특화) ===================
class BoardDetector:
    def __init__(self):
        self.enable_contour_fallback = True
        self.debug_draw = False
        self.min_len_ratio = 0.15
        self.max_gap_ratio = 0.05
        self.ortho_tol_deg = 30.0
        self.angle_align_tol_deg = 20.0

    # ---------- 내부 유틸 ----------
    @staticmethod
    def _right_angle_score(pts4):
        pts = np.asarray(pts4, dtype=np.float32).reshape(4,2)
        s = 0.0
        for i in range(4):
            p0, p1, p2 = pts[i], pts[(i+1)%4], pts[(i+2)%4]
            v0 = p0 - p1; v2 = p2 - p1
            denom = (np.linalg.norm(v0) * np.linalg.norm(v2) + 1e-6)
            cosv = abs(np.dot(v0, v2)) / denom
            s += (1 - min(cosv, 1.0))
        return s / 4.0

    @staticmethod
    def _order_corners(pts4):
        pts = np.asarray(pts4, dtype=np.float32).reshape(4,2)
        s = pts.sum(axis=1)
        d = np.diff(pts, axis=1).reshape(-1)
        tl = pts[np.argmin(s)]
        br = pts[np.argmax(s)]
        tr = pts[np.argmin(d)]
        bl = pts[np.argmax(d)]
        return np.stack([tl,tr,br,bl],0)

    @staticmethod
    def _poly_area(pts):
        x = pts[:,0]; y=pts[:,1]
        return 0.5*abs(np.dot(x, np.roll(y,-1)) - np.dot(y, np.roll(x,-1)))

    @staticmethod
    def _angle_of_vec(dx, dy):
        th = np.arctan2(dy, dx)
        th = (th + np.pi) % np.pi
        return th

    @staticmethod
    def _line_normal(theta):
        return np.array([np.cos(theta+np.pi/2), np.sin(theta+np.pi/2)], dtype=np.float32)

    @staticmethod
    def _intersect(nA,cA, nB,cB):
        A = np.stack([nA, nB], 0)
        b = -np.array([cA, cB], dtype=np.float32)
        det = float(np.linalg.det(A))
        if abs(det) < 1e-6: return None
        p = np.linalg.solve(A, b).astype(np.float32)
        return p

    # ---------- 검은 테이프 강화(Black-hat) ----------
    def _tape_mask(self, roi_gray: np.ndarray) -> np.ndarray:
        # 흰 바탕 위 검은 선(테이프)만 강조
        g = cv2.GaussianBlur(roi_gray, (5,5), 0)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
        bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, k)  # 밝은 배경 위의 어두운 구조 강조
        # Otsu로 자동 임계
        _, th = cv2.threshold(bh, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # 약한 연결/노이즈 제거
        th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)), iterations=2)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN,  cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)), iterations=1)
        return th

    # ---------- 선분 검출: LSD + Hough ----------
    def _detect_lines(self, edges, min_len, max_gap):
        segs = []
        try:
            lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
            lines, _, _, _ = lsd.detect(edges)
            if lines is not None:
                for l in lines.reshape(-1,4):
                    x1,y1,x2,y2 = map(float, l)
                    if np.hypot(x2-x1, y2-y1) >= min_len:
                        segs.append([x1,y1,x2,y2])
        except Exception:
            pass
        hsegs = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=60,
                                minLineLength=int(min_len), maxLineGap=int(max_gap))
        if hsegs is not None:
            for x1,y1,x2,y2 in hsegs.reshape(-1,4):
                if np.hypot(x2-x1, y2-y1) >= min_len:
                    segs.append([float(x1),float(y1),float(x2),float(y2)])
        if not segs:
            return None
        return np.array(segs, dtype=np.float32)

    # ---------- 각도 2-클러스터 ----------
    def _cluster_angles(self, segs):
        L = []
        for (x1,y1,x2,y2) in segs:
            dx, dy = x2-x1, y2-y1
            if dx == 0 and dy == 0: 
                continue
            theta = self._angle_of_vec(dx, dy)
            L.append([x1,y1,x2,y2, theta])
        if len(L) < 2:
            return None, None, None
        L = np.array(L, dtype=np.float32)
        feats = np.stack([np.cos(2*L[:,4]), np.sin(2*L[:,4])], axis=1).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
        ok, labels, centers = cv2.kmeans(feats, 2, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
        if not ok:
            return None, None, None
        labels = labels.reshape(-1)
        def center_to_theta(c):
            ang = np.arctan2(c[1], c[0]) / 2.0
            if ang < 0: ang += np.pi
            return ang
        th0, th1 = center_to_theta(centers[0]), center_to_theta(centers[1])
        return L, labels, (th0, th1)

    # ---------- 경계 선택(2개 또는 1개) ----------
    def _pick_boundaries(self, group, theta_ref):
        if len(group) == 0: 
            return None
        ns, cs = [], []
        for (x1,y1,x2,y2,theta) in group:
            n = self._line_normal(theta_ref)
            mid = np.array([(x1+x2)/2.0, (y1+y2)/2.0], dtype=np.float32)
            c = - float(n @ mid)
            ns.append(n); cs.append(c)
        ns, cs = np.stack(ns,0), np.array(cs, dtype=np.float32)
        i_min, i_max = int(np.argmin(cs)), int(np.argmax(cs))
        return (ns[i_min], cs[i_min]), (ns[i_max], cs[i_max]), (i_min != i_max)

    # ---------- minAreaRect 보완 ----------
    def _rect_from_minAreaRect(self, edges, expect_dirs=None):
        pts = np.column_stack(np.where(edges > 0))
        if pts.shape[0] < 50:
            return None
        pts = pts[:, ::-1].astype(np.float32)
        rect = cv2.minAreaRect(pts)
        box = cv2.boxPoints(rect).astype(np.float32)
        if expect_dirs is not None:
            v0 = box[1]-box[0]; v1 = box[2]-box[1]
            a0 = self._angle_of_vec(v0[0], v0[1])
            a1 = self._angle_of_vec(v1[0], v1[1])
            def ang_min_diff(a,b): return abs(((a-b+np.pi/2)%np.pi)-np.pi/2)
            th0, th1 = expect_dirs
            d0 = min(ang_min_diff(a0,th0), ang_min_diff(a0,th1))
            d1 = min(ang_min_diff(a1,th0), ang_min_diff(a1,th1))
            if np.degrees(max(d0,d1)) > self.angle_align_tol_deg:
                return None
        return box.reshape(4,1,2).astype(np.float32)

    # ---------- 테이프 폴라리티 검사(사각형 warp 후 4변 평가) ----------
    def _tape_polarity_ok(self, roi_gray: np.ndarray, ordered: np.ndarray) -> bool:
        # 사각형을 정규 해상도로 워프하여, 변 스트립은 어둡고 내부는 밝은지 검사
        w_est = np.linalg.norm(ordered[1]-ordered[0])
        h_est = np.linalg.norm(ordered[3]-ordered[0])
        ar = max(w_est, h_est) / (min(w_est, h_est)+1e-6)
        # 워프 크기 제한(속도/안정)
        LONG = 320
        if w_est >= h_est:
            Ww = LONG; Hw = int(LONG / (ar+1e-6))
        else:
            Hw = LONG; Ww = int(LONG / (ar+1e-6))
        Ww = int(clamp(Ww, 160, 360)); Hw = int(clamp(Hw, 120, 360))
        dst = np.array([[0,0],[Ww-1,0],[Ww-1,Hw-1],[0,Hw-1]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(ordered.astype(np.float32), dst)
        warp = cv2.warpPerspective(roi_gray, M, (Ww, Hw))
        # 밝기 정규화(환경 보정)
        warp = cv2.GaussianBlur(warp, (3,3), 0)

        t = max(2, int(0.05*min(Ww,Hw)))     # 변 스트립 두께
        m = max(2, int(0.05*min(Ww,Hw)))     # 내부 스트립 오프셋
        # 변 평균(더 어두워야 함)
        top    = np.mean(warp[0:t, :])
        bottom = np.mean(warp[Hw-t:Hw, :])
        left   = np.mean(warp[:, 0:t])
        right  = np.mean(warp[:, Ww-t:Ww])
        # 내부 평균(더 밝아야 함)
        top_in    = np.mean(warp[t:t+m, :]) if Hw > t+m else 255
        bottom_in = np.mean(warp[Hw-m-t:Hw-t, :]) if Hw > t+m else 255
        left_in   = np.mean(warp[:, t:t+m]) if Ww > t+m else 255
        right_in  = np.mean(warp[:, Ww-m-t:Ww-t]) if Ww > t+m else 255
        center    = np.mean(warp[m:Hw-m, m:Ww-m]) if (Hw>2*m and Ww>2*m) else np.mean(warp)

        # 조건: 변은 내부보다 충분히 어둡고, 내부/센터는 충분히 밝다
        delta = 20.0  # 밝기 차 임계(0~255)
        bright_min = 100.0
        cond = [
            (top_in    - top)    >= delta and top_in    >= bright_min,
            (bottom_in - bottom) >= delta and bottom_in >= bright_min,
            (left_in   - left)   >= delta and left_in   >= bright_min,
            (right_in  - right)  >= delta and right_in  >= bright_min,
            center >= bright_min
        ]
        passed_edges = sum(cond[:4])
        return (passed_edges >= 3) and cond[4]

    # ---------- Corner-first detect (검은 테이프 우선) ----------
    def detect(self, bin_img, detect_params, dbg_canvas=None, roi_gray=None):
        # detect_params: (min_ar, max_ar, cos, extent, solidity) or with brightness leading
        if len(detect_params) == 6:
            _, min_ar, max_ar, cos_value, extent_value, solidity_value = detect_params
        else:
            min_ar, max_ar, cos_value, extent_value, solidity_value = detect_params

        h, w = bin_img.shape[:2]
        if h < 20 or w < 20:
            return None

        # 0) 검은 테이프 강화 마스크(roi_gray가 있으면 사용)
        if roi_gray is not None:
            tape = self._tape_mask(roi_gray)
            src_bin = cv2.bitwise_and(bin_img, tape)  # 이진화 결과와 AND
        else:
            src_bin = bin_img

        # 1) Canny + 약한 Close
        edges = cv2.Canny(src_bin, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 2) 선분(LSD+Hough)
        min_len = self.min_len_ratio * min(h, w)
        max_gap = self.max_gap_ratio * max(h, w)
        segs = self._detect_lines(edges, min_len, max_gap)
        if segs is None or len(segs) < 2:
            rect_mr = self._rect_from_minAreaRect(edges)
            if rect_mr is not None:
                ordered = self._order_corners(rect_mr.reshape(4,2))
                if roi_gray is None or self._tape_polarity_ok(roi_gray, ordered):
                    return rect_mr
            return self._fallback_contour(src_bin, detect_params)

        if self.debug_draw and dbg_canvas is not None:
            for x1,y1,x2,y2 in segs.astype(int):
                cv2.line(dbg_canvas, (x1,y1), (x2,y2), (255,0,255), 1)

        # 3) 각도 2-클러스터
        L, labels, dirs = self._cluster_angles(segs)
        if L is None:
            rect_mr = self._rect_from_minAreaRect(edges)
            if rect_mr is not None:
                ordered = self._order_corners(rect_mr.reshape(4,2))
                if roi_gray is None or self._tape_polarity_ok(roi_gray, ordered):
                    return rect_mr
            return self._fallback_contour(src_bin, detect_params)

        th0, th1 = dirs
        ang_diff = abs(((th1 - th0 + np.pi/2) % np.pi) - np.pi/2)
        if ang_diff > np.deg2rad(self.ortho_tol_deg):
            rect_mr = self._rect_from_minAreaRect(edges, expect_dirs=dirs)
            if rect_mr is not None:
                ordered = self._order_corners(rect_mr.reshape(4,2))
                if roi_gray is None or self._tape_polarity_ok(roi_gray, ordered):
                    return rect_mr
            return self._fallback_contour(src_bin, detect_params)

        group0 = L[labels==0]
        group1 = L[labels==1]
        b0a_b0b = self._pick_boundaries(group0, th0)
        b1a_b1b = self._pick_boundaries(group1, th1)
        if b0a_b0b is None or b1a_b1b is None:
            rect_mr = self._rect_from_minAreaRect(edges, expect_dirs=dirs)
            if rect_mr is not None:
                ordered = self._order_corners(rect_mr.reshape(4,2))
                if roi_gray is None or self._tape_polarity_ok(roi_gray, ordered):
                    return rect_mr
            return self._fallback_contour(src_bin, detect_params)

        (n0a,c0a),(n0b,c0b), ok0 = b0a_b0b
        (n1a,c1a),(n1b,c1b), ok1 = b1a_b1b

        def make_quad(n0a,c0a,n0b,c0b,n1a,c1a,n1b,c1b):
            pts = [
                self._intersect(n0a,c0a, n1a,c1a),
                self._intersect(n0a,c0a, n1b,c1b),
                self._intersect(n0b,c0b, n1b,c1b),
                self._intersect(n0b,c0b, n1a,c1a),
            ]
            if any(p is None for p in pts): return None
            return np.stack(pts,0)

        quad = None
        if ok0 and ok1:
            quad = make_quad(n0a,c0a,n0b,c0b,n1a,c1a,n1b,c1b)
        else:
            rect_mr = self._rect_from_minAreaRect(edges, expect_dirs=dirs)
            if rect_mr is not None:
                quad = rect_mr.reshape(4,2)

        if quad is None:
            return self._fallback_contour(src_bin, detect_params)

        if not np.all((quad[:,0]>=-3)&(quad[:,0]<=w+3)&(quad[:,1]>=-3)&(quad[:,1]<=h+3)):
            return self._fallback_contour(src_bin, detect_params)

        ordered = self._order_corners(quad)

        # 유효성 검사
        side = np.array([np.linalg.norm(ordered[(i+1)%4]-ordered[i]) for i in range(4)])
        if np.min(side) < 0.08*min(h,w):
            return None
        w_est = np.linalg.norm(ordered[1]-ordered[0])
        h_est = np.linalg.norm(ordered[3]-ordered[0])
        if h_est < 1 or w_est < 1: return None
        ar = float(w_est/h_est) if w_est>=h_est else float(h_est/w_est)
        if len(detect_params) == 6:
            _, min_ar, max_ar, cos_value, extent_value, solidity_value = detect_params
        else:
            min_ar, max_ar, cos_value, extent_value, solidity_value = detect_params
        if not (min_ar < ar < max_ar):
            return self._fallback_contour(src_bin, detect_params)
        ra = self._right_angle_score(ordered)
        if ra < (1 - cos_value):
            return self._fallback_contour(src_bin, detect_params)

        area = self._poly_area(ordered)
        x,y,wbb,hbb = cv2.boundingRect(ordered.astype(np.int32))
        extent = float(area) / float(max(1,wbb*hbb))
        solidity = 1.0
        if extent < extent_value or solidity < solidity_value:
            return self._fallback_contour(src_bin, detect_params)

        # ★ 테이프 폴라리티 검사: 4변 중 3변 이상 '검은 테이프' + 내부 밝기
        if roi_gray is not None and not self._tape_polarity_ok(roi_gray, ordered):
            return None

        if self.debug_draw and dbg_canvas is not None:
            for p in ordered.astype(int):
                cv2.circle(dbg_canvas, (p[0],p[1]), 4, (0,255,255), -1)

        return ordered.reshape(4,1,2).astype(np.float32)

    # ---------- Optional fallback: contour ----------
    def _fallback_contour(self, bin_img, detect_params):
        if not self.enable_contour_fallback:
            return None
        if len(detect_params) == 6:
            _, min_ar, max_ar, cos_value, extent_value, soild_value = detect_params
        else:
            min_ar, max_ar, cos_value, extent_value, soild_value = detect_params

        contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        h_img, w_img = bin_img.shape
        best=None; best_score=-1

        def _right_angle_score(pts4):
            pts = np.asarray(pts4, dtype=np.float32).reshape(4,2)
            s = 0.0
            for i in range(4):
                p0, p1, p2 = pts[i], pts[(i+1)%4], pts[(i+2)%4]
                v0 = p0 - p1; v2 = p2 - p1
                denom = (np.linalg.norm(v0) * np.linalg.norm(v2) + 1e-6)
                cosv = abs(np.dot(v0, v2)) / denom
                s += (1 - min(cosv, 1.0))
            return s / 4.0

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            if len(approx) != 4: continue
            area = cv2.contourArea(approx)
            if area < 0.02 * (h_img*w_img): continue
            x,y,w,h = cv2.boundingRect(approx)
            if h == 0: continue
            ar = float(w)/h if w>=h else float(h)/w
            if not (min_ar < ar < max_ar): continue
            ra = _right_angle_score(approx.reshape(4,2))
            if ra < (1 - cos_value): continue
            rectangularity = area / float(w*h + 1e-6)
            hull = cv2.convexHull(approx)
            hull_area = float(cv2.contourArea(hull) + 1e-6)
            solidity = area / hull_area
            extent   = area / float(w*h + 1e-6)
            if extent < extent_value or solidity < soild_value or rectangularity < 0.80:
                continue
            area_n = area / float(h_img*w_img)
            score = 0.5*area_n + 0.2*rectangularity + 0.2*ra + 0.1*solidity
            if score > best_score:
                best_score = score; best = approx
        return best

    # ---------- Helper ----------
    @staticmethod
    def get_board_pts(rect_full):
        pts = rect_full.reshape(4, 2).astype(np.float32)
        sum_pts = pts.sum(axis=1)
        diff_pts = np.diff(pts, axis=1)
        tl = pts[np.argmin(sum_pts)]
        br = pts[np.argmax(sum_pts)]
        tr = pts[np.argmin(diff_pts)]
        bl = pts[np.argmax(diff_pts)]
        ordered = np.array([tl, tr, br, bl], dtype=np.float32)
        width_px  = np.linalg.norm(tr - tl)
        height_px = np.linalg.norm(bl - tl)
        return ordered, width_px, height_px


# =================== 메인 ===================
def main():
    global roi_tl, roi_br, selecting_roi

    RELI_WINDOW = 20
    RELI_K = 18
    MAX_TRIES_PER_FRAME = 3

    cap, fps = camera_open(source=None)
    undist = Undistorter(camera_cfg['type'], camera_cfg['matrix'], camera_cfg['dist'], camera_cfg['size'])

    WIN = "Board Tester (Corner/Tape)"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, mouse_callback)

    roi_filter = AutoROIFilter()
    reli = ReliabilityAccumulator(window=RELI_WINDOW, threshold=RELI_K)
    search = AutoParamSearch()
    detector = BoardDetector()

    prev_roi_rect = None
    last_rect_full = None
    autosweep = False
    show_debug = False
    detector.debug_draw = False

    status = "Set ROI by two clicks (LMB). r:reset f:refind a:auto d:debug q:quit"

    DETECT_PARAMS = (0.50, 2.50, 0.30, 0.70, 0.90)

    while True:
        ret, raw = cap.read()
        if not ret:
            print("[ERROR] frame grab failed")
            break

        frame, _ = undist.undistort(raw)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()
        H, W = gray.shape[:2]

        if roi_tl and roi_br:
            x1, y1 = roi_tl; x2, y2 = roi_br
            x1, x2 = sorted([x1, x2]); y1, y2 = sorted([y1, y2])
            if x2 > x1 and y2 > y1:
                roi = gray[y1:y2, x1:x2]
                if prev_roi_rect != (x1,y1,x2,y2):
                    prev_roi_rect = (x1,y1,x2,y2)
                    last_rect_full = None
                    reli.set_shape(roi.shape)
                    search.active = False
                    status = "ROI updated. Auto tuning & searching..."

                # Auto-tune base params
                roi_filter.autotune_params(roi)
                base_ap = dict(roi_filter.auto_params)

                found_rect = None
                tried = 0

                if autosweep and (last_rect_full is None) and not search.active:
                    search.prepare(base_ap)

                def try_with(ap):
                    nonlocal found_rect
                    filt = roi_filter.binarize_with(roi, ap)
                    reli.push(filt)
                    rel_mask = reli.get_mask()
                    src = rel_mask if reli.enough() else filt
                    dbg = None
                    if show_debug:
                        dbg = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
                    # ★ roi_gray 전달 → 검은테이프 폴라리티/black-hat 사용
                    rect_roi = detector.detect(src, DETECT_PARAMS, dbg_canvas=dbg, roi_gray=roi)
                    if show_debug and dbg is not None:
                        cv2.imshow("DEBUG_ROI", dbg)
                    return rect_roi

                if autosweep and search.active:
                    for ap in search.next(MAX_TRIES_PER_FRAME):
                        rect_roi = try_with(ap)
                        tried += 1
                        if rect_roi is not None:
                            found_rect = rect_roi
                            base_ap = ap
                            break
                    search.advance(tried)
                else:
                    rect_roi = try_with(base_ap)
                    found_rect = rect_roi

                if found_rect is not None:
                    rect_full = found_rect.copy()
                    rect_full[:,0,0] += x1; rect_full[:,0,1] += y1
                    last_rect_full = rect_full
                    status = "Board detected."
                else:
                    if last_rect_full is None:
                        status = "Searching..."

                cv2.rectangle(display, (x1,y1), (x2,y2), (0,0,255), 2)
        else:
            status = "Set ROI by two clicks (LMB). r:reset f:refind a:auto d:debug q:quit"

        if last_rect_full is not None:
            ordered, _, _ = BoardDetector.get_board_pts(last_rect_full)
            cv2.polylines(display, [ordered.astype(np.int32)], True, (0,255,0), 2)
            cv2.putText(display, "LOCKED", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)

        cv2.putText(display, status, (20, H-30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220,220,220), 2, cv2.LINE_AA)
        if roi_tl: cv2.circle(display, roi_tl, 5, (255,0,0), -1)
        if roi_tl and roi_br: cv2.circle(display, roi_br, 5, (255,0,0), -1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            roi_tl = roi_br = None
            selecting_roi = False
            prev_roi_rect = None
            last_rect_full = None
            reli.reset()
            search.active = False
            status = "Reset. Set ROI again."
        elif key == ord('f'):
            last_rect_full = None
            search.active = False
            status = "Force refind..."
        elif key == ord('a'):
            autosweep = not autosweep
            status = f"Auto sweep: {'ON' if autosweep else 'OFF'}"
        elif key == ord('d'):
            show_debug = not show_debug
            detector.debug_draw = show_debug
            if not show_debug:
                try: cv2.destroyWindow("DEBUG_ROI")
                except: pass

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
