import cv2
import numpy as np
from dataclasses import dataclass
from typing import Dict

@dataclass
class BoardDetectionResult:
    corners: np.ndarray
    origin: np.ndarray
    width_px: float
    height_px: float
    cm_per_px: tuple
    perspective_matrix: np.ndarray
    warped: np.ndarray
    warped_resized: np.ndarray
    grid_reference: dict | None = None 
    found: bool = True

class BoardDetector:
    """
    v3 기반:
    - 자동 ROI 이진화 + 간소 파라미터 스윕(비차단 step) 내장
    - 프레임당 소량 후보만 시도하여 메인 루프를 블로킹하지 않음
    - Reliability 누적(윈도/스레시홀드)은 얇게 보강용으로 유지
    """
    # ======== 내부 유틸 ========
    @staticmethod
    def _clamp(v, a, b): return max(a, min(b, v))
    @staticmethod
    def _odd(n):
        n = int(round(n))
        return n if n % 2 == 1 else n + 1
    @staticmethod
    def _ksize(n):
        n = int(round(n)); n = max(1, n)
        return (n, n) if n % 2 == 1 else (n+1, n+1)
    
    @staticmethod
    def draw_inf_line(canvas, n, c, color, thickness=1):
        h, w = canvas.shape[:2]
        # n·p + c = 0  → 두 점 구해 화면 가장자리와 교차시키는 식으로
        import numpy as np, cv2
        # candidate points along image bounds
        pts = []
        for x in [0, w-1]:
            # n=[nx, ny], nx*x + ny*y + c = 0 → y = -(nx*x + c)/ny
            ny = n[1] if abs(n[1])>1e-6 else 1e-6
            y = int(round(-(n[0]*x + c)/ny))
            pts.append((x,y))
        for y in [0, h-1]:
            nx = n[0] if abs(n[0])>1e-6 else 1e-6
            x = int(round(-(n[1]*y + c)/nx))
            pts.append((x,y))
        # pick two that are inside frame
        pts = [(x,y) for (x,y) in pts if -w<=x<=2*w and -h<=y<=2*h]
        if len(pts)>=2:
            cv2.line(canvas, pts[0], pts[1], color, thickness)


    class BoardROIFilter:
        def __init__(self):
            self.auto_params = {
                "offset": 0, "clip": 2.0, "block": 21, "C": 10,
                "close": (5,5), "open": (3,3), "invert": True
            }
        
        @staticmethod
        def clamp(v, a, b): return max(a, min(b, v))
        @staticmethod
        def odd(n):
            n = int(round(n)); return n if n % 2 == 1 else n + 1
        @staticmethod
        def ksize(n):
            n = int(round(n)); n = max(1, n); return (n, n) if n % 2 == 1 else (n+1, n+1)

        def autotune_params(self, roi_gray: np.ndarray):
            H, W = roi_gray.shape[:2]
            S = min(H, W)

            block = self.odd(self.clamp(S/20, 15, 51))
            close_k = self.ksize(self.clamp(S/100, 3, 7))
            open_k  = self.ksize(self.clamp(S/160, 3, 7))

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
            clip = self.clamp(1.0 + (20.0 / (std + 1e-6)), 1.2, 3.0)
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
            C = int(self.clamp(C, 3, 20))

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
            opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, ap["open"]))
            h, w = opened.shape[:2]
            min_area = max(20, int(0.005 * h * w))
            n, labels, stats, _ = cv2.connectedComponentsWithStats(opened, connectivity=8)
            out = np.zeros_like(opened)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] >= min_area:
                    out[labels == i] = 255
            return out
        
    # ======== 생성자 ========
    def __init__(self, board_width_cm, board_height_cm, grid_width, grid_height, board_margin):
        # 물리/그리드 파라미터 저장
        self.board_width_cm = float(board_width_cm)
        self.board_height_cm = float(board_height_cm)
        self.grid_width = int(grid_width)
        self.grid_height = int(grid_height)
        self.board_margin = float(board_margin)

        # 탐색 파라미터
        self.enable_contour_fallback = True
        self.debug_draw = True
        self.min_len_ratio = 0.05
        self.max_gap_ratio = 0.05
        self.ortho_tol_deg = 30.0

        # 내부 필터/상태
        self.roi_filter = self.BoardROIFilter()
        self._result = None
        self._locked = False
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
    
    @staticmethod
    def weighted_dir_theta(group):
        if len(group) == 0:
            return None
        acc_x, acc_y = 0.0, 0.0
        for x1,y1,x2,y2,theta in group:
            w = float(np.hypot(x2-x1, y2-y1))  # 선분 길이 가중치
            acc_x += np.cos(2*theta) * w
            acc_y += np.sin(2*theta) * w
        if acc_x == 0 and acc_y == 0:
            return None
        ang = 0.5 * np.arctan2(acc_y, acc_x)
        if ang < 0: ang += np.pi
        return ang

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
    
    def _project_c_values(self, group, theta_ref, use_endpoints=True):
        """
        그룹 선분들을 theta_ref의 법선방향으로 투영해 1D 값 c를 만든다.
        use_endpoints=True면 엔드포인트를 사용(더 안정적), False면 중점 사용.
        """
        if len(group) == 0:
            return None, None, None
        n = self._line_normal(theta_ref)
        cs = []
        ws = []
        for (x1,y1,x2,y2,theta) in group:
            L = float(np.hypot(x2-x1, y2-y1))
            if use_endpoints:
                c1 = - float(n @ np.array([x1,y1], dtype=np.float32))
                c2 = - float(n @ np.array([x2,y2], dtype=np.float32))
                cs.extend([c1, c2]); ws.extend([L*0.5, L*0.5])
            else:
                mid = np.array([(x1+x2)/2.0, (y1+y2)/2.0], dtype=np.float32)
                cs.append(- float(n @ mid)); ws.append(L)
        return np.array(cs, dtype=np.float32), np.array(ws, dtype=np.float32), n

    def _split_by_position(self, group, theta_ref,
                        min_gap_px=10, alpha_std=0.7,
                        min_count=3, min_len_sum_ratio=0.05, img_size=None):
        """
        2단계 분할: 같은 각도 군집을 위치(법선 투영값 c) 기준으로 좌/우(또는 상/하) 2개로 분할.
        - 너무 타이트하게 안 나누도록 간격/지지조건을 걸어 과분할 방지.
        - 분할 불충족이면 [group] 그대로 리턴.
        """
        cs, ws, n = self._project_c_values(group, theta_ref, use_endpoints=True)
        if cs is None or len(cs) < 4:
            return [group]

        # 1D kmeans(k=2)
        Z = cs.reshape(-1,1).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
        ok, labels, centers = cv2.kmeans(Z, 2, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
        if not ok:
            return [group]
        labels = labels.reshape(-1)
        mu = np.sort(centers.reshape(-1))
        gap = abs(mu[1]-mu[0])

        # "널널한" 분할 여부 판단
        std_all = float(np.std(cs)) + 1e-6
        min_gap = float(min_gap_px)
        if img_size is not None:
            # 이미지 크기에 비례해 최소 간격 자동 상향 (짧은 변의 1~2%)
            short_side = min(img_size)
            min_gap = max(min_gap, 0.012 * short_side)

        if gap < max(min_gap, alpha_std * std_all):
            return [group]

        # 원래 선분 단위로 좌/우 할당(중점 투영값으로)
        n = self._line_normal(theta_ref)
        c_mid = []
        for (x1,y1,x2,y2,theta) in group:
            mid = np.array([(x1+x2)/2.0, (y1+y2)/2.0], dtype=np.float32)
            c_mid.append(- float(n @ mid))
        c_mid = np.array(c_mid, dtype=np.float32)
        thr = float(np.mean(mu))  # 두 중심 사이
        left_idx  = np.where(c_mid <= thr)[0]
        right_idx = np.where(c_mid >  thr)[0]

        gL = group[left_idx]  if len(left_idx)  > 0 else np.empty((0,5), np.float32)
        gR = group[right_idx] if len(right_idx) > 0 else np.empty((0,5), np.float32)

        # 지지 조건(개수/길이합) 체크
        def total_len(g):
            s = 0.0
            for x1,y1,x2,y2,theta in g:
                s += float(np.hypot(x2-x1, y2-y1))
            return s
        if img_size is not None:
            short_side = min(img_size)
            min_len_sum = min_len_sum_ratio * short_side
        else:
            min_len_sum = 0.0

        okL = (len(gL) >= min_count) and (total_len(gL) >= min_len_sum)
        okR = (len(gR) >= min_count) and (total_len(gR) >= min_len_sum)
        if not okL or not okR:
            return [group]

        # 항상 작은 c가 "왼/상", 큰 c가 "오/하"가 되도록 정렬
        if np.mean(c_mid[left_idx]) > np.mean(c_mid[right_idx]):
            gL, gR = gR, gL
        return [gL, gR]

    def _weighted_quantile(self, values, weights, q):
        # values, weights: 1D numpy arrays
        order = np.argsort(values)
        v = values[order]; w = weights[order]
        cw = np.cumsum(w)
        t = q * (cw[-1] + 1e-6)
        idx = int(np.searchsorted(cw, t))
        idx = max(0, min(idx, len(v)-1))
        return float(v[idx])

    def _single_boundary_from_group(self, group, theta_ref, side='auto', q=0.90):
        """
        하위군집 하나(=한 변)에서 '최외곽' 직선 하나 추정.
        - side: 'low' (왼/상), 'high' (오/하), 'auto'(데이터로 판단)
        - q: 분위수 (high면 0.9~0.95, low면 0.05~0.10 권장)
        """
        if len(group) == 0:
            return None
        cs, ws, n = self._project_c_values(group, theta_ref, use_endpoints=True)
        if cs is None or len(cs) == 0:
            return None

        if side == 'auto':
            # 평균 c로 반 나눠서 자동 판단
            if np.mean(cs) <= np.median(cs):
                side = 'low'
            else:
                side = 'high'

        if side == 'low':
            c = self._weighted_quantile(cs, ws, 1.0 - q)  # 예: 0.10
        elif side == 'high':
            c = self._weighted_quantile(cs, ws, q)        # 예: 0.90
        else:
            # fallback: 중앙값
            order = np.argsort(cs)
            cs_o, ws_o = cs[order], ws[order]
            cumsum = np.cumsum(ws_o) / (float(ws_o.sum()) + 1e-6)
            idx = int(np.searchsorted(cumsum, 0.5))
            c = float(cs_o[min(idx, len(cs_o)-1)])

        return (self._line_normal(theta_ref), float(c))

    def make_quad(self,n0a,c0a,n0b,c0b,n1a,c1a,n1b,c1b):
        pts = [
            self._intersect(n0a,c0a, n1a,c1a),
            self._intersect(n0a,c0a, n1b,c1b),
            self._intersect(n0b,c0b, n1b,c1b),
            self._intersect(n0b,c0b, n1a,c1a),
        ]
        if any(p is None for p in pts): return None
        return np.stack(pts,0)

    def detect(self, roi_gray, detect_params, tape_mask=None):
        return self._detect_board_from_line(roi_gray, detect_params, tape_mask=tape_mask)

    def process(self, frame_gray, detect_params, roi_offset=None, rect_override=None) -> BoardDetectionResult | None:
        if self._locked and self._result is not None:
            H = self._result.perspective_matrix
            w_px = int(self._result.width_px)
            h_px = int(self._result.height_px)
            warped = cv2.warpPerspective(frame_gray, H, (w_px, h_px))
            warped_resized = cv2.resize(warped, (frame_gray.shape[1]//2, frame_gray.shape[1]//2))
            self._result.warped = warped
            self._result.warped_resized = warped_resized
            return self._result
        
        tape_mask = self._tape_mask(frame_gray)
        self.roi_filter.autotune_params(frame_gray)
        ap = dict(self.roi_filter.auto_params)
        bin_img = self.roi_filter.binarize_with(frame_gray, ap)

        rect = rect_override if rect_override is not None else self.detect(bin_img, detect_params, tape_mask=tape_mask)
        if rect is None:
            return self._result  # 기존 있으면 유지, 없으면 None
        rect_input = rect  # ROI 좌표
        rect_full = rect.copy()
        if roi_offset is not None:
            ox, oy = roi_offset
            rect_full[:,0,0] += ox
            rect_full[:,0,1] += oy

        if rect is not None:
            corners_full, width_px, height_px = self._get_board_pts(rect_full)
            corners_roi, _, _ = self._get_board_pts(rect_input)
            origin = self._get_board_origin(corners_full[0])
            warped, warped_resized, perspective_matrix, warped_w_px, warped_h_px = self._warp_board(frame_gray, corners_roi, width_px, height_px)
            cm_per_px = self._calculate_cm_per_px(warped_w_px, warped_h_px)
            self._result = BoardDetectionResult(
                corners=corners_full, origin=origin,
                width_px=width_px, height_px=height_px,
                cm_per_px=cm_per_px,
                perspective_matrix=perspective_matrix,
                warped=warped, warped_resized=warped_resized,
            )
            return self._result
        else:
            return self._result

    def _detect_board_from_binary(self, gray_bin, detect_params):
        if len(detect_params) == 6:
            _, min_ar, max_ar, cos_value, extent_value, soild_value = detect_params
        else:
            min_ar, max_ar, cos_value, extent_value, soild_value = detect_params

        contours, _ = cv2.findContours(gray_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        h_img, w_img = gray_bin.shape
        candidates = []

        def _right_angle_score(pts4):
            pts = np.asarray(pts4, dtype=np.float32).reshape(4,2)
            s = 0.0
            for i in range(4):
                p0, p1, p2 = pts[i], pts[(i+1)%4], pts[(i+2)%4]
                v0 = p0 - p1
                v2 = p2 - p1
                denom = (np.linalg.norm(v0) * np.linalg.norm(v2) + 1e-6)
                cosv = abs(np.dot(v0, v2)) / denom
                s += (1 - min(cosv, 1.0))
            return s / 4.0

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            if len(approx) != 4:
                continue
            area = cv2.contourArea(approx)
            if area < 0.02 * (h_img*w_img):
                continue
            x,y,w,h = cv2.boundingRect(approx)
            if h == 0:
                continue
            ar = float(w)/h
            if not (min_ar < ar < max_ar):
                continue

            ra = _right_angle_score(approx.reshape(4,2))
            if ra < (1 - cos_value):
                continue

            rectangularity = area / float(w*h + 1e-6)
            hull = cv2.convexHull(approx)
            hull_area = float(cv2.contourArea(hull) + 1e-6)
            solidity = area / hull_area
            extent   = area / float(w*h + 1e-6)

            if extent < extent_value or solidity < soild_value or rectangularity < 0.80:
                continue

            area_n = area / float(h_img*w_img)
            score = 0.5*area_n + 0.2*rectangularity + 0.2*ra + 0.1*solidity
            candidates.append((approx, score))

        if not candidates:
            return None
        best, _ = max(candidates, key=lambda x:x[1])
        return best
    
    def _detect_board_from_line(self, bin_img, detect_params, tape_mask=None):
        if len(detect_params) == 6:
            _, min_ar, max_ar, cos_value, extent_value, solidity_value = detect_params
        else:
            min_ar, max_ar, cos_value, extent_value, solidity_value = detect_params
        
        dbg_canvas = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
        if tape_mask is not None:
            src_bin = cv2.bitwise_and(bin_img, tape_mask)
        else:
            src_bin = bin_img.copy()
        
        overlay = {"segs": None, "inf_lines": [], "corners": None}
        
        h, w = src_bin.shape[:2]
        if h < 20 or w < 20:
            return None

        # 1) Canny + 약한 Close
        edges = cv2.Canny(src_bin, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 2) 선분(LSD+Hough)
        min_len = self.min_len_ratio * min(h, w)
        max_gap = self.max_gap_ratio * max(h, w)
        segs = self._detect_lines(edges, min_len, max_gap)
        if segs is None or len(segs) < 2:
            if self.debug_draw: self._draw_overlay_once(dbg_canvas, overlay)
            return None
        overlay["segs"] = segs.copy()
        
        # 3) 각도 기준 클러스터
        L, labels, dirs = self._cluster_angles(segs)
        if L is None:
            return None
        group0 = L[labels==0]
        group1 = L[labels==1]
        
        th0 = self.weighted_dir_theta(group0)
        th1 = self.weighted_dir_theta(group1)
        if th0 is None or th1 is None:
            return None
        sub0 = self._split_by_position(group0, th0, img_size=(w, h))
        sub1 = self._split_by_position(group1, th1, img_size=(w, h))

        lines0 = []
        lines1 = []
        if len(sub0) == 2:
            th0L = self.weighted_dir_theta(sub0[0])  # 왼/상
            th0R = self.weighted_dir_theta(sub0[1])  # 오른/하
            # 실패 시 부모 각 사용
            if th0L is None: th0L = th0
            if th0R is None: th0R = th0
            b0L = self._single_boundary_from_group(sub0[0], th0L, side='low',  q=0.90)  # 왼/상 → 낮은 분위수
            b0R = self._single_boundary_from_group(sub0[1], th0R, side='high', q=0.90)  # 오른/하 → 높은 분위수
            if b0L is not None and b0R is not None:
                (n0a,c0a),(n0b,c0b) = sorted([b0L, b0R], key=lambda z: z[1])
                lines0 = [(n0a,c0a),(n0b,c0b)]
        else:
            pb = self._pick_boundaries(group0, th0)
            if pb is not None:
                (n0a,c0a),(n0b,c0b),_ = pb
                lines0 = [(n0a,c0a),(n0b,c0b)]

        # --- 군집1: 하위군집별 각도 재추정 ---
        if len(sub1) == 2:
            th1T = self.weighted_dir_theta(sub1[0])
            th1B = self.weighted_dir_theta(sub1[1])
            if th1T is None: th1T = th1
            if th1B is None: th1B = th1
            b1T = self._single_boundary_from_group(sub1[0], th1T, side='low',  q=0.90)
            b1B = self._single_boundary_from_group(sub1[1], th1B, side='high', q=0.90)
            if b1T is not None and b1B is not None:
                (n1a,c1a),(n1b,c1b) = sorted([b1T, b1B], key=lambda z: z[1])
                lines1 = [(n1a,c1a),(n1b,c1b)]
        else:
            pb = self._pick_boundaries(group1, th1)
            if pb is not None:
                (n1a,c1a),(n1b,c1b),_ = pb
                lines1 = [(n1a,c1a),(n1b,c1b)]

        # 분기 실패 시 폴백
        if len(lines0) != 2 or len(lines1) != 2:
            return None
        (n0a,c0a),(n0b,c0b) = lines0
        (n1a,c1a),(n1b,c1b) = lines1
        overlay["inf_lines"] = [(n0a,c0a),(n0b,c0b),(n1a,c1a),(n1b,c1b)]

        def ortho_dev(a, b):
            # 0..π 로 정규화된 두 각도의 차
            d = abs((a - b) % np.pi)
            if d > np.pi/2:
                d = np.pi - d     # 0..π/2 범위의 "평행 편차"
            return abs(d - np.pi/2)  # 직교에서의 편차(0=직교)
        # --- 바깥선 각으로 직교성 검사 ---
        def n2theta(n):
            return (np.arctan2(n[1], n[0]) - np.pi/2) % np.pi
        pairs = [
            (n2theta(n0a), n2theta(n1a)),
            (n2theta(n0a), n2theta(n1b)),
            (n2theta(n0b), n2theta(n1a)),
            (n2theta(n0b), n2theta(n1b)),
        ]
        ang_diff_outer = min(ortho_dev(a, b) for a, b in pairs)

        # 직교에 충분히 가깝지 않으면 폴백
        if ang_diff_outer > np.deg2rad(self.ortho_tol_deg):
            return None
        quad = self.make_quad(n0a,c0a,n0b,c0b,n1a,c1a,n1b,c1b)
        if quad is None or not np.all((quad[:,0]>=-3)&(quad[:,0]<=w+3)&(quad[:,1]>=-3)&(quad[:,1]<=h+3)):
            return None
        ordered = self._order_corners(quad)
        overlay["corners"] = ordered.copy()
       
        if self.debug_draw:
            self._draw_overlay_once(dbg_canvas, overlay)

        return ordered.reshape(4,1,2).astype(np.float32)

    def generate_coordinate_system(self):
        if self._result is None:
            raise RuntimeError("Board not yet detected")
        src_pts = self._result.corners
        dst_pts = np.array([
            [0, 0],
            [self.board_width_cm, 0],
            [self.board_width_cm, self.board_height_cm],
            [0,                 self.board_height_cm]
            ], dtype=np.float32)
        H_metric = cv2.getPerspectiveTransform(src_pts, dst_pts)

        cell_centers = []
        inner_board_width_cm = self.board_width_cm - 2 * self.board_margin
        inner_board_height_cm = self.board_height_cm - 2 * self.board_margin
        cw = inner_board_width_cm  / self.grid_width
        ch = inner_board_height_cm / self.grid_height
        for row in range(self.grid_height):
            for col in range(self.grid_width):
                cx = (col + 0.5) * cw
                cy = (row + 0.5) * ch
                cell_centers.append((cx, cy))

        horizontal = [((0, i*ch), (inner_board_width_cm, i*ch))
                    for i in range(self.grid_height+1)]
        vertical   = [((j*cw, 0), (j*cw, inner_board_height_cm))
                    for j in range(self.grid_width+1)]

        return {
            "H_metric":     H_metric,
            "cell_centers": cell_centers,
            "grid_lines":   {"horizontal": horizontal, "vertical": vertical}
        }

    def lock(self):
        if self._result is None:
            return
        self._result.grid_reference = self.generate_coordinate_system()
        self._locked = True

    def reset(self):
        self._locked = False
        self._result = None

    def get_result(self) -> BoardDetectionResult | None:
        return self._result
    
    def draw(self, frame, result: BoardDetectionResult):
        if result is None:
            return
        cv2.drawContours(frame, [result.corners.astype(np.int32)], -1, (255, 0, 0), 2)
        cv2.circle(frame, tuple(result.origin[:2].astype(int)), 5, (0, 0, 255), -1)
        if result.grid_reference is None:
            return
        overlay = self.get_grid_overlay_points(result)
        if overlay is None:
            return
        for x, y in overlay["centers"]:
            cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 0), -1)
        for pt1, pt2 in overlay["lines"]:
            cv2.line(frame, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (0, 255, 255), 1)

    def _draw_overlay_once(self, dbg_canvas, overlay):
        if dbg_canvas is None:
            return

        # 1) 원시 선분(그대로 유지하고 싶으면 색 자유)
        if overlay.get("segs") is not None:
            for x1,y1,x2,y2 in overlay["segs"].astype(int):
                cv2.line(dbg_canvas, (x1,y1), (x2,y2), (255,0,255), 1)  # magenta

        # 2) 무한 직선 4개 (연두 제외, 서로 다른 색)
        #    BGR: 노랑(0,255,255), 시안(255,255,0), 주황(0,128,255), 보라(255,0,255)
        line_colors = [
            (0,255,255),   # yellow
            (255,255,0),   # cyan
            (0,128,255),   # orange
            (255,0,255),   # magenta
        ]
        infs = overlay.get("inf_lines", [])
        for i, item in enumerate(infs):
            n, c = item
            color = line_colors[i % len(line_colors)]
            self.draw_inf_line(dbg_canvas, n, c, color, 2)

        # 3) 코너 점: 연두색(라임)으로
        if overlay.get("corners") is not None:
            for p in overlay["corners"].astype(int):
                cv2.circle(dbg_canvas, (p[0], p[1]), 5, (0,255,0), -1)  # light green / lime

    def get_grid_overlay_points(self, result):
        if result is None or result.grid_reference is None:
            return None
        H_inv = np.linalg.inv(result.grid_reference["H_metric"])
        m = self.board_margin
        T = np.array([[1,0,m],[0,1,m],[0,0,1]], dtype=np.float32)
        H_inv_margin = H_inv @ T

        pts_cm = np.array(result.grid_reference["cell_centers"], dtype=np.float32).reshape(-1,1,2)
        pts_px = cv2.perspectiveTransform(pts_cm, H_inv_margin).reshape(-1,2)

        mapped_lines = []
        for segs in result.grid_reference["grid_lines"].values():
            for p1_cm, p2_cm in segs:
                seg = np.array([[p1_cm, p2_cm]], dtype=np.float32)
                p1_px, p2_px = cv2.perspectiveTransform(seg, H_inv_margin)[0]
                mapped_lines.append((tuple(p1_px), tuple(p2_px)))
        return {"centers": pts_px, "lines": mapped_lines}

    def _get_board_pts(self, rect):
        pts = rect.reshape(4, 2).astype(np.float32)
        sum_pts = pts.sum(axis=1)
        diff_pts = np.diff(pts, axis=1)
        top_left = pts[np.argmin(sum_pts)]
        bottom_right = pts[np.argmax(sum_pts)]
        top_right = pts[np.argmin(diff_pts)]
        bottom_left = pts[np.argmax(diff_pts)]
        ordered = np.array([top_left, top_right, bottom_right, bottom_left])
        width_px = np.linalg.norm(top_right - top_left)
        height_px = np.linalg.norm(bottom_left - top_left)
        return ordered, width_px, height_px

    def _calculate_cm_per_px(self, warped_width_px, warped_height_px):
        cm_per_px_x = self.board_width_cm / max(warped_width_px, 1)
        cm_per_px_y = self.board_height_cm / max(warped_height_px, 1)
        return (cm_per_px_x, cm_per_px_y)

    def _warp_board(self, frame, corners, board_width_px, board_height_px):
        dst = np.array([[0, 0], [board_width_px - 1, 0],
                        [board_width_px - 1, board_height_px - 1], [0, board_height_px - 1]], dtype="float32")
        matrix = cv2.getPerspectiveTransform(corners, dst)
        warped = cv2.warpPerspective(frame, matrix, (int(board_width_px), int(board_height_px)))
        warped_resized = cv2.resize(warped, (frame.shape[1] // 2, frame.shape[1] // 2))
        warped_board_width_px = int(np.linalg.norm(dst[1] - dst[0]))
        warped_board_height_px = int(np.linalg.norm(dst[3] - dst[0]))
        return warped, warped_resized, matrix, warped_board_width_px, warped_board_height_px
    
    def _get_board_origin(self, top_left):
        return np.array([top_left[0], top_left[1], 0], dtype=np.float32)

    @property
    def is_locked(self):
        return self._locked
