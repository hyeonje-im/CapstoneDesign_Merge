import cv2
import numpy as np
from dataclasses import dataclass

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
    - 기존 BoardDetector를 확장: 자동 ROI 이진화 + 파라미터 스윕 + 신뢰도 누적을 내부에 통합.
    - 프레임당 소량 후보만 시도하는 증분형(auto step) 구조로, 메인 루프를 블로킹하지 않는다.
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

    # ======== 내부 중첩 클래스들 ========
    class _ReliabilityAccumulator:
        def __init__(self, window=20, threshold=18):
            self.window = int(window)
            self.threshold = int(threshold)
            self.buf = []
            self.sum_img = None
            self.shape = None

        def reset(self):
            self.buf.clear()
            self.sum_img = None
            self.shape = None

        def set_shape(self, shape):
            if self.shape != shape:
                self.reset()
                self.shape = shape
                self.sum_img = np.zeros(shape, dtype=np.uint16)

        def push(self, bin_img: np.ndarray):
            if bin_img is None or bin_img.size == 0:
                return
            h, w = bin_img.shape[:2]
            if self.shape != (h, w):
                self.set_shape((h, w))
            mask01 = (bin_img > 0).astype(np.uint8)
            self.buf.append(mask01)
            self.sum_img += mask01
            if len(self.buf) > self.window:
                oldest = self.buf.pop(0)
                self.sum_img -= oldest

        def enough(self):
            return len(self.buf) >= min(self.window, 3)

        def get_mask(self):
            if self.sum_img is None:
                return None
            k = min(self.threshold, self.window)
            return (self.sum_img >= k).astype(np.uint8) * 255

    class _AutoROIFilter:
        def __init__(self, parent:'BoardDetector'):
            self._P = parent
            self.auto_params = {
                "offset": 0, "clip": 2.0, "block": 21, "C": 10,
                "close": (5,5), "open": (3,3), "invert": True
            }

        def autotune(self, roi_gray: np.ndarray):
            H, W = roi_gray.shape[:2]
            S = min(H, W)

            # 스케일 기반 초기값
            block   = self._P._odd(self._P._clamp(S/20, 15, 51))
            close_k = self._P._ksize(self._P._clamp(S/100, 3, 7))
            open_k  = (3,3)

            mean = float(np.mean(roi_gray))
            std  = float(np.std(roi_gray))
            p98  = float(np.percentile(roi_gray, 98))

            # 조명 불균일
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
            clip = self._P._clamp(1.0 + (20.0 / (std + 1e-6)), 1.2, 3.0)
            if glare: clip = min(clip, 1.8)
            if pattern_noise: clip = max(1.2, clip * 0.9)

            # Brightness offset
            offset = 0
            if mean < 90:  offset = +15
            elif mean > 165: offset = -15

            # C
            C = 10
            if uneven: C += 4
            if pattern_noise: C += 4
            if glare: C -= 4
            C = int(self._P._clamp(C, 3, 20))

            self.auto_params = {
                "offset": int(offset),
                "clip": float(round(clip,2)),
                "block": int(block),
                "C": int(C),
                "close": close_k,
                "open": open_k,
                "invert": True
            }

        def binarize_with(self, roi_bgr_or_gray, ap: dict):
            if roi_bgr_or_gray.ndim == 3:
                gray = cv2.cvtColor(roi_bgr_or_gray, cv2.COLOR_BGR2GRAY)
            else:
                gray = roi_bgr_or_gray.copy()

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

    class _AutoParamSearch:
        """간소 스윕: base에서 (block,C) / (clip,offset) / invert False 후보 생성."""
        def __init__(self, parent:'BoardDetector'):
            self._P = parent
            self.candidates = []
            self.idx = 0
            self.active = False

        def _mk_ap(self, base, **over):
            ap = dict(base); ap.update(over)
            ap["block"] = self._P._odd(self._P._clamp(ap["block"], 15, 51))
            ap["C"]     = int(self._P._clamp(int(ap["C"]), 3, 20))
            ap["clip"]  = float(self._P._clamp(float(ap["clip"]), 1.2, 3.0))
            ap["offset"]= int(self._P._clamp(int(ap["offset"]), -40, 40))
            ap["close"] = self._P._ksize(self._P._clamp(ap.get("close",(5,5))[0], 3, 7))
            ap["open"]  = self._P._ksize(self._P._clamp(ap.get("open",(3,3))[0], 3, 5))
            ap["invert"]= bool(ap.get("invert", True))
            return ap

        def prepare(self, base_ap: dict):
            cand = []
            # 1) block & C
            for db in [0, +8, -8, +12, -12]:
                for dC in [0, +4, -4, +8, -8]:
                    cand.append(self._mk_ap(base_ap, block=base_ap["block"]+db, C=base_ap["C"]+dC))
            # 2) clip / offset
            for dclip in [0.0, +0.5, -0.5]:
                for doff in [0, +15, -15]:
                    cand.append(self._mk_ap(base_ap, clip=base_ap["clip"]+dclip, offset=base_ap["offset"]+doff))
            # 3) invert flip
            cand.append(self._mk_ap(base_ap, invert=False))

            # de-dup
            seen, uniq = set(), []
            for ap in cand:
                key = (ap["block"], ap["C"], ap["clip"], ap["offset"], ap["invert"])
                if key not in seen:
                    uniq.append(ap); seen.add(key)

            self.candidates = uniq
            self.idx = 0
            self.active = True

        def next_batch(self, n=3):
            if not self.active or self.idx >= len(self.candidates):
                return []
            j = min(self.idx + n, len(self.candidates))
            return self.candidates[self.idx:j]

        def advance(self, k:int):
            self.idx = min(self.idx + k, len(self.candidates))
            if self.idx >= len(self.candidates):
                self.active = False

    # ======== 기존 BoardDetector ========
    def __init__(self, board_width_cm: float, board_height_cm: float, grid_width: int, grid_height: int, board_margin: float):
        self.board_width_cm = board_width_cm
        self.board_height_cm = board_height_cm
        self.board_margin = board_margin
        self._result = None
        self._locked = False
        self.grid_width = grid_width
        self.grid_height = grid_height

        self.auto_enabled = True            # 항상 ON (light 모드)
        self.roi_rect: tuple[int,int,int,int] | None = None
        self._reli = self._ReliabilityAccumulator(window=20, threshold=18)
        self._autof = self._AutoROIFilter(self)
        self._search = self._AutoParamSearch(self)
        self._prev_roi_rect = None
        self._max_tries_per_frame = 3       # 비차단 보장: 프레임당 후보 제한

    # 외부에서 ROI 설정
    def set_roi(self, rect_xyxy: tuple[int,int,int,int] | None):
        """(x1,y1,x2,y2); None이면 자동 탐색 비활성화"""
        self.roi_rect = rect_xyxy
        self._prev_roi_rect = None
        self._reli.reset()
        self._search.active = False
        if self._locked:
            self._locked = False
            self._result = None

    def detect(self, roi_gray, detect_params):
        return self._detect_board_from_binary(roi_gray, detect_params)

    def process(self, frame_gray, detect_params, rect_override=None) -> BoardDetectionResult | None:
        if self._locked and self._result is not None:
            H = self._result.perspective_matrix
            w_px = int(self._result.width_px)
            h_px = int(self._result.height_px)
            warped = cv2.warpPerspective(frame_gray, H, (w_px, h_px))
            warped_resized = cv2.resize(warped, (frame_gray.shape[1]//2, frame_gray.shape[1]//2))
            self._result.warped = warped
            self._result.warped_resized = warped_resized
            return self._result

        # 1) 우선순위: 외부 rect_override
        if rect_override is not None:
            rect = rect_override

        # 2) 자동 모드 + ROI 세팅됨 → 증분 자동 탐색 (프레임당 소량만 시도)
        elif self.auto_enabled and self.roi_rect is not None:
            rect = self._detect_board_auto_step(frame_gray, detect_params)

        # 3) 그 외(ROI 없음) → 바이너리 기반 기존 검출기에 위임(필요 시 상위에서 bin 처리)
        else:
            rect = self._detect_board_from_binary(frame_gray, detect_params)

        if rect is not None:
            corners, width_px, height_px = self._get_board_pts(rect)
            origin = self._get_board_origin(corners[0])
            warped, warped_resized, perspective_matrix, warped_width_px, warped_height_px = self._warp_board(frame_gray, corners, width_px, height_px)
            cm_per_px = self._calculate_cm_per_px(warped_width_px, warped_height_px)
            
            self._result = BoardDetectionResult(
                corners=corners,
                origin=origin,
                width_px=width_px,
                height_px=height_px,
                cm_per_px=cm_per_px,
                perspective_matrix=perspective_matrix,
                warped=warped,
                warped_resized=warped_resized,
            )
            return self._result
        else:
            return self._result

    # ---- 증분 자동 탐색(step) ----
    def _detect_board_auto_step(self, frame_gray, detect_params):
        if self.roi_rect is None:
            return None
        x1, y1, x2, y2 = self.roi_rect
        x1, x2 = sorted([x1, x2]); y1, y2 = sorted([y1, y2])
        if x2 <= x1 or y2 <= y1:
            return None

        roi = frame_gray[y1:y2, x1:x2]
        roi_rect_now = (x1, y1, x2, y2)

        # ROI 변경 시 상태 초기화
        if self._prev_roi_rect != roi_rect_now:
            self._reli.set_shape(roi.shape)
            self._prev_roi_rect = roi_rect_now
            self._search.active = False

        # 베이스 파라미터 추정
        self._autof.autotune(roi)
        base_ap = dict(self._autof.auto_params)

        # 필요한 경우 후보 스윕 준비
        if (self._result is None) and not self._search.active:
            self._search.prepare(base_ap)

        # 프레임당 제한된 후보만 시도(비차단)
        tried = 0
        found_rect = None

        if self._search.active:
            for ap in self._search.next_batch(self._max_tries_per_frame):
                filt = self._autof.binarize_with(roi, ap)
                self._reli.push(filt)
                rel_mask = self._reli.get_mask()
                src = rel_mask if self._reli.enough() else filt
                rect_roi = self._detect_board_from_binary(src, detect_params)
                tried += 1
                if rect_roi is not None:
                    found_rect = rect_roi
                    base_ap = ap  # 채택
                    break
            self._search.advance(tried)
        else:
            # 스윕 비활성: 베이스 한 번만
            filt = self._autof.binarize_with(roi, base_ap)
            self._reli.push(filt)
            rel_mask = self._reli.get_mask()
            src = rel_mask if self._reli.enough() else filt
            found_rect = self._detect_board_from_binary(src, detect_params)

        if found_rect is not None:
            rect_full = found_rect.copy()
            rect_full[:,0,0] += x1
            rect_full[:,0,1] += y1
            # 채택한 파라미터 보존(다음 프레임에 베이스로 사용)
            self._autof.auto_params = dict(base_ap)
            self._search.active = False
            return rect_full
        else:
            return None

    # ---- 기존 강화 필터 기반 바이너리 컨투어 검출 ----
    def _detect_board_from_binary(self, gray_bin, detect_params):
        """
        gray_bin: 이미 '이진화된' ROI/이미지
        detect_params: (brightness, min_ar, max_ar, cos, extent, solidity) 혹은
                       (min_ar, max_ar, cos, extent, solidity) 형태 모두 허용
        """
        # detect_params 호환성 처리
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

    # ======== 나머지(원본 그대로) ========
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
        # (선택) warp된 뷰에 스케일 텍스트 추가는 원본과 동일하게 유지 가능

    def reset(self):
        self._locked = False
        self._result = None
        self._reli.reset()
        self._search.active = False

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
