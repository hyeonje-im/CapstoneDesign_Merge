import cv2
import numpy as np
import math
from typing import Optional, Tuple, List, Dict
from interface import slider_create, slider_value

from vision.camera import camera_open, Undistorter

from config import camera_cfg, board_width_cm, board_height_cm, grid_row, grid_col

def _order_corners_clockwise(pts: np.ndarray) -> np.ndarray:
    """Sort 4 points into TL, TR, BR, BL (clockwise) regardless of rotation.
    pts: (4,2) float32
    """
    c = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:,1] - c[1], pts[:,0] - c[0])
    order = np.argsort(angles)
    pts = pts[order]
    # ensure first is TL: minimal y; if tie, minimal x
    i = np.lexsort((pts[:,0], pts[:,1]))[0]
    pts = np.roll(pts, -np.where(np.arange(4)==i)[0][0], axis=0)
    # enforce clockwise: cross of (p1-p0) x (p2-p0) should be positive for CCW;
    # if CCW, reverse to make CW.
    if np.cross(pts[1]-pts[0], pts[2]-pts[0]) > 0:
        pts = np.array([pts[0], pts[3], pts[2], pts[1]], dtype=np.float32)
    return pts.astype(np.float32)


def _quad_is_reasonable(quad: np.ndarray,
                        w: int,
                        h: int,
                        angle_tol_deg: float = 12.0,
                        min_side_frac: float = 0.15,
                        max_cover_frac: float = 0.95) -> bool:
    """Geometric sanity checks for a quadrilateral to be our board rectangle.
    - angle near 90° within tolerance
    - side length >= min_side_frac * min(W,H)
    - quad mostly inside ROI (<= max_cover_frac of ROI size as a rough upper bound)
    """
    quad = _order_corners_clockwise(quad.copy())
    # side lengths
    sides = [np.linalg.norm(quad[(i+1)%4]-quad[i]) for i in range(4)]
    if min(sides) < min_side_frac * min(w, h):
        return False
    # angles
    def angle(a,b,c):
        v1 = a-b
        v2 = c-b
        cosang = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2)+1e-8)
        ang = np.degrees(np.arccos(np.clip(cosang, -1, 1)))
        return ang
    angs = [angle(quad[(i-1)%4], quad[i], quad[(i+1)%4]) for i in range(4)]
    if not all(abs(a-90.0) <= angle_tol_deg for a in angs):
        return False
    # roughly inside ROI
    if np.any(quad[:,0] < -1) or np.any(quad[:,1] < -1) or \
       np.any(quad[:,0] > w+1) or np.any(quad[:,1] > h+1):
        return False
    rect_area = _poly_area(quad)
    if rect_area > max_cover_frac * (w*h):
        return False
    return True


def _poly_area(pts: np.ndarray) -> float:
    x = pts[:,0]; y = pts[:,1]
    return 0.5*abs(np.dot(x, np.roll(y,-1)) - np.dot(y, np.roll(x,-1)))


def _detect_corners_gftt(gray: np.ndarray,
                         mask: Optional[np.ndarray],
                         max_corners: int,
                         quality_level: float,
                         min_dist_pix: float,
                         subpix_win: int = 5) -> np.ndarray:
    """Detect corners (GoodFeaturesToTrack) with optional mask,
    then refine to subpixel. Returns (N,2) float32.
    """
    corners = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=max_corners,
        qualityLevel=quality_level,
        minDistance=max(1, int(min_dist_pix)),
        mask=mask,
        blockSize=3,
        useHarrisDetector=False
    )
    if corners is None:
        return np.zeros((0,2), dtype=np.float32)
    corners = corners.reshape(-1,2).astype(np.float32)

    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    cv2.cornerSubPix(gray, corners, (subpix_win, subpix_win), (-1,-1), term)
    return corners


def _select_four_corners_by_pca_quadrant(corners: np.ndarray) -> Optional[np.ndarray]:
    """Project corners to PCA-basis and pick one farthest point per quadrant.
    Returns (4,2) or None.
    """
    if corners.shape[0] < 4:
        return None
    # PCA
    c = np.mean(corners, axis=0)
    X = corners - c
    cov = np.cov(X.T)
    eigvals, eigvecs = np.linalg.eig(cov)
    # sort eigenvectors by eigenvalues descending
    order = np.argsort(-eigvals)
    R = eigvecs[:, order]
    Y = X @ R  # rotate into PCA frame
    # assign quadrant by sign
    quads = [None, None, None, None]  # (+,+),(+,-),(-,-),(-,+)
    for i, y in enumerate(Y):
        idx = (1 if y[0] < 0 else 0) + (2 if y[1] < 0 else 0)
        # we want the farthest point in each quadrant
        d2 = float(np.dot(y, y))
        if quads[idx] is None or d2 > quads[idx][0]:
            quads[idx] = (d2, corners[i])
    if any(q is None for q in quads):
        return None
    pts = np.array([q[1] for q in quads], dtype=np.float32)
    return _order_corners_clockwise(pts)


# ---------------------------
# Line-based fallback path
# ---------------------------

def _to_rho_theta(p1: np.ndarray, p2: np.ndarray) -> Tuple[float, float]:
    """Convert segment endpoints to normal form line: n=(cos t, sin t), rho.
    We use theta as normal angle.
    """
    dx, dy = p2[0]-p1[0], p2[1]-p1[1]
    theta = math.atan2(dy, dx) + math.pi/2.0
    n = np.array([math.cos(theta), math.sin(theta)])
    rho = float(np.dot(n, p1))
    return rho, theta


def _line_intersection(rho1: float, th1: float, rho2: float, th2: float) -> Optional[np.ndarray]:
    # Solve [cos th1, sin th1; cos th2, sin th2] x = [rho1, rho2]
    A = np.array([[math.cos(th1), math.sin(th1)],
                  [math.cos(th2), math.sin(th2)]], dtype=np.float64)
    b = np.array([rho1, rho2], dtype=np.float64)
    det = A[0,0]*A[1,1] - A[0,1]*A[1,0]
    if abs(det) < 1e-8:
        return None
    x = np.linalg.solve(A, b)
    return x.astype(np.float32)


def _cluster_two_orientations(thetas: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Split line angles into 2 clusters using k-means on angle unit circle.
    Returns boolean masks for cluster A and B.
    """
    pts = np.stack([np.cos(thetas), np.sin(thetas)], axis=1).astype(np.float32)
    # kmeans
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1e-3)
    compactness, labels, centers = cv2.kmeans(pts, K=2, bestLabels=None, criteria=criteria,
                                             attempts=5, flags=cv2.KMEANS_PP_CENTERS)
    labels = labels.ravel()
    return labels==0, labels==1


def _line_fallback(gray: np.ndarray,
                   mask: Optional[np.ndarray],
                   min_line_len_pix: float,
                   angle_tol_deg: float,
                   want_debug: bool=False) -> Optional[np.ndarray]:
    """Detect long line pairs (two orientations) and form a rectangle from outermost pairs.
    Returns (4,2) or None.
    """
    h, w = gray.shape[:2]

    # Edges: prefer morphological gradient on mask for stability; else Canny on gray
    if mask is not None:
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        edges = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, k)
    else:
        edges = cv2.Canny(gray, 50, 150, L2gradient=True)

    lines_rho_theta = []

    # Try LSD first (more robust to gaps)
    lsd = None
    if hasattr(cv2, 'createLineSegmentDetector'):
        try:
            lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
        except Exception:
            lsd = None
    if lsd is not None:
        segs = lsd.detect(edges)[0]
        if segs is not None:
            for s in segs.reshape(-1,4):
                x1,y1,x2,y2 = map(float, s)
                L = math.hypot(x2-x1, y2-y1)
                if L >= min_line_len_pix:
                    rho, th = _to_rho_theta(np.array([x1,y1]), np.array([x2,y2]))
                    lines_rho_theta.append((rho, th))
    else:
        # Fallback to HoughLinesP
        hl = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=80,
                             minLineLength=int(min_line_len_pix), maxLineGap=int(min_line_len_pix*0.2))
        if hl is not None:
            for s in hl.reshape(-1,4):
                x1,y1,x2,y2 = map(float, s)
                rho, th = _to_rho_theta(np.array([x1,y1]), np.array([x2,y2]))
                lines_rho_theta.append((rho, th))

    if len(lines_rho_theta) < 4:
        return None

    rhos = np.array([rt[0] for rt in lines_rho_theta], dtype=np.float32)
    ths  = np.array([rt[1] for rt in lines_rho_theta], dtype=np.float32)

    # cluster into two orientations
    m0, m1 = _cluster_two_orientations(ths % np.pi)
    groups = []
    for m in [m0, m1]:
        if np.count_nonzero(m) < 2:
            return None
        r = rhos[m]
        t = ths[m]
        # pick outermost two lines by rho extrema
        i_min = np.argmin(r)
        i_max = np.argmax(r)
        groups.append([(r[i_min], t[i_min]), (r[i_max], t[i_max])])

    # form rectangle from 2x2 lines
    (rho_x1, th_x1), (rho_x2, th_x2) = groups[0]
    (rho_y1, th_y1), (rho_y2, th_y2) = groups[1]

    p00 = _line_intersection(rho_x1, th_x1, rho_y1, th_y1)
    p01 = _line_intersection(rho_x1, th_x1, rho_y2, th_y2)
    p11 = _line_intersection(rho_x2, th_x2, rho_y2, th_y2)
    p10 = _line_intersection(rho_x2, th_x2, rho_y1, th_y1)

    if any(p is None for p in [p00,p01,p11,p10]):
        return None

    quad = np.vstack([p00,p01,p11,p10]).astype(np.float32)
    return quad


# ---------------------------
# Public API
# ---------------------------

def detect_board_corners(
    roi_bgr: np.ndarray,
    bin_mask: Optional[np.ndarray] = None,
    *,
    max_corners: int = 200,
    quality_level: float = 0.01,
    min_corner_dist_frac: float = 0.05,  # distance as % of min(ROI W,H)
    min_line_len_frac: float = 0.15,     # per user: long lines >= 15% of ROI
    angle_tol_deg: float = 12.0,
    max_cover_frac: float = 0.95,
    want_debug: bool = False,
) -> Dict[str, object]:
    """
    Corner-first, line-fallback rectangle detection.

    Inputs:
      - roi_bgr: ROI image (BGR or grayscale). No rescaling done.
      - bin_mask: optional binary mask that emphasizes the tape (1/255). Strongly recommended.

    Outputs dict:
      { 'ok': bool,
        'method': 'corners'|'lines'|'none',
        'corners': (4,2) float32 in ROI coords (TL,TR,BR,BL) if ok,
        'debug': dict of optional debug images }
    """
    if roi_bgr.ndim == 3:
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi_bgr.copy()

    h, w = gray.shape[:2]
    min_side_pix = float(min(w, h))
    min_dist_pix = max(2.0, min_corner_dist_frac * min_side_pix)
    min_line_len_pix = max(8.0, min_line_len_frac * min_side_pix)

    # --- Corner-first path ---
    # Prefer using the binary mask as a mask to suppress background
    mask_for_corners = None
    if bin_mask is not None:
        # ensure mask is single-channel uint8 {0,255}
        m = bin_mask
        if m.ndim == 3:
            m = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
        _, m = cv2.threshold(m, 0, 255, cv2.THRESH_BINARY)
        mask_for_corners = m

    corners = _detect_corners_gftt(
        gray, mask_for_corners, max_corners=max_corners,
        quality_level=quality_level, min_dist_pix=min_dist_pix, subpix_win=5)

    # Try PCA-quadrant selection
    quad = None
    if corners.shape[0] >= 4:
        quad = _select_four_corners_by_pca_quadrant(corners)
        if quad is not None and _quad_is_reasonable(quad, w, h, angle_tol_deg, min_line_len_frac, max_cover_frac):
            quad = _order_corners_clockwise(quad)
            out = {
                'ok': True,
                'method': 'corners',
                'corners': quad,
                'debug': {}
            }
            if want_debug:
                dbg = roi_bgr.copy() if roi_bgr.ndim==3 else cv2.cvtColor(roi_bgr, cv2.COLOR_GRAY2BGR)
                for p in corners:
                    cv2.circle(dbg, tuple(np.round(p).astype(int)), 2, (0,255,0), -1)
                for i,p in enumerate(quad):
                    cv2.circle(dbg, tuple(np.round(p).astype(int)), 6, (0,0,255), 2)
                    cv2.putText(dbg, str(i), tuple(np.round(p+np.array([4,-4])).astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
                out['debug']['overlay'] = dbg
            return out

    # --- Fallback: line-based reconstruction (long lines only) ---
    quad2 = _line_fallback(gray, bin_mask, min_line_len_pix, angle_tol_deg, want_debug)
    if quad2 is not None and _quad_is_reasonable(quad2, w, h, angle_tol_deg, min_line_len_frac, max_cover_frac):
        quad2 = _order_corners_clockwise(quad2)
        out = {
            'ok': True,
            'method': 'lines',
            'corners': quad2,
            'debug': {}
        }
        if want_debug:
            dbg = roi_bgr.copy() if roi_bgr.ndim==3 else cv2.cvtColor(roi_bgr, cv2.COLOR_GRAY2BGR)
            for i,p in enumerate(quad2):
                cv2.circle(dbg, tuple(np.round(p).astype(int)), 6, (255,0,0), 2)
                cv2.putText(dbg, str(i), tuple(np.round(p+np.array([4,-4])).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1, cv2.LINE_AA)
            out['debug']['overlay'] = dbg
        return out

    return { 'ok': False, 'method': 'none', 'corners': None, 'debug': {} }

# --- 전역 변수: ROI 선택 상태 ---
roi_tl = None       # Top-left 좌표
roi_br = None       # Bottom-right 좌표
selecting_roi = False

# 마우스 콜백: 좌클릭 두 번으로 ROI 설정
def mouse_callback(event, x, y, flags, param):
    global roi_tl, roi_br, selecting_roi
    if event == cv2.EVENT_LBUTTONDOWN:
        if not selecting_roi:
            roi_tl = (x, y)
            selecting_roi = True
            print(f"[ROI] Top-left set to: {roi_tl}")
        else:
            roi_br = (x, y)
            selecting_roi = False
            print(f"[ROI] Bottom-right set to: {roi_br}")


def main():
    global roi_tl, roi_br, selecting_roi
    # 1) 카메라 열기 및 왜곡 보정기 설정
    cap, fps = camera_open()  # 기본 카메라 검색 및 설정 fileciteturn0file2
    undistorter = Undistorter(
        camera_cfg['type'], camera_cfg['matrix'], camera_cfg['dist'], camera_cfg['size']
    )

    # 2) 보드 탐지기 및 필터 초기화
    board_detector = BoardDetector(
        board_width_cm, board_height_cm, grid_row, grid_col, 0.0
    )
    roi_filter = ROIFilter()

    # 3) 윈도우 및 콜백 등록
    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", mouse_callback)

    # 탐지 파라미터: (min_aspect_ratio, max_aspect_ratio)
    slider_create()
    bright, min_ar, max_ar, cos_value, extent_value, soild_value = slider_value()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 프레임 획득 실패")
            break
        
        bright, min_ar, max_ar, cos_value, extent_value, soild_value = slider_value()

        # 왜곡 보정 및 회색조
        undistorted, _ = undistorter.undistort(frame)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        display = undistorted.copy()

        # 4) ROI가 설정되었으면 그 영역에서 전처리 및 보드 탐지
        if roi_tl and roi_br:
            x1, y1 = roi_tl
            x2, y2 = roi_br
            x1, x2 = sorted([x1, x2])
            y1, y2 = sorted([y1, y2])

            if x2 > x1 and y2 > y1:
                roi = gray[y1:y2, x1:x2]
                if roi.size == 0:
                    print("[ERROR] Empty ROI region")
                else:
                    # 이진화 전처리 및 필터 결과 보기
                    filtered = roi_filter.binarize(roi, manual_thresh=int(bright))
                    cv2.imshow("Filtered ROI", filtered)

                    # === 새 보드 검출: 코너 주도, 직선(≥15%) 폴백, findContours 미사용 ===
                    res = detect_board_corners(
                        roi_bgr=undistorted[y1:y2, x1:x2],   # BGR ROI
                        bin_mask=filtered,                    # 기존 이진화 마스크를 그대로 활용
                        min_line_len_frac=0.15,               # 요청대로 "긴 직선" 기준 = ROI의 15%
                        angle_tol_deg=12.0,                   # 직각 허용 오차(튜닝 가능)
                        max_cover_frac=0.95,                  # ROI를 거의 꽉 채우면 거르는 상한(원하면 조정/해제)
                        want_debug=True
                    )

                    if res['ok']:
                        quad = res['corners'].astype(np.float32)   # (TL, TR, BR, BL), ROI 좌표
                        quad_full = quad + np.array([[x1, y1]], dtype=np.float32)  # 프레임 좌표로 오프셋

                        # 폴리라인 시각화
                        cv2.polylines(display, [quad_full.astype(np.int32)], True, (0,255,0), 2)

                        # 퍼스펙티브 워프 (가로/세로 픽셀 크기 보수적으로 계산)
                        TL, TR, BR, BL = quad_full
                        w_px = int(max(np.linalg.norm(TR - TL), np.linalg.norm(BR - BL)))
                        h_px = int(max(np.linalg.norm(BL - TL), np.linalg.norm(BR - TR)))
                        w_px = max(w_px, 10); h_px = max(h_px, 10)

                        dst = np.array([[0,0], [w_px-1,0], [w_px-1,h_px-1], [0,h_px-1]], np.float32)
                        H = cv2.getPerspectiveTransform(quad_full, dst)
                        warped = cv2.warpPerspective(gray, H, (w_px, h_px))
                        cv2.imshow("Warped Board", warped)
                    else:
                        pass
            else:
                print(f"[ERROR] Invalid ROI coordinates: tl={roi_tl}, br={roi_br}")

        # ROI 영역 시각화
        if roi_tl:
            cv2.circle(display, roi_tl, 5, (255,0,0), -1)
        if roi_tl and roi_br:
            cv2.rectangle(display, (x1, y1), (x2, y2), (0,0,255), 2)

        cv2.imshow("Frame", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            roi_tl, roi_br = None, None
            selecting_roi = False
            print("[INFO] ROI reset")

    cap.release()
    cv2.destroyAllWindows()

class ROIFilter:
    def __init__(
        self,
        # --- Binarization pipeline ---
        clahe_clip_bin: float = 2.0,
        clahe_tile_bin: Tuple[int,int] = (8,8),
        adaptive_block: int = 21,
        adaptive_C: int = 5,
        # --- Enhancement pipeline ---
        scale_enh: int = 2,
        unsharp_ksize: Tuple[int,int] = (9,9),
        unsharp_sigma: float = 10,
        clahe_clip_enh: float = 3.0,
        clahe_tile_enh: Tuple[int,int] = (8,8),
        bilateral_d: int = 9,
        bilateral_sigma_color: float = 75,
        bilateral_sigma_space: float = 75,
    ):
        # Binarization params
        self.clahe_clip_bin = clahe_clip_bin
        self.clahe_tile_bin = clahe_tile_bin
        self.adaptive_block = adaptive_block
        self.adaptive_C = adaptive_C

        # Enhancement params
        self.scale_enh = scale_enh
        self.unsharp_ksize = unsharp_ksize
        self.unsharp_sigma = unsharp_sigma
        self.clahe_clip_enh = clahe_clip_enh
        self.clahe_tile_enh = clahe_tile_enh
        self.bilateral_d = bilateral_d
        self.bilateral_sigma_color = bilateral_sigma_color
        self.bilateral_sigma_space = bilateral_sigma_space

    def binarize(self, img: np.ndarray, manual_thresh: Optional[int] = None) -> np.ndarray:
        """
        1) Grayscale 변환  
        2) CLAHE (명암 대비 향상)  
        3) Adaptive Threshold 이진화  
        4) Median Blur (소금·후추 노이즈 제거) 
        5) 색 반전
        """
        # 1) Gray
        if img.ndim == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        # # 2) CLAHE
        # clahe = cv2.createCLAHE(
        #     clipLimit=self.clahe_clip_bin,
        #     tileGridSize=self.clahe_tile_bin
        # )
        # gray = clahe.apply(gray)  # :contentReference[oaicite:6]{index=6}
        gray = cv2.GaussianBlur(gray, (5,5), 0)

        # # 3) Adaptive Threshold
        # bsize = max(3, self.adaptive_block)
        # if bsize % 2 == 0: bsize += 1
        # thresh = cv2.adaptiveThreshold(
        #     gray, 255,
        #     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        #     cv2.THRESH_BINARY,
        #     bsize,
        #     self.adaptive_C
        # )  # :contentReference[oaicite:7]{index=7}
        t = int(max(0, min(255, manual_thresh)))
        _, thresh = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)

        # 4) Noise removal
        median = cv2.medianBlur(thresh, 3)

        # 5) Invert colors
        inverted = cv2.bitwise_not(median)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        closed = cv2.morphologyEx(inverted, cv2.MORPH_CLOSE, kernel)

        return closed

class BoardDetector:
    def __init__(self, board_width_cm: float, board_height_cm: float, grid_width: int, grid_height: int, board_margin: float):
        self.board_width_cm = board_width_cm
        self.board_height_cm = board_height_cm
    
    def detect(self, roi_gray, detect_params):
        rect = self._detect_board(roi_gray, detect_params)
        return rect
    
    def _detect_board(self, gray, detect_params):
        """
        gray: 이미 이진화된 ROI
        detect_params: (min_aspect_ratio, max_aspect_ratio)
        """
        bright, min_ar, max_ar, cos_value , extent_value, soild_value = detect_params
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        h_img, w_img = gray.shape

        for cnt in contours:
            # 1) 4점 근사 + 면적·종횡비 필터 (기존)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            if len(approx) != 4:
                continue
            area = cv2.contourArea(approx)
            if area < 500:
                continue
            x,y,w,h = cv2.boundingRect(approx)
            ar = float(w)/h
            if not (min_ar < ar < max_ar):
                continue

            # 2) 각도가 직각에 가까운지 (벡터 내적)
            pts = approx.reshape(4,2)
            def angle(p0, p1, p2):
                v0 = p0 - p1
                v2 = p2 - p1
                # cos θ = (v0·v2)/(||v0||·||v2||), 직각이면 0에 가까워짐
                return abs(np.dot(v0, v2)) / (np.linalg.norm(v0)*np.linalg.norm(v2) + 1e-6)
            angles_ok = all(angle(pts[i], pts[(i+1)%4], pts[(i+2)%4]) < cos_value for i in range(4))
            if not angles_ok:
                continue

            # 3) extent / solidity 필터
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            extent   = area / float(w*h)
            solidity = area / float(hull_area + 1e-6)
            if extent < extent_value or solidity < soild_value:
                continue

            candidates.append((approx, area))

        # 최대 면적 후보 리턴
        if not candidates:
            return None
        best, _ = max(candidates, key=lambda x:x[1])
        return best
        
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



if __name__ == "__main__":
    main()