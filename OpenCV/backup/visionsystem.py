import cv2
import numpy as np
import math
from typing import Optional, Tuple, Dict

from vision.apriltag import AprilTagDetector
from config import board_width_cm, board_height_cm, grid_row, grid_col, cell_size, cell_size_cm, tag_size, CORRECTION_COEF, NORTH_TAG_ID, board_margin
from vision.board import BoardDetectionResult, BoardDetector

class VisionSystem:
    def __init__(self, undistorter, visualize=True):
        
        self.correction_coef_getter = lambda: CORRECTION_COEF
        self.tags = AprilTagDetector(self.correction_coef_getter)
        self.visualize = visualize
        self.grid_row = grid_row
        self.grid_col = grid_col
        self.undistorter = undistorter
        self.last_valid_result = None
        self.board = BoardDetector(board_width_cm, board_height_cm, grid_row, grid_col, board_margin)
        self.board_result: BoardDetectionResult | None = None
        self.frame_count = 0
        self.roi_filter = ROIFilter()
        
        # 수동 ROI 설정
        self.manual_roi_top_left = None
        self.manual_roi_bottom_right = None
        self.user_selecting_roi = False

        # 화면 해상도 설정
        self.frame_shape = None
        self.target_display_size = (960, 540)
        self.display_size = None

    # =====수동 ROI 선택===== 
    
    def start_roi_selection(self):
        self.manual_roi_top_left = None
        self.manual_roi_bottom_right = None
        self.user_selecting_roi = True
        print("[ROI] Selection mode enabled. Click top-left and bottom-right.")

    def mouse_callback(self, event, x, y, flags, param):
        if not (self.user_selecting_roi and event == cv2.EVENT_LBUTTONDOWN and self.display_size and self.frame_shape):
            return
        x0, y0 = (x, y)
        if self.display_size:
            x0, y0 = self.to_original_coords(x, y)

        if self.manual_roi_top_left is None:
            self.manual_roi_top_left = (x0, y0)
            print(f"[ROI] Top-left set to: {self.manual_roi_top_left}")
        else:
            self.manual_roi_bottom_right = (x0, y0)
            print(f"[ROI] Bottom-right set to: {self.manual_roi_bottom_right}")
            self.user_selecting_roi = False
    
    # ===== 수동 ROI 선택 끝 =====
        
    # ===== 프레임 처리 =====
    def process_frame(self, raw_frame, detect_params=None, scale=2):
        
        # 1) 기본 프레임 전처리 및 회색조
        frame, new_camera_matrix = self.undistorter.undistort(raw_frame)
        self.frame_shape = frame.shape[:2]
        self.frame_count += 1
        raw_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        rect_override = None

        # 2) ROI 영역 설정
        board_tag = self.tags.get_board_tag()

        roi_frame, (roi_x_min, roi_y_min, roi_x_max, roi_y_max) = self._compute_roi(
            raw_gray=raw_gray,
            frame_shape=frame.shape,
            board=self.board,
            board_result=self.board_result,
            manual_tl=self.manual_roi_top_left,
            manual_br=self.manual_roi_bottom_right,
            board_tag=board_tag,
        )

        # 3) 태그 탐지
        # ROI에 태그 탐지 필터링
        tag_filtered_frame = self.roi_filter.enhance(roi_frame)

        # 태그 필터 프레임에서 태그 탐지
        raw_tags = self.tags.detect(tag_filtered_frame)
        tags = self.correct_tag_coordinates(raw_tags, (roi_x_min, roi_y_min, roi_x_max - roi_x_min, roi_y_max - roi_y_min), scale)
        self.tags.update(tags, self.frame_count, new_camera_matrix, self.frame_shape)
            
        # 4) ROI 내에서 보드 검출
        board_filtered_frame = self.roi_filter.binarize(roi_frame)
        board_corners = self.board.detect(board_filtered_frame, detect_params)
        if board_corners is not None:
            # ROI 성공 → override 좌표로 풀프레임 검출
            rect_full = board_corners.copy()
            rect_full[:,0,0] += roi_x_min
            rect_full[:,0,1] += roi_y_min
            self.board.process(raw_gray, detect_params, rect_override=rect_full)
        else:
            # ROI 실패 → 순수 풀프레임 검출
            self.board.process(raw_gray, detect_params, rect_override=None)
        self.board_result = self.board.get_result()
                
        # 5) 보드를 이용해 태그 처리 및 업데이트
        if self.board_result:
            self.tags.process(self.board_result.origin, self.board_result.cm_per_px)

        tag_info = self.tags.get_raw_tags()
        if self.board_result:
            self.transform_coordinates(tag_info)
            self.compute_tag_orientation(tag_info)

        # 6) 시각화 처리
        if self.visualize:
            cv2.rectangle(frame, (roi_x_min, roi_y_min), (roi_x_max, roi_y_max), (0, 0, 255), 2)
            self.board.draw(frame, self.board_result)
            self.tags.draw(frame)
            self.draw_tag_overlay(frame, tag_info)

        if self.manual_roi_top_left and self.manual_roi_bottom_right:
            roi_display = roi_frame.copy()
            roi_display = cv2.resize(roi_display, (min(roi_display.shape[1]*2, 800), min(roi_display.shape[0]*2, 800)))
            cv2.imshow("ROI_Display", roi_display)

        # 7) 기타
        disp_w, disp_h = self.target_display_size
        display_frame = cv2.resize(frame, (disp_w, disp_h))
        self.display_size = (disp_w, disp_h)
        
        # 보드가 lock 상태이고 결과가 있을 때
        if self.board.is_locked and self.board_result is not None:
            # 화면 크기로 리사이즈된 warp 이미지
            cv2.imshow("Warped Board Preview", self.board_result.warped_resized)


        return {
            "frame": display_frame,
            "tag_info": tag_info,
        }

    def correct_tag_coordinates(self,
                                tags: list,
                                roi_range: tuple[int, int, int, int],
                                scale: float = 2) -> list:
        x0, y0, _, _ = roi_range
        for tag in tags:
            tag.center = tag.center / scale + np.array([x0, y0])
            tag.corners = tag.corners / scale + np.array([x0, y0])
        return tags

    # 외부 이벤트 처리용 API들
    def lock_board(self):
        self.board.lock()

    def reset_board(self):
        self.board.reset()
        self.last_valid_result = None

    def toggle_visualization(self):
        self.visualize = not self.visualize

    def get_raw_tag_info(self):
        return self.tags.get_raw_tags()

    def get_robot_tags(self):
        return self.tags.get_robot_tags()
    
    def get_fps(self):
        return self.fps

    # def correct_tag_position_polar(self, X, Y, Cx, Cy, coef=None):
    #     if coef is None:
    #         coef = self.correction_coef_getter()
    #     dx = X - Cx
    #     dy = Y - Cy
    #     r = math.sqrt(dx**2 + dy**2)
    #     theta = math.atan2(dy, dx)
    #     r_prime = r * coef
    #     X_prime = Cx + r_prime * math.cos(theta)
    #     Y_prime = Cy + r_prime * math.sin(theta)
    #     return X_prime, Y_prime

    def transform_coordinates(self, tag_infos: dict[int, dict]):
        if not (self.board_result and self.board.is_locked and self.board_result.grid_reference):
            return
        ref = self.board_result.grid_reference
        H = ref["H_metric"]
        centers = ref["cell_centers"]

        for tag_id, data in tag_infos.items():
            cx_px, cy_px = data.get("center", (None, None))
            if cx_px is None or cy_px is None:
                continue
            pts = np.array([[[cx_px, cy_px]]], dtype=np.float32)
            X_cm, Y_cm = cv2.perspectiveTransform(pts, H)[0][0]
            col = int(X_cm // cell_size_cm)
            row = int(Y_cm // cell_size_cm)
            col = max(0, min(self.grid_col - 1, col))
            row = max(0, min(self.grid_row - 1, row))
            data["grid_position"] = (row, col)
            idx = row * self.grid_col + col
            if idx < len(centers):
                cx_cm, cy_cm = centers[idx]
                data["dist_cm"] = math.hypot(X_cm - cx_cm, Y_cm - cy_cm)
            else:
                data["dist_cm"] = None

    def compute_tag_orientation(self, tag_infos: dict[int, dict]):
        """
        H_metric 좌표계에서 태그 정면 벡터와
        그리드→태그 벡터의 각도를 계산해 tag_infos에 저장.
        """
        if not (self.board_result and self.board.is_locked and self.board_result.grid_reference):
            return

        H = self.board_result.grid_reference["H_metric"]
        grid_centers = self.board_result.grid_reference["cell_centers"]

        for tag_id, data in tag_infos.items():
            corners = data.get("corners")
            if corners is None or "grid_position" not in data:
                continue

            # 1) 태그 정면 벡터 (top-left→top-right) 를 cm 좌표계로 투영
            pts = np.array([corners[0], corners[1]], dtype=np.float32).reshape(-1,1,2)
            pt_cm = cv2.perspectiveTransform(pts, H).reshape(2,2)
            front_vec = pt_cm[0] - pt_cm[1]

            # 2) 태그 중심과 그리드 중심을 cm 좌표계로 투영
            if "center" not in data:
                continue  # 보정값 없으면 스킵
            X_cm, Y_cm = data["center"]
            tag_cm = np.array([X_cm, Y_cm])
            row, col = data["grid_position"]
            grid_cm = np.array(grid_centers[row * self.grid_col + col])
            dir_vec = tag_cm - grid_cm

            # 3) 각도 계산 (atan2: rad → deg)
            yaw_front = math.degrees(math.atan2(front_vec[1],  front_vec[0]))
            yaw_dir   = math.degrees(math.atan2(dir_vec[1],    dir_vec[0]))

            # 4) 상대 각도 저장
            data["yaw_front_deg"]    = yaw_front # 태그 정면 기준 yaw
            data["yaw_to_grid_deg"]  = yaw_dir   # 태그 중심→그리드 중심 기준 yaw  
            data["relative_angle_deg"] = ((yaw_dir - yaw_front + 180) % 360) - 180 # 정면과 그리드 중심 기준 yaw 각도 차이

            # 5) 4방위 기준 yaw 계산    
            yaw_deg = (yaw_front + 360) % 360
            direction_names   = ["N", "W", "S", "E"]
            direction_angles  = [90, 0, 270, 180]
            diffs = [abs(((yaw_deg - a + 180) % 360) - 180) for a in direction_angles]
            min_idx = diffs.index(min(diffs))
            base_dir   = direction_names[min_idx]
            base_angle = direction_angles[min_idx]
            delta = ((yaw_deg - base_angle + 180) % 360) - 180
            
            data["heading_base_dir"]    = base_dir   # 4방위 이름
            data["heading_base_angle"]  = base_angle # 4방위 기준 각도
            data["heading_offset_deg"]  = delta      # 태그 정면 기준 yaw에서 4방위까지 오차 각도

        north = tag_infos.get(NORTH_TAG_ID)
        if north is not None and north.get('status') == 'On':
            north_yaw = north.get('yaw')
            north_front = north.get('yaw_front_deg')

            for tag_id, data in tag_infos.items():
                if tag_id == NORTH_TAG_ID or data.get('status') != 'On':
                    continue

                # (a) 라디안 기반
                cur_yaw = data.get('yaw')
                if cur_yaw is not None and north_yaw is not None:
                    delta = ((cur_yaw - north_yaw + math.pi) % (2 * math.pi)) - math.pi
                    data['yaw_to_north_rad'] = delta # 북쪽 기준 각도(라디안)
                    data['yaw_to_north_deg'] = math.degrees(delta) # 북쪽 기준 각도(도)

                # (b) 정면(yaw_front_deg) 기반
                cur_front = data.get('yaw_front_deg')
                if cur_front is not None and north_front is not None:
                    delta_front_deg = ((cur_front - north_front + 180) % 360) - 180
                    data['yaw_front_to_north_deg'] = delta_front_deg # 북쪽까지 각도(도)

    def to_original_coords(self, x, y):
            orig_h, orig_w = self.frame_shape
            disp_w, disp_h = self.display_size
            orig_x = int(x * orig_w / disp_w)
            orig_y = int(y * orig_h / disp_h)
            return orig_x, orig_y

            
    def draw_tag_overlay(self, frame, tag_info):
        for tag_id, data in tag_info.items():
            if data.get("status") != "On":
                continue

            # 픽셀 표시는 그대로
            cx_raw, cy_raw = data.get("center_raw", (None, None))
            if cx_raw is None or self.board_result is None:
                continue
            center_raw = (int(cx_raw), int(cy_raw))
            cv2.circle(frame, center_raw, 6, (0, 0, 255), 2)

            cx, cy = data.get("center", (None, None))
            if cx is None:
                continue
            center = (int(cx), int(cy))
            cv2.circle(frame, center, 6, (255, 0, 0), 2)

            if self.board_result and self.board_result.grid_reference and data.get("grid_position") is not None:
                ref = self.board_result.grid_reference
                H_inv = np.linalg.inv(ref["H_metric"])
                row, col = data["grid_position"]
                idx = row * self.grid_col + col
                if idx < len(ref["cell_centers"]):
                    grid_cm = np.array([ref["cell_centers"][idx]], dtype=np.float32).reshape(-1,1,2)
                    gx, gy = cv2.perspectiveTransform(grid_cm, H_inv)[0][0]
                    grid_pt = (int(gx), int(gy))

                    cv2.line(frame, grid_pt, center, (0, 0, 255), 2)

                    dist = data.get("dist_cm")
                    rel  = data.get("relative_angle_deg")
                    if dist is not None and rel is not None:
                        deg = int(abs(round(rel)))
                        LR  = f"L{deg}" if rel < 0 else f"R{deg}"
                        cv2.putText(frame, f"{LR}, {dist:.1f}cm",
                                    (center[0] + 5, center[1] + 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

                    n_front = data.get("yaw_front_to_north_deg")
                    if n_front is not None:
                        cv2.putText(frame, f"N: {n_front:+.1f}°",
                                    (center[0] + 5, center[1] + 65),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
                        
                    base_dir = data.get("heading_base_dir")
                    delta    = data.get("heading_offset_deg")
                    if base_dir is not None and delta is not None:
                        if delta < -45 or delta > 45:
                            heading_str = f"{base_dir}:ERR"
                        else:
                            sign = "+" if delta >= 0 else "-"
                            heading_str = f"{base_dir}:{sign}{abs(round(delta, 1)):.1f}"
                        cv2.putText(frame, f"H: {heading_str}",
                                    (center[0] + 5, center[1] + 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    
    def _compute_roi(
        self,
        raw_gray: np.ndarray,
        frame_shape: Tuple[int, int, int],
        board,
        board_result,
        manual_tl: Optional[Tuple[int,int]],
        manual_br: Optional[Tuple[int,int]],
        board_tag: Optional[Dict],
    ) -> Tuple[np.ndarray, Tuple[int,int,int,int]]:
        """
        우선순위:
        1) board.is_locked
        2) 수동 ROI
        3) Tag 기반 ROI
        4) 전체 화면
        """
        h, w = frame_shape[:2]

        # 1순위. lock된 보드
        if board.is_locked and board_result is not None:
            corners = board_result.corners
            xs = corners[:, 0].astype(int)
            ys = corners[:, 1].astype(int)
            x_min, x_max = xs.min(), xs.max()
            y_min, y_max = ys.min(), ys.max()

        # 2순위. 수동 ROI 선택
        elif manual_tl and manual_br:
            x1, y1 = manual_tl
            x2, y2 = manual_br
            x_min, x_max = min(x1, x2), max(x1, x2)
            y_min, y_max = min(y1, y2), max(y1, y2)

        # 3순위. 보드 태그가 검출된 경우
        elif board_tag is not None and "corners" in board_tag:
            tag_corners = np.array(board_tag["corners"], dtype=np.float32)
            tag_len_cm = tag_size * 100.0
            tag_right_len = np.linalg.norm(tag_corners[0] - tag_corners[1])
            tag_up_len    = np.linalg.norm(tag_corners[0] - tag_corners[3])
            px_per_cm_r = tag_right_len / tag_len_cm
            px_per_cm_u = tag_up_len    / tag_len_cm

            roi_w = int(px_per_cm_r * board_width_cm  * 1.18)
            roi_h = int(px_per_cm_u * board_height_cm * 1.18)
            center_x, center_y = tag_corners.mean(axis=0)

            x_min = int(max(0, center_x - roi_w/5))
            x_max = int(min(w, center_x + roi_w))
            y_min = int(max(0, center_y - roi_h))
            y_max = int(min(h, center_y + roi_h/5))

        # 4순위. 전체 화면
        else:
            x_min, y_min, x_max, y_max = 0, 0, w, h

        # size 체크: 잘못된 범위일 때 전체 화면으로 fallback
        if x_max <= x_min or y_max <= y_min:
            x_min, y_min, x_max, y_max = 0, 0, w, h

        roi = raw_gray[y_min:y_max, x_min:x_max]
        return roi, (x_min, y_min, x_max, y_max)
    

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

    def binarize(self, img: np.ndarray) -> np.ndarray:
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

        # 2) CLAHE
        clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_bin,
            tileGridSize=self.clahe_tile_bin
        )
        gray = clahe.apply(gray)  # :contentReference[oaicite:6]{index=6}

        # 3) Adaptive Threshold
        bsize = max(3, self.adaptive_block)
        if bsize % 2 == 0: bsize += 1
        thresh = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            bsize,
            self.adaptive_C
        )  # :contentReference[oaicite:7]{index=7}

        # 4) Noise removal
        median = cv2.medianBlur(thresh, 3)

        # 5) Invert colors
        inverted = cv2.bitwise_not(median)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        closed = cv2.morphologyEx(inverted, cv2.MORPH_CLOSE, kernel)

        return closed

    def enhance(self, img: np.ndarray) -> np.ndarray:
        """
        1) Upscale (크롭 후 확대)  
        2) Unsharp Mask (선명도 향상)  
        3) CLAHE  
        4) Bilateral Filter (엣지 보존 스무딩)  
        """
        # 1) Upscale
        up = cv2.resize(
            img, None,
            fx=self.scale_enh, fy=self.scale_enh,
            interpolation=cv2.INTER_CUBIC
        )

        # 2) Unsharp mask
        blur = cv2.GaussianBlur(up, self.unsharp_ksize, self.unsharp_sigma)
        sharp = cv2.addWeighted(up, 1.5, blur, -0.5, 0)

        # 3) CLAHE
        clahe = cv2.createCLAHE(
            clipLimit=self.clahe_clip_enh,
            tileGridSize=self.clahe_tile_enh
        )
        enhanced = clahe.apply(sharp)

        # 4) Bilateral
        return cv2.bilateralFilter(
            enhanced,
            d=self.bilateral_d,
            sigmaColor=self.bilateral_sigma_color,
            sigmaSpace=self.bilateral_sigma_space
        )  # :contentReference[oaicite:8]{index=8}
