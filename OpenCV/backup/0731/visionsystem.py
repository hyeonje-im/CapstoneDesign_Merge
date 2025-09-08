import cv2
import numpy as np
import math
from vision.apriltag import AprilTagDetector
from config import board_width_cm, board_height_cm, grid_row, grid_col, cell_size, cell_size_cm, tag_size, CORRECTION_COEF
from vision.board import BoardDetectionResult, ROIProcessor

NORTH_TAG_ID = 12


class VisionSystem:
    def __init__(self, undistorter, visualize=True, board_mode='contour'):
        
        self.tags = AprilTagDetector()
        self.visualize = visualize
        self.grid_row = grid_row
        self.grid_col = grid_col
        self.board_mode = board_mode
        self.undistorter = undistorter
        self.last_valid_result = None
        self.board_result: BoardDetectionResult | None = None
        self.frame_count = 0
        #self.roi_processor = ROIProcessor() 수동 rpi기능 제거
        # 임시 ROI 설정
        self.roi_top_left = None
        self.roi_bottom_right = None
        self.selecting_roi = False

        #화면 해상도 설정
        self.raw_shape = None
        self.target_display_size = (960, 540)
        self.display_size = None

        self.correction_coef_getter = lambda: CORRECTION_COEF

        self._init_board_detector()

    def _init_board_detector(self):
        if self.board_mode == 'contour':
            from vision.board import BoardDetector
            self.board = BoardDetector(board_width_cm, board_height_cm, grid_row, grid_col)
        elif self.board_mode == 'tag':
            from vision.board_tag import TagBoardDetector
            self.board = TagBoardDetector(board_width_cm, board_height_cm, grid_row, grid_col)

    def mouse_callback(self, event, x, y, flags, param):
        if not (self.selecting_roi and event == cv2.EVENT_LBUTTONDOWN and self.display_size and self.raw_shape):
            return

        # 표시 크기로 받은 x,y → 원본 좌표로 변환
        x0, y0 = (x, y)
        if self.display_size:
            x0, y0 = self.to_original_coords(x, y)

        if self.roi_top_left is None:
            self.roi_top_left = (x0, y0)
            print(f"[ROI] Top-left set to: {self.roi_top_left}")
        else:
            self.roi_bottom_right = (x0, y0)
            print(f"[ROI] Bottom-right set to: {self.roi_bottom_right}")
            self.selecting_roi = False
    def start_roi_selection(self):
        self.roi_top_left = None
        self.roi_bottom_right = None
        self.selecting_roi = True
        print("[ROI] Selection mode enabled. Click top-left and bottom-right.")


    def set_board_mode(self, new_mode):
        self.board_mode = new_mode
        self._init_board_detector()
        
    # 프레임 처리
    def process_frame(self, raw_frame, detect_params=None, scale=2):
        self.raw_shape = raw_frame.shape[:2]
        frame, new_camera_matrix = self.undistorter.undistort(raw_frame)
        raw_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_count += 1
        h, w = frame.shape[:2]

        x_min, y_min, x_max, y_max = 0, 0, w, h
        if self.board_result and hasattr(self.board_result, 'bounding_box'):
            bx, by, bw, bh = self.board_result.bounding_box
            x_min, y_min, x_max, y_max = bx, by, bx + bw, by + bh
        board_tag = self.tags.get_board_tag()
        if board_tag is not None and "corners" in board_tag:
            tag_corners = np.array(board_tag["corners"], dtype=np.float32)
            # 태그 실측 길이(cm) 계산 (config.tag_size는 미터 단위)
            tag_len_cm = tag_size * 100.0
            # 태그 픽셀 길이 계산
            tag_right_len = np.linalg.norm(tag_corners[0] - tag_corners[1])
            tag_up_len    = np.linalg.norm(tag_corners[0] - tag_corners[3])
            px_per_cm_right = tag_right_len / tag_len_cm
            px_per_cm_up    = tag_up_len    / tag_len_cm
            # ROI 크기 (보드 크기 + 18% 마진)
            roi_w = int(px_per_cm_right * board_width_cm  * 1.18)
            roi_h = int(px_per_cm_up    * board_height_cm * 1.18)
            # 5cm 만큼 추가 마진
            offset_px_right = px_per_cm_right * 5
            offset_px_up    = px_per_cm_up    * 5
            # ROI 원점 계산
            tag_right_unit = (tag_corners[2] - tag_corners[3]) / (tag_right_len + 1e-8)
            tag_up_unit  = (tag_corners[0] - tag_corners[3]) / (tag_right_len + 1e-8)
            roi_origin = tag_corners[3] - tag_right_unit * offset_px_right - tag_up_unit * offset_px_up
            x_min = int(roi_origin[0])
            x_max = min(frame.shape[1], x_min + roi_w)
            y_max = int(roi_origin[1])
            y_min = max(0, y_max - roi_h)
            # 빨간색 ROI 표시
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0,0,255), 2)

        roi_box = (x_min, y_min, x_max - x_min, y_max - y_min)
        filtered_gray = self.filter_roi(frame, roi_box, scale)

        # --- ① AprilTag으로 자동 ROI 계산 및 빨간 테두리 그리기 ---
        raw_tags = self.tags.detect(filtered_gray)
        tags = self.correct_tag_coordinates(raw_tags, roi_box, scale)
        self.tags.update(tags, self.frame_count, new_camera_matrix)        
        rect_override = None
        if board_tag is not None and "corners" in board_tag:
            roi_gray = raw_gray[y_min:y_max, x_min:x_max]
            
            # --- ② ROI 내에서 가장 큰 흰색 컨투어(4각형) 검출 ---
            blur = cv2.GaussianBlur(roi_gray, (5,5), 0)
            edges = cv2.Canny(blur, 60, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_rect, best_area = None, 0
            for cnt in contours:
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                if len(approx) == 4 and cv2.isContourConvex(approx):
                    area = cv2.contourArea(approx)
                    if area > best_area:
                        best_area, best_rect = area, approx
            if best_rect is not None:
                # ROI 좌표계를 전체 프레임 좌표계로 변환
                best_rect = best_rect.reshape(4,2) + np.array([x_min, y_min])
                rect_override = best_rect.reshape(4,1,2).astype(np.float32)

        # --- ③ 보드 검출: red-ROI 기반 rect_override 전달 ---
        self.board.process(raw_gray, detect_params, rect_override=rect_override)
        self.board_result = self.board.get_result()
                
        # 2. 태그 탐지 및 처리
        if self.board_result:
            self.tags.update_and_process(tags, self.frame_count, self.board_result.origin, self.board_result.cm_per_px, new_camera_matrix)
        else:
            self.tags.update(tags, self.frame_count, new_camera_matrix)

        tag_info = self.tags.get_raw_tags()
        if self.board_result:
            self.transform_coordinates(tag_info)
            self.compute_tag_orientation(tag_info)
        
        north = tag_info.get(NORTH_TAG_ID)
        if north is not None and north.get('status') == 'On':
            north_yaw = north['yaw']  # 라디안
            for tag_id, data in tag_info.items():
                # 자신(태그3)은 건너뛰고, 검출 상태인 태그만
                if tag_id == NORTH_TAG_ID or data.get('status') != 'On':
                    continue
                cur_yaw = data.get('yaw')
                # –π~+π 범위로 정규화한 Δ값
                delta = ((cur_yaw - north_yaw + math.pi) % (2*math.pi)) - math.pi
                # 태그 정보에 추가 저장
                data['yaw_to_north_rad'] = delta
                data['yaw_to_north_deg'] = math.degrees(delta)
        
        

        # 3. 시각화 처리
        if self.visualize:
            self.board.draw(frame, self.board_result)
            self.tags.draw(frame)
            self.draw_tag_overlay(frame, tag_info)

        if self.roi_top_left and self.roi_bottom_right:
            roi_display = roi_gray.copy()
            roi_display = cv2.resize(roi_display, (min(roi_display.shape[1]*2, 800), min(roi_display.shape[0]*2, 800)))
            cv2.imshow("ROI_Display", roi_display)


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
                                roi_box: tuple[int, int, int, int],
                                scale: float = 2) -> list:
        x0, y0, _, _ = roi_box
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

    def correct_tag_position_polar(self, X, Y, Cx, Cy, coef=None):
        if coef is None:
            coef = self.correction_coef_getter()
        dx = X - Cx
        dy = Y - Cy
        r = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy, dx)
        r_prime = r * coef
        X_prime = Cx + r_prime * math.cos(theta)
        Y_prime = Cy + r_prime * math.sin(theta)
        return X_prime, Y_prime

    def transform_coordinates(self, tag_infos: dict[int, dict]):
        if not (self.board_result and self.board.is_locked and self.board_result.grid_reference):
            return
        ref = self.board_result.grid_reference
        H = ref["H_metric"]
        centers = ref["cell_centers"]
        Cx = board_width_cm / 2
        Cy = board_height_cm / 2
        cw = cell_size_cm
        ch = cell_size_cm
        for tag_id, data in tag_infos.items():
            cx_px, cy_px = data.get("center", (None, None))
            if cx_px is None or cy_px is None:
                continue
            pts = np.array([[[cx_px, cy_px]]], dtype=np.float32)
            X_cm, Y_cm = cv2.perspectiveTransform(pts, H)[0][0]
            # --------- [보정값 동적으로 적용] ---------
            X_corr, Y_corr = self.correct_tag_position_polar(X_cm, Y_cm, Cx, Cy)
            data["corrected_center"] = (X_corr, Y_corr)
            col = int(X_corr // cw)
            row = int(Y_corr // ch)
            col = max(0, min(self.grid_col - 1, col))
            row = max(0, min(self.grid_row - 1, row))
            data["grid_position"] = (row, col)
            idx = row * self.grid_col + col
            if idx < len(centers):
                cx_cm, cy_cm = centers[idx]
                data["dist_cm"] = math.hypot(X_corr - cx_cm, Y_corr - cy_cm)
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
            center_px = np.array(data["center"], dtype=np.float32).reshape(-1,1,2)
            tag_cm = cv2.perspectiveTransform(center_px, H)[0][0]
            row, col = data["grid_position"]
            grid_cm = grid_centers[row * self.grid_col + col]
            dir_vec = tag_cm - np.array(grid_cm)

            # 3) 각도 계산 (atan2: rad → deg)
            yaw_front = math.degrees(math.atan2(front_vec[1],  front_vec[0]))
            yaw_dir   = math.degrees(math.atan2(dir_vec[1],    dir_vec[0]))

            # 4) 상대 각도 저장
            data["yaw_front_deg"]    = yaw_front
            data["yaw_to_grid_deg"]  = yaw_dir
            data["relative_angle_deg"] = ((yaw_dir - yaw_front + 180) % 360) - 180

    def to_original_coords(self, x, y):
            orig_h, orig_w = self.raw_shape
            disp_w, disp_h = self.display_size
            orig_x = int(x * orig_w / disp_w)
            orig_y = int(y * orig_h / disp_h)
            return orig_x, orig_y

        
    def draw_tag_overlay(self, frame, tag_info):
        for tag_id, data in tag_info.items():
            if data.get("status") != "On":
                continue

            # 원래 태그 위치 표시 (빨간 원)
            cx_px, cy_px = data.get("center", (None, None))
            if cx_px is None or self.board_result is None:
                continue
            center = (int(cx_px), int(cy_px))
            cv2.circle(frame, center, 6, (0, 0, 255), 2)

            # 보정된 태그 위치 계산 및 표시 (파란 원)
            corr_px_int = None
            if self.board_result and "corrected_center" in data:
                X_corr, Y_corr = data["corrected_center"]
                ref = self.board_result.grid_reference
                H_inv = np.linalg.inv(ref["H_metric"])
                pt_cm = np.array([[[X_corr, Y_corr]]], dtype=np.float32)
                corr_px = cv2.perspectiveTransform(pt_cm, H_inv)[0][0]
                corr_px_int = (int(corr_px[0]), int(corr_px[1]))
                cv2.circle(frame, corr_px_int, 6, (255, 0, 0), 2)
                cv2.putText(frame, "Corr", (corr_px_int[0] + 5, corr_px_int[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

            # 그리드 중심과 보정된 좌표를 잇는 선 그리기 및 거리/각도 표시
            if self.board_result and self.board_result.grid_reference and \
               corr_px_int is not None and data.get("grid_position") is not None:
                ref = self.board_result.grid_reference
                H_inv = np.linalg.inv(ref["H_metric"])
                row, col = data.get("grid_position", (None, None))
                if row is not None:
                    idx = row * self.grid_col + col
                    if idx < len(ref["cell_centers"]):
                        # cm → 픽셀 변환
                        grid_cm = np.array([ref["cell_centers"][idx]], dtype=np.float32).reshape(-1, 1, 2)
                        gx, gy = cv2.perspectiveTransform(grid_cm, H_inv)[0][0]
                        grid_pt = (int(gx), int(gy))

                        # 붉은 선
                        cv2.line(frame, grid_pt, corr_px_int, (0, 0, 255), 2)

                        # 거리·각도 텍스트
                        dist = data.get("dist_cm")
                        rel = data.get("relative_angle_deg")
                        if dist is not None and rel is not None:
                            deg = int(abs(round(rel)))
                            LR = f"L{deg}" if rel < 0 else f"R{deg}"
                            text = f"{LR}, {dist:.1f}cm"
                            cv2.putText(frame, text,
                                        (corr_px_int[0] + 5, corr_px_int[1] + 50),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0, 0, 255), 1)

    def filter_roi(self,
                frame: np.ndarray,
                roi_box: tuple[int, int, int, int],
                scale: int = 2,
                unsharp_ksize=(9, 9),
                unsharp_sigma=10,
                clahe_clip=3.0,
                clahe_grid=(8, 8),
                bilateral_d=9,
                bilateral_sigma_color=75,
                bilateral_sigma_space=75) -> np.ndarray:
        """
        frame: BGR 컬러 프레임
        roi_box: (x, y, w, h) 포맷의 ROI 좌표
        scale: ROI 업스케일 배율

        ROI 영역에 대해 다음 필터 체인을 단일 함수로 적용합니다:
        1) 크롭 & 업스케일
        2) BGR->GRAY
        3) 언샤프 마스크
        4) CLAHE
        5) 양방향 필터

        반환: 필터 적용된 그레이스케일 ROI 이미지
        """
        x, y, w, h = roi_box
        # 1) ROI 크롭 및 업스케일
        roi = frame[y:y+h, x:x+w]
        upscaled = cv2.resize(roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

        # 2) 그레이 변환
        gray = cv2.cvtColor(upscaled, cv2.COLOR_BGR2GRAY)

        # 3) 언샤프 마스크
        blurred = cv2.GaussianBlur(gray, unsharp_ksize, unsharp_sigma)
        unsharp = cv2.addWeighted(gray, 1.5, blurred, -0.5, 0)

        # 4) CLAHE
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=clahe_grid)
        enhanced = clahe.apply(unsharp)

        # 5) 양방향 필터
        filtered = cv2.bilateralFilter(
            enhanced,
            d=bilateral_d,
            sigmaColor=bilateral_sigma_color,
            sigmaSpace=bilateral_sigma_space
        )

        return filtered
    
    