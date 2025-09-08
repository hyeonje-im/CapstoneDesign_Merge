import cv2
import numpy as np
import math
from vision.apriltag import AprilTagDetector
from config import board_width_cm, board_height_cm, grid_row, grid_col, cell_size, cell_size_cm
from vision.board import BoardDetectionResult, ROIProcessor

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
        self.roi_processor = ROIProcessor()
        # 임시 ROI 설정
        self.roi_top_left = None
        self.roi_bottom_right = None
        self.selecting_roi = False

        #화면 해상도 설정
        self.raw_shape = None
        self.target_display_size = (960, 540)
        self.display_size = None

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
    def process_frame(self, raw_frame, detect_params=None):
        self.raw_shape = raw_frame.shape[:2]
        frame, new_camera_matrix = self.undistorter.undistort(raw_frame)
        raw_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_count += 1

        tags = self.tags.detect(raw_gray)
        
        if self.roi_top_left and self.roi_bottom_right:
            x1, y1 = self.roi_top_left
            x2, y2 = self.roi_bottom_right
            x_min, x_max = min(x1, x2), max(x1, x2)
            y_min, y_max = min(y1, y2), max(y1, y2)
            cropped = raw_gray[y_min:y_max, x_min:x_max]
        else:
            x_min, y_min = 0, 0
            cropped = raw_gray

        roi_gray = self.roi_processor.process(cropped)

        # 1. 보드 탐지 또는 고정 상태 유지
        if self.board_mode == 'tag':
            data = tags
        else:
            data = detect_params

        # 1) ROI에서 검출 시도
        corners_roi = self.board.detect(roi_gray, data)
        if corners_roi is not None:
            # ROI 성공 → override 좌표로 풀프레임 검출
            rect_full = corners_roi.copy()
            rect_full[:,0,0] += x_min
            rect_full[:,0,1] += y_min
            self.board.process(raw_gray, data, rect_override=rect_full)
        else:
            # ROI 실패 → 순수 풀프레임 검출
            self.board.process(raw_gray, data, rect_override=None)

        # 2) 딱 한 번만 get_result() 호출
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

        return {
            "frame": display_frame,
            "tag_info": tag_info,
            "warped": self.board_result.warped if self.board_result else None,
        }

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

    def transform_coordinates(self, tag_infos: dict[int, dict]):
        """
        각 태그 픽셀 좌표를 H_metric으로 보드 평면의 cm 좌표로 투영한 뒤,
        미리 생성된 cm 단위 grid cell 중심과의 유클리드 거리를 계산합니다.
        """
        # 보드가 lock 되어 있고, grid_reference가 준비되지 않았다면 건너뜀
        if not (self.board_result and self.board.is_locked and self.board_result.grid_reference):
            return

        ref = self.board_result.grid_reference
        H = ref["H_metric"]             # metric homography
        centers = ref["cell_centers"]    # [(cx_cm, cy_cm), ...] flat list

        # 한 셀의 cm 크기
        cw = cell_size_cm
        ch = cell_size_cm

        for tag_id, data in tag_infos.items():
            cx_px, cy_px = data.get("center", (None, None))
            if cx_px is None or cy_px is None:
                continue

            # 픽셀 → (X_cm, Y_cm)
            pts = np.array([[[cx_px, cy_px]]], dtype=np.float32)
            X_cm, Y_cm = cv2.perspectiveTransform(pts, H)[0][0]

            # grid index 계산 및 클램핑
            col = int(X_cm // cw)
            row = int(Y_cm // ch)
            col = max(0, min(self.grid_col - 1, col))
            row = max(0, min(self.grid_row - 1, row))
            data["grid_position"] = (row, col)

            # 실제 거리 계산
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

            cx_px, cy_px = data.get("center", (None, None))
            if cx_px is None or self.board_result is None:
                continue
            center = (int(cx_px), int(cy_px))

            # 보드가 lock 되어 있으면 그리드 중심도 구해서 선 그리기
            if self.board_result and self.board_result.grid_reference:
                ref = self.board_result.grid_reference
                H = ref["H_metric"]
                H_inv = np.linalg.inv(H)
                row, col = data.get("grid_position", (None, None))
                if row is not None:
                    idx = row * self.grid_col + col
                    if idx < len(ref["cell_centers"]):
                        # cm→pixel
                        grid_cm = np.array([ref["cell_centers"][idx]], dtype=np.float32).reshape(-1,1,2)
                        gx, gy = cv2.perspectiveTransform(grid_cm, H_inv)[0][0]
                        grid_pt = (int(gx), int(gy))

                        # 붉은 선
                        cv2.line(frame, grid_pt, center, (0, 0, 255), 2)

                        # 거리·각도 표시
                        dist = data.get("dist_cm")
                        rel = data.get("relative_angle_deg")
                        if dist is not None and rel is not None:
                            deg = int(abs(round(rel)))
                            LR = f"L{deg}" if rel < 0 else f"R{deg}"
                            text = f"{LR}, {dist:.1f}cm"
                            cv2.putText(frame, text,
                                        (center[0] + 5, center[1] + 50),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0, 0, 255), 1)
