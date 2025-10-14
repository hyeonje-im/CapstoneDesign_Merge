from vision.board import BoardDetector, BoardDetectionResult
from config import tag_role, tag_size
import numpy as np
import cv2


class TagBoardDetector(BoardDetector):
    def __init__(self, board_width_cm, board_height_cm, grid_row, grid_col):
        super().__init__(board_width_cm, board_height_cm, grid_row, grid_col)
        self._tag_size_cm = tag_size*100    # cm 단위로 변환
        self.grid_row = grid_row
        self.grid_col = grid_col
        self._last_tag_px = None
        self._result = None
        self._locked = False

    def detect(self, binary_roi, data):
        corners = self._detect_board(data, binary_roi)
        return corners
    
    def _detect_board(self, tags, frame_gray):
        tag = next((t for t in tags if t.tag_id in tag_role and t.corners is not None), None)
        if tag is None:
            return None

        corners_2d = tag.corners
        tag_px = np.linalg.norm(corners_2d[0] - corners_2d[1])  # 왼쪽 위 ~ 오른쪽 위
        
        self._last_tag_px = tag_px

        dx = self.board_width_cm / self._tag_size_cm * tag_px
        dy = self.board_height_cm / self._tag_size_cm * tag_px

        tag_origin = corners_2d[0]
        x_axis = corners_2d[1] - corners_2d[0]
        y_axis = corners_2d[3] - corners_2d[0]
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        pt0 = tag_origin
        pt1 = pt0 + x_axis * dx
        pt2 = pt0 + x_axis * dx + y_axis * dy
        pt3 = pt0 + y_axis * dy

        rect = np.array([pt0, pt1, pt2, pt3], dtype=np.float32)
        print(f"[DEBUG] Trying to find board tag ID: {tag_role}")

        return rect

    def _calculate_cm_per_px(self, warped_width_px, warped_height_px):
        if self._last_tag_px is None:
            return super()._calculate_cm_per_px(warped_width_px, warped_height_px)
        scale = self._tag_size_cm / max(self._last_tag_px, 1)
        return (scale, scale)

    def process(self, frame_gray, data, rect_override) -> BoardDetectionResult | None:
        if self._locked:
            return self._result

        if not isinstance(data, (list, dict)):
                return self._result

        tags = list(data.values()) if isinstance(data, dict) else data

        rect = self._detect_board(tags, frame_gray)

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

    def get_position_px(self, col: int, row: int, result: BoardDetectionResult) -> np.ndarray:
        # corners order: tl, tr, br, bl
        tl, tr, br, bl = result.corners
        # compute unit vectors
        vec_x = (tr - tl) / self.grid_col
        vec_y = (bl - tl) / self.grid_row
        # compute position
        return tl + vec_x * col + vec_y * row

    def get_grid_overlay_points(self, result: BoardDetectionResult):
        # 상속된 BoardDetector의 메서드 사용
        return super().get_grid_overlay_points(result)
    

    def draw(self, frame, result: BoardDetectionResult):
        # 1) 보드 결과 없으면 넘어감
        if result is None:
            return frame

        # 2) 보드 외곽선은 항상 그리기
        cv2.polylines(frame,
                      [np.int32(result.corners)],
                      isClosed=True,
                      color=(0, 255, 0),
                      thickness=2)
        cv2.circle(frame, tuple(result.origin[:2].astype(int)), 5, (0, 0, 255), -1)
        # 3) 격자 overlay 계산
        overlay = self.get_grid_overlay_points(result)
        if overlay is None:
            return frame

        # 4) 각 셀 중앙에 점 찍기 (초록)
        for x, y in overlay["centers"]:
            cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 0), -1)

        # 5) 셀 경계선 그리기 (노랑)
        for pt1, pt2 in overlay["lines"]:
            cv2.line(frame,
                     (int(pt1[0]), int(pt1[1])),
                     (int(pt2[0]), int(pt2[1])),
                     (0, 255, 255), 1)
        return frame



# 초록색 보드 외곽선
        