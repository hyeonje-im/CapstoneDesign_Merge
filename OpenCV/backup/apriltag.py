import numpy as np
from pupil_apriltags import Detector
import cv2
from config import dist_coeffs, object_points, tag_role

class AprilTagDetector:
    def __init__(self, k_getter):
        self.detector = Detector(families="tag36h11")
        self.detected_ids = set()  # 현재 감지된 태그 ID들
        self.tag_info = {}  # 태그별 정보 저장
        self.board_tag = None
        self.robot_tag = []
        self.k_getter = k_getter

    def detect(self, gray):
        return self.detector.detect(gray)

    # 태그 정보 업데이트
    def update(self,tags,frame_count, new_camera_matrix, frame_shape):
        self.tag_info.clear()
        self.detected_ids.clear()
        new_ids = set()
        h, w = frame_shape[:2]
        cx, cy = w * 0.5, h * 0.5
        k = float(self.k_getter())
        
        # 태그별 처리
        for tag in tags:
            tag_id = tag.tag_id
            rvec, tvec = solve_pose(tag, object_points, new_camera_matrix)
            center = np.array(tag.center)
            rmat, _ = cv2.Rodrigues(rvec)
            yaw, pitch, roll = cv2.decomposeProjectionMatrix(np.hstack((rmat, tvec)))[-1]
            status = "On"
            yaw_deg = compute_yaw_deg(tag)

            x_corr = cx + k * (center[0] - cx)
            y_corr = cy + k * (center[1] - cy)
            x_corr = max(0, min(w - 1, x_corr))
            y_corr = max(0, min(h - 1, y_corr))


            self.tag_info[tag_id] = {
                "id": tag_id,
                "center_raw": center,
                "center": (int(round(x_corr)), int(round(y_corr))),
                "status": status,
                "corners": tag.corners,
                "tvec": tvec,
                "rotation": (roll[0], pitch[0], yaw[0]),
                "yaw": yaw_deg,
                "frame_count": frame_count,
            }

            new_ids.add(tag_id)
        
        for tag_id in list(self.tag_info.keys()):
            if tag_id not in new_ids:
                if frame_count - self.tag_info[tag_id]["frame_count"] > 30:
                    self.tag_info[tag_id]["status"] = "Off"

        self.detected_ids = new_ids
        self._classify()

    def _classify(self):
        self.board_tag = None
        self.robot_tags = []
        for tag_id, data in self.tag_info.items():
            role = tag_role.get(tag_id, "robot")
            if role == "board" and self.board_tag is None:
                self.board_tag = data
            elif role == "robot":
                self.robot_tags.append(data)

    def process(self, board_origin_tvec: np.ndarray, cm_per_px: tuple):
        if not self.robot_tags:
            return  # 보드 정보 없으면 처리 생략

        for tag in self.robot_tags:
            center_px = tag["center"]
            offset_px = center_px - board_origin_tvec[:2]

            tag_cm_x = float(offset_px[0] * cm_per_px[0])
            tag_cm_y = float(offset_px[1] * cm_per_px[1])
            tag["coordinates"] = (tag_cm_x, tag_cm_y)

    def update_and_process(self, tags, frame_count, board_origin, cm_per_px, camera_matrix):
        self.update(tags, frame_count, camera_matrix)
        self.process(board_origin, cm_per_px)

    def draw(self, frame: np.ndarray):
        arrow_length = 150  # 화살표 길이
        for tag_id, data in self.tag_info.items():
            # 사각형 그리기
            if "corners" in data:
                corners = data["corners"].astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

            # yaw 방향 화살표 그리기
            if "yaw" not in data or "center" not in data:
                continue
            center = tuple(map(int, data["center"]))
            yaw_deg = data["yaw"]
            theta = np.deg2rad(yaw_deg)

            dx = int(arrow_length * np.cos(theta))
            dy = int(-arrow_length * np.sin(theta))  # OpenCV 영상 좌표계 보정

            pt1 = center
            pt2 = (center[0] + dx, center[1] + dy)
            cv2.arrowedLine(frame, pt1, pt2, (0, 222, 0), 2, tipLength=0.3)
            

            # 중심 ID 텍스트 그리기
            center = tuple(map(int, data["center"]))
            cv2.putText(frame, f"ID: {tag_id}", (center[0], center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def get_board_tag(self):
        return self.board_tag

    def get_robot_tags(self):
        return self.robot_tags

    def get_raw_tags(self):
        return self.tag_info.copy()
    
    def update_tag_info(self) -> dict:
        return self.tag_info

def solve_pose(tag, object_points, new_camera_matrix):
    corners = tag.corners.astype(np.float32)
    retval, rvec, tvec = cv2.solvePnP(object_points[:4], corners, new_camera_matrix, dist_coeffs)
    return rvec, tvec

def compute_yaw_deg(tag):
    pt0 = tag.corners[0]  # top-left
    pt1 = tag.corners[1]  # top-right

    dx = pt1[0] - pt0[0]
    dy = pt1[1] - pt0[1]

    # 영상 좌표계 기준 (origin = top-left, y+ is downward)
    angle_rad = np.arctan2(-dy, dx)  # y축 반전 보정
    angle_deg = np.rad2deg(angle_rad)

    return angle_deg

def estimate_tag_offset_cm(tag, frame_shape, tag_size_cm=3.2):
    """
    화면 중심과 태그 중심의 픽셀 오프셋을 cm 단위로 변환하고
    그 유클리드 거리를 반환합니다.
    """
    img_h, img_w = frame_shape[:2]
    frame_center = np.array([img_w / 2, img_h / 2])
    tag_center   = np.array(tag.center)
    offset_px    = tag_center - frame_center

    corners = tag.corners
    edge_lengths = [
        np.linalg.norm(corners[0] - corners[1]),
        np.linalg.norm(corners[1] - corners[2]),
        np.linalg.norm(corners[2] - corners[3]),
        np.linalg.norm(corners[3] - corners[0])
    ]
    avg_edge_px = np.mean(edge_lengths)
    cm_per_px   = tag_size_cm / avg_edge_px

    offset_cm   = offset_px * cm_per_px
    distance_cm = np.linalg.norm(offset_cm)
    return offset_cm, distance_cm


def compute_relative_yaw(tag, camera_matrix, dist_coeffs, object_points):
    """
    태그의 top-left→top-right 벡터로부터 태그가 바라보는 절대 yaw(°)를 계산합니다.
    """
    pt0 = tag.corners[0]
    pt1 = tag.corners[1]
    dx  = pt1[0] - pt0[0]
    dy  = pt1[1] - pt0[1]
    angle_rad = np.arctan2(-dy, dx)
    return np.rad2deg(angle_rad)


def compute_center_yaw(tag, frame_shape):
    """
    화면 중심에서 태그 중심을 향하는 방향의 yaw(°)를 계산합니다.
    """
    frame_center = np.array([frame_shape[1] / 2, frame_shape[0] / 2])
    tag_center   = np.array(tag.center)
    dx, dy       = frame_center - tag_center
    angle_rad    = np.arctan2(-dy, dx)
    return np.rad2deg(angle_rad)


def compute_relative_yaw_difference(yaw_tag, center_yaw):
    """
    태그 절대 yaw와 화면 중심을 향하는 yaw 차이를
    –180°~+180° 범위로 정규화하여 반환합니다.
    """
    return (center_yaw - yaw_tag + 180) % 360 - 180