import numpy as np
from pupil_apriltags import Detector
import cv2
from OpenCV.config import detected_ids, dist_coeffs, board_width_cm, board_height_cm, tag_info

_detector = None

class AprilTagDetector:
    def __init__(self):
        self.detector = Detector(families="tag36h11")
        self.detected_ids = set()  # 현재 감지된 태그 ID들
        self.tag_info = {}  # 태그별 정보 저장

    def tag_detect(self, gray):
        return self.detector.detect(gray)

    def tags_process(self,tags, object_points, frame_count, board_origin_tvec, cm_per_px, frame, new_camera_matrix, dist_coeffs, visualize=True):
        global tag_info
        
        new_detected_ids = set()
        
        for tag_id in list(tag_info.keys()):
            if tag_id not in new_detected_ids: 
                last_seen_frame = tag_info[tag_id]["frame_count"]
                if frame_count - last_seen_frame > 30:
                    tag_info[tag_id]["status"] = "Off"

        for tag in tags:
            tag_id = tag.tag_id
            rvec, tvec = tag_pose(tag, object_points, new_camera_matrix)
            center = tuple(map(int, tag.center))
            rmat, _ = cv2.Rodrigues(rvec)
            yaw, pitch, roll = cv2.decomposeProjectionMatrix(np.hstack((rmat, tvec)))[-1]
            tag_x = (center[0] - board_origin_tvec[0])
            tag_y = (center[1] - board_origin_tvec[1])
            tag_cm_x = float(tag_x * cm_per_px[0])
            tag_cm_y = float(tag_y * cm_per_px[1])
            status = "On"

            tag_info[tag_id] = {
                "status": status,
                "3D_coordinates": (tvec[0][0], tvec[1][0], tvec[2][0]),
                "rotation": (roll[0], pitch[0], yaw[0]),
                "coordinates": (float(tag_cm_x), float(tag_cm_y)),
                "frame_count": frame_count
            }
            new_detected_ids.add(tag_id)
            if visualize:
                imgpts, _ = cv2.projectPoints(object_points, rvec, tvec, new_camera_matrix, dist_coeffs)
                imgpts = np.int32(imgpts).reshape(-1, 2)

                cv2.drawContours(frame, [imgpts[:4]], -1, (0, 255, 0), 2)
                cv2.putText(frame, f"ID: {tag_id}", (center[0], center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 3)

        self.detected_ids = new_detected_ids

def get_detector():
    global _detector
    if _detector is None:
        _detector = Detector(families="tag36h11")
    return _detector

# def tag_detect(gray):
#     detector = get_detector()
#     return detector.detect(gray)

def tag_pose(tag, object_points, new_camera_matrix):
    corners = tag.corners.astype(np.float32)
    retval, rvec, tvec = cv2.solvePnP(object_points[:4], corners, new_camera_matrix, dist_coeffs)
    return rvec, tvec

# def tag_update(tag_info, detected_ids, frame_count):
#     for tag_id in list(tag_info.keys()):
#         if tag_id not in detected_ids:
#             last_seen_frame = tag_info[tag_id]["frame_count"]
#             if frame_count - last_seen_frame > 30: 
#                 tag_info[tag_id]["status"] = "Off"


def cm_per_px(warped_board_width_px, warped_board_height_px):
    cm_per_px_x = board_width_cm / max(warped_board_width_px, 1)
    cm_per_px_y = board_height_cm / max(warped_board_height_px, 1)
    return (cm_per_px_x, cm_per_px_y)


# def tags_process(tags, object_points, frame_count, board_origin_tvec, cm_per_px, frame, new_camera_matrix, dist_coeffs):
#     global detected_ids

#     new_detected_ids = set()
    
#     for tag_id in list(tag_info.keys()):
#         if tag_id not in new_detected_ids: 
#             last_seen_frame = tag_info[tag_id]["frame_count"]
#             if frame_count - last_seen_frame > 30:
#                 tag_info[tag_id]["status"] = "Off"

    
#     for tag in tags:
#         tag_id = tag.tag_id
#         rvec, tvec = tag_pose(tag, object_points, new_camera_matrix)
#         center = tuple(map(int, tag.center))
#         rmat, _ = cv2.Rodrigues(rvec)
#         yaw, pitch, roll = cv2.decomposeProjectionMatrix(np.hstack((rmat, tvec)))[-1]
#         tag_x = (center[0] - board_origin_tvec[0])
#         tag_y = (center[1] - board_origin_tvec[1])
#         tag_cm_x = float(tag_x * cm_per_px[0])
#         tag_cm_y = float(tag_y * cm_per_px[1])
#         status = "On"

#         tag_info[tag_id] = {
#             "status": status,
#             "3D_coordinates": (tvec[0][0], tvec[1][0], tvec[2][0]),
#             "rotation": (roll[0], pitch[0], yaw[0]),
#             "coordinates": (float(tag_cm_x), float(tag_cm_y)),
#             "frame_count": frame_count
#         }
#         new_detected_ids.add(tag_id)

#         imgpts, _ = cv2.projectPoints(object_points, rvec, tvec, new_camera_matrix, dist_coeffs)
#         imgpts = np.int32(imgpts).reshape(-1, 2)

#         cv2.drawContours(frame, [imgpts[:4]], -1, (0, 255, 0), 2)
#         cv2.putText(frame, f"ID: {tag_id}", (center[0], center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 3)

#     detected_ids.update(new_detected_ids)


def transform_coordinates(tag_info: dict[int, dict]) -> dict[int, dict]:
    from  OpenCV.config import board_width_cm, board_height_cm, grid_width, grid_height, cell_size

    result = {}
    for tag_id, data in tag_info.items():
        x_cm, y_cm = data["coordinates"]
        tag_grid_x = int(x_cm * grid_width / board_width_cm)
        tag_grid_y = int(y_cm * grid_height / board_height_cm)
        col = tag_grid_x // cell_size
        row = tag_grid_y // cell_size

        row = max(0, min(grid_height - 1, row))
        col = max(0, min(grid_width - 1, col))

        result[tag_id] = {
            **data,
            "grid_position": (row, col)
        }

    return result
