import cv2
import numpy as np

def camera_open(source=None):
    if source is None:
        preferred_order = [ 1,0]
        for cam_id in preferred_order:
            cap = cv2.VideoCapture(cam_id)
            if cap.isOpened():
                print(f"[INFO] Using default camera {cam_id}")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                fps = cap.get(cv2.CAP_PROP_FPS)
                return cap, fps
        raise RuntimeError("Error: Cannot open any camera.")
    else:
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise RuntimeError(f"Error: Cannot open source {source}")
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"[INFO] Opened video source {source} with fps: {fps}")
        return cap, fps

# def camera_undistort(frame, camera_type, camera_matrix, dist_coeffs, balance=0.2):
#     if camera_type == 'fisheye':
#         h, w = frame.shape[:2]
#         # 새로운 카메라 행렬 계산 (비율 유지 + fisheye 방식)
#         new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
#             camera_matrix, dist_coeffs, (w, h), np.eye(3), balance=balance
#         )
#         # 맵 생성
#         map1, map2 = cv2.fisheye.initUndistortRectifyMap(
#             camera_matrix, dist_coeffs, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2
#         )
#         # remap을 통해 왜곡 보정
#         undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
#         return undistorted, new_camera_matrix
#     else:
#         h, w = frame.shape[:2]
#         new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
#         frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
#         return frame, new_camera_matrix

class Undistorter:
    def __init__(self, camera_type, camera_matrix, dist_coeffs, image_size, balance=0.2):
        self.camera_type = camera_type
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        h, w = image_size
        if camera_type == 'fisheye':
            self.new_cam = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                camera_matrix, dist_coeffs, (w, h), np.eye(3), balance=balance
            )
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, np.eye(3),
                self.new_cam, (w, h), cv2.CV_16SC2
            )
        else:
            self.new_cam, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h)
            )
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, None,
                self.new_cam, (w, h), cv2.CV_16SC2
            )

    def undistort(self, frame):
        return cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR), self.new_cam

    
    # def camera_frame(cap):
#     ret, frame = cap.read()
#     if not ret:
#         return None
#     return frame

# def frame_process(cap, camera_matrix, dist_coeffs):
#     frame = camera_frame(cap)
#     if frame is None:
#         return None, None
#     frame_undistort, new_camera_matrix = camera_undistort(frame, camera_matrix, dist_coeffs)
#     frame_gray = cv2.cvtColor(frame_undistort, cv2.COLOR_BGR2GRAY)
#     return frame_undistort, frame_gray, new_camera_matrix