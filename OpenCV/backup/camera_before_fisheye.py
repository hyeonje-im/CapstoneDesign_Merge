import cv2

def camera_open(source=None):
    if source is None:
        preferred_order = [1, 0]
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


def camera_frame(cap):
    ret, frame = cap.read()
    if not ret:
        return None
    return frame

def camera_undistort(frame, camera_matrix, dist_coeffs):
    h, w = frame.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return frame, new_camera_matrix

def frame_process(cap, camera_matrix, dist_coeffs):
    frame = camera_frame(cap)
    if frame is None:
        return None, None
    frame_undistort, new_camera_matrix = camera_undistort(frame, camera_matrix, dist_coeffs)
    frame_gray = cv2.cvtColor(frame_undistort, cv2.COLOR_BGR2GRAY)
    return frame_undistort, frame_gray, new_camera_matrix