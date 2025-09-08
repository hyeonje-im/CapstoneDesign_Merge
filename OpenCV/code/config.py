import numpy as np
import os

# 브로커 정보
#연구실 : 192.168.0.25
IP_address_ = "192.168.0.25"
MQTT_PORT      = 1883
MQTT_TOPIC_COMMANDS_ = "command/transfer"

NORTH_TAG_ID = 12  # 바닥에 고정된 북쪽 태그 겸 빨간 ROI생성 기준 태그

critical_dist =12

# ===카메라 매개변수 (캘리브레이션된 값 사용)===
# 선택할 카메라 ID
CAMERA_ID = 4  # 1: 처음 쓴 카메라, 2: 교체형 - 소형 렌즈, 4: 교체형 - fisheye 렌즈

# 카메라 ID별 폴더 경로
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "camera"))

# 지원할 카메라 목록
SUPPORTED_CAMERA_IDS = [1, 2, 4]

# 설정 딕셔너리 생성
CAMERA_SETTINGS = {}
for cam_id in SUPPORTED_CAMERA_IDS:
    cam_dir = os.path.join(BASE_DIR, str(cam_id))
    # calibration 파일 경로
    matrix_path = os.path.join(cam_dir, "camera_matrix.npy")
    dist_path   = os.path.join(cam_dir, "dist_coeffs.npy")

    # numpy 로드
    cam_matrix  = np.load(matrix_path)
    dist_coeffs = np.load(dist_path)

    # 카메라 타입 지정 (fisheye vs normal)
    cam_type = 'fisheye' if cam_id == 4 else 'normal'
    image_size = (720, 1280)

    CAMERA_SETTINGS[cam_id] = {
        'type': cam_type,
        'matrix': cam_matrix,
        'dist': dist_coeffs,
        'size': image_size
    }
# 현재 선택된 카메라 설정 가져오기
camera_cfg = CAMERA_SETTINGS[CAMERA_ID]

#===카메라 매개변수 끝===

CAMERA_HEIGHT=148.5
ROBOT_HEIGHT=9.1
CORRECTION_COEF = ((CAMERA_HEIGHT-ROBOT_HEIGHT)/CAMERA_HEIGHT)
Dist_Correction_Coefficient = 1 #화면상의 거리와 실제거리 보정계수


# 보드 크기 (cm 단위)
board_width_cm = 90
board_height_cm = 90

board_margin=0

# 태그 정보
tag_size = 0.044  # 태그 크기 (단위: 미터)
tag_role = {      # 태그 ID와 역할 매핑
     NORTH_TAG_ID : "board",
}
object_points = np.array([
    [0, 0, 0],
    [tag_size, 0, 0],
    [tag_size, tag_size, 0],
    [0, tag_size, 0],
    [0, 0, tag_size],
    [tag_size, 0, tag_size],
    [tag_size, tag_size, tag_size],
    [0, tag_size, tag_size]
], dtype=np.float32)

# 격자 배열 생성
cell_size_cm = 15 # 격자 크기 (cm 단위)
grid_row = int((board_height_cm-board_margin) / cell_size_cm) # 세로 행 수
grid_col = int((board_width_cm-board_margin) / cell_size_cm) # 가로 열 수

# 격자 시각화 크기
cell_size = 50
grid_width = cell_size * grid_col
grid_height = cell_size * grid_row

# 트래커 관련련
tracker_dict={}

# 경로 시각화용 색상
COLORS = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
    (0, 0, 128), (128, 128, 0), (128, 0, 128), (0, 128, 128),
    (64, 0, 0), (0, 64, 0), (0, 0, 64), (64, 64, 0),
    (64, 0, 64), (0, 64, 64), (192, 0, 0), (0, 192, 0),
    (0, 0, 192), (192, 192, 0), (192, 0, 192), (0, 192, 192)
]


# 명령 수행 시간
MOTION_DURATIONS = {
    "Move": 1.0,     # 전진
    "Stop": 1.0,      # 대기
    "Rotate": 1.0,     # 우회전
}

