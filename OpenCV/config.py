import numpy as np
import os

# # 카메라 매개변수 (캘리브레이션된 값 사용)
# camera_matrix = np.array([
#     [1.44881455e+03, 0, 9.80488323e+02],
#     [0, 1.45151609e+03, 5.39528675e+02],
#     [0, 0, 1.00000000e+00]
# ], dtype=np.float32)

# dist_coeffs = np.array([
#     [3.75921025e-03, 1.02703292e-01, -7.06313415e-05, 1.59368677e-03, -2.21477882e-01]
# ])

CAMERA_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "camera"))

camera_matrix_path = os.path.join(CAMERA_DIR, "camera_matrix.npy")
dist_coeffs_path = os.path.join(CAMERA_DIR, "dist_coeffs.npy")

camera_matrix = np.load(camera_matrix_path)
dist_coeffs = np.load(dist_coeffs_path)

# 보드 크기 (cm 단위)
board_width_cm = 60
board_height_cm = 60

# 3D 상자 정의
tag_size = 0.05  # 태그 크기 (단위: 미터)
object_points = np.array([
    [0, 0, 0], [tag_size, 0, 0], [tag_size, tag_size, 0], [0, tag_size, 0],  # 태그 평면
    [0, 0, tag_size], [tag_size, 0, tag_size], [tag_size, tag_size, tag_size], [0, tag_size, tag_size]  # 상단 꼭짓점
], dtype=np.float32)

# 태그 상태 관리
tag_info = {}
detected_ids = set()

# 격자 배열 생성
cell_size_cm = 10 # 격자 크기 (cm 단위)
grid_row = int(board_height_cm / cell_size_cm) # 세로 행 수
grid_col = int(board_width_cm / cell_size_cm) # 가로 열 수

# 격자 시각화 크기기
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