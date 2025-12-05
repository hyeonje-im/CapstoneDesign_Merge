# #!/usr/bin/env python3
# """
# Wide‑angle camera calibration script
# ------------------------------------

# * 최적화 대상 : 대각 FOV ≈ 120–140° 광각(예: 2.8 mm @ 1/2.8" ≈ 134°)
# * 체커보드 또는 ChArUco 보드에 모두 적용 가능 (현재는 체커보드 가정)

# Usage 예시 (더 이상 사용하지 않음)
# ~~~~~~~~~~
# 명령줄 인자 대신 코드 상단에서 경로와 체커보드 설정을 지정합니다.
# """

# import cv2
# import numpy as np
# import glob
# import os

# # === 사용자 설정 ===
# IMG_PATH = r"D:\git\Capstone_Design-OpenCV\img\calibration\3-2\*.jpg"
# CHECKERBOARD = (10, 7)                # 내부 코너 수 (실제 격자는 11x8)
# SQUARE_SIZE  = 0.025                  # 한 칸 크기 (미터 단위, 예: 25mm → 0.025)

# # === 코너 객체 정의 ===
# objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
# objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
# objp *= SQUARE_SIZE

# objpoints = []  # 3D points
# imgpoints = []  # 2D points

# # === 이미지 불러오기 ===
# images = glob.glob(IMG_PATH)
# assert len(images) > 0, "이미지를 찾을 수 없습니다. IMG_PATH 경로를 확인하세요."

# for fname in images:
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     # 코너 탐지 (향상된 SB 버전)
#     ret, corners = cv2.findChessboardCornersSB(
#         gray, CHECKERBOARD,
#         cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)

#     if ret:
#         objpoints.append(objp)
#         imgpoints.append(corners)

#         # 시각화
#         cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
#         cv2.imshow('Corners', img)
#         cv2.waitKey(100)
#     else:
#         print(f"[!] 코너 탐지 실패: {fname}")

# cv2.destroyAllWindows()

# # === 캘리브레이션 ===
# flags  = cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_ASPECT_RATIO
# flags |= cv2.CALIB_ZERO_TANGENT_DIST  # 필요에 따라 제거 가능

# ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints, imgpoints, gray.shape[::-1], None, None, flags=flags)

# print("\n▶︎ 캘리브레이션 완료")
# print("▶︎ 카메라 행렬 (K):\n", K)
# print("▶︎ 왜곡 계수 (D):\n", D.ravel())

# # === RMS 및 재투영 오차 측정 ===
# mean_error = 0
# for i in range(len(objpoints)):
#     imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
#     error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
#     mean_error += error

# print(f"▶︎ RMS reprojection error: {ret:.4f} px")
# print(f"▶︎ Mean reprojection error: {mean_error / len(objpoints):.4f} px")


#!/usr/bin/env python3
"""
Wide‑angle camera calibration script with image viewer
------------------------------------------------------

* 광각 보정 결과를 확인하면서 'A', 'D' 키로 다음/이전 이미지 탐색
* 'Q' 또는 ESC 키로 종료

이미지 폴더 설정은 IMG_PATH에서 *.jpg 또는 *.png 등으로 지정
"""

#!/usr/bin/env python3
"""
Wide‑angle camera calibration viewer
-------------------------------------

* 한 이미지를 두 방식으로 보정:
  1) ROI 크롭: 왜곡 제거 후 유효한 영역만 표시
  2) Full view: 전체 이미지 보정 (검은 여백 포함)

* A/D 키로 이미지 넘김, Q/ESC로 종료
"""

import cv2
import numpy as np
import glob
import os

# === 사용자 설정 ===
IMG_PATH     = r"D:\git\Capstone_Design-OpenCV\img\calibration\3-2\*.jpg"
CHECKERBOARD = (10, 7)
SQUARE_SIZE  = 0.025

# === 보정 결과 입력 ===
K = np.array([[1018.97213, 0, 977.311765],
              [0, 1018.97213, 521.850957],
              [0, 0, 1]])
D = np.array([1.36108715, -0.65238083, 0, 0, -0.1031927, 
              1.74353897, -0.29850331, -0.35689505, 
              0, 0, 0, 0, 0, 0])

# === 이미지 목록 ===
images = sorted(glob.glob(IMG_PATH))
assert images, "이미지를 찾을 수 없습니다. IMG_PATH 확인."

index = 0
N = len(images)

while True:
    path = images[index % N]
    img = cv2.imread(path)
    h, w = img.shape[:2]

    # 전체 이미지 보정 (검은 영역 포함)
    new_K_full, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    full_view = cv2.undistort(img, K, D, None, new_K_full)

    # ROI만 잘라낸 보정 결과
    new_K_crop, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    cropped = cv2.undistort(img, K, D, None, new_K_crop)
    x, y, rw, rh = roi
    cropped = cropped[y:y+rh, x:x+rw]

    # 크기 통일 및 위아래 결합
    target_width = min(full_view.shape[1], cropped.shape[1])
    scale = 800 / target_width
    full_resized = cv2.resize(full_view, None, fx=scale, fy=scale)
    crop_resized = cv2.resize(cropped, (full_resized.shape[1], full_resized.shape[0]))
    stacked = np.vstack([full_resized, crop_resized])

    # 창 이름 표시
    name = os.path.basename(path)
    cv2.imshow(f"[Full vs Cropped] {name} ({index+1}/{N})", stacked)

    key = cv2.waitKey(0) & 0xFF
    if key in [ord('q'), 27]:
        break
    elif key == ord('a'):
        index = (index - 1) % N
    elif key == ord('d'):
        index = (index + 1) % N

    cv2.destroyAllWindows()

cv2.destroyAllWindows()
