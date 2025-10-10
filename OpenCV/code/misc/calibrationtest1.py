#!/usr/bin/env python3
"""
Wide-angle calibration viewer (uniform 2x2 layout)
==================================================

* Rational vs Fisheye 보정 결과를 공정하게 비교:
  - 좌상: Rational Full
  - 좌하: Rational Cropped (ROI)
  - 우상: Fisheye Full
  - 우하: Fisheye 중앙 Crop

* 모든 이미지를 동일 크기로 맞춤 (작으면 회색 패딩)
* A/D 키로 이미지 넘김, Q/ESC 종료
"""

import cv2
import numpy as np
import glob
import os

# === 사용자 설정 ===
IMG_PATH     = r"D:\git\Capstone_Design-OpenCV\img\calibration\3-2\*.jpg"
CHECKERBOARD = (10, 7)
SQUARE_SIZE  = 0.025
CACHE_FILE   = "calib_cache.npz"
VIEW_WIDTH   = 700  # 각 패널 너비
VIEW_HEIGHT  = 500  # 각 패널 높이

# === 보정 결과 (로드 또는 캘리브레이션) ===
def calibrate(images):
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints, imgpoints = [], []

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print(f"[!] 코너 탐지 실패: {fname}")

    img_size = gray.shape[::-1]

    flags_r = cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
    rms_r, K_r, D_r, *_ = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None, flags=flags_r)

    objpoints_f = [o.reshape(-1,1,3) for o in objpoints]
    imgpoints_f = [i.reshape(-1,1,2) for i in imgpoints]
    K_f = np.zeros((3,3)); D_f = np.zeros((4,1))
    flags_f = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    rms_f, K_f, D_f, *_ = cv2.fisheye.calibrate(
        objpoints_f, imgpoints_f, img_size, K_f, D_f, None, None, flags=flags_f,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))

    np.savez_compressed(CACHE_FILE, K_r=K_r, D_r=D_r, rms_r=rms_r,
                        K_f=K_f, D_f=D_f, rms_f=rms_f, img_size=img_size)
    print("[√] Calibration 저장 완료")


def load_calibration():
    if not os.path.exists(CACHE_FILE):
        calibrate(glob.glob(IMG_PATH))
    return np.load(CACHE_FILE)


def pad_and_resize(image, size=(VIEW_HEIGHT, VIEW_WIDTH)):
    h, w = image.shape[:2]
    scale = min(size[1]/w, size[0]/h)
    resized = cv2.resize(image, (int(w*scale), int(h*scale)))
    canvas = np.full((size[0], size[1], 3), 160, dtype=np.uint8)
    ry, rx = resized.shape[:2]
    y0 = (size[0] - ry) // 2
    x0 = (size[1] - rx) // 2
    canvas[y0:y0+ry, x0:x0+rx] = resized
    return canvas


def generate_grid_view(img, K_r, D_r, K_f, D_f, balance):
    h, w = img.shape[:2]

    # Rational full & cropped (고정)
    newK_r_full, _ = cv2.getOptimalNewCameraMatrix(K_r, D_r, (w, h), 1)
    rational_full = cv2.undistort(img, K_r, D_r, None, newK_r_full)

    newK_r_crop, roi = cv2.getOptimalNewCameraMatrix(K_r, D_r, (w, h), 0)
    rational_crop = cv2.undistort(img, K_r, D_r, None, newK_r_crop)
    x, y, rw, rh = roi
    rational_crop = rational_crop[y:y+rh, x:x+rw]

    # Fisheye with adjustable balance
    newK_f = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K_f, D_f, (w, h), np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K_f, D_f, np.eye(3), newK_f, (w, h), cv2.CV_16SC2)
    fisheye_full = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

    pad = int(0.1 * h)
    fisheye_crop = fisheye_full[pad:-pad, pad:-pad]

    # Resize & pad
    rf = pad_and_resize(rational_full)
    rc = pad_and_resize(rational_crop)
    ff = pad_and_resize(fisheye_full)
    fc = pad_and_resize(fisheye_crop)

    left = np.vstack([rf, rc])
    right = np.vstack([ff, fc])
    return np.hstack([left, right])




def main():
    data = load_calibration()
    K_r, D_r, rms_r = data["K_r"], data["D_r"], data["rms_r"]
    K_f, D_f, rms_f = data["K_f"], data["D_f"], data["rms_f"]

    images = sorted(glob.glob(IMG_PATH))
    idx, N = 0, len(images)

    window_name = "Calibration Comparison (2x2)"
    cv2.namedWindow(window_name)

    # 슬라이더 생성: 0~100 → balance 0.00~1.00
    def nothing(x): pass
    cv2.createTrackbar("Fisheye Balance", window_name, 20, 100, nothing)

    while True:
        img = cv2.imread(images[idx])
        balance_val = cv2.getTrackbarPos("Fisheye Balance", window_name) / 100.0
        stacked = generate_grid_view(img, K_r, D_r, K_f, D_f, balance=balance_val)

        name = os.path.basename(images[idx])
        title = f"{name} ({idx+1}/{N}) | Rational RMS={rms_r:.3f} | Fisheye RMS={rms_f:.3f} | Balance={balance_val:.2f}"
        cv2.imshow(window_name, stacked)
        key = cv2.waitKey(50) & 0xFF

        if key in [ord('q'), 27]:
            break
        elif key == ord('a'):
            idx = (idx - 1) % N
        elif key == ord('d'):
            idx = (idx + 1) % N

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
