#!/usr/bin/env python3
"""
Fisheye Calibration Script (Resized to 1280x720)
==================================================

* 광각 카메라를 fisheye 모델로 캘리브레이션 (balance=0)
* 기존 고해상도 이미지들을 1280x720으로 리사이즈해서 보정
* 결과를 .npz로 저장 (K, D는 1280x720 해상도 기준)
"""

import cv2
import numpy as np
import glob
import os

# === 사용자 설정 ===
IMG_PATH     = r"D:\git\Capstone_Design-OpenCV\img\calibration\3-2\*.jpg"
CHECKERBOARD = (10, 7)
SQUARE_SIZE  = 0.025
CACHE_FILE   = "fisheye_calib_1280x720.npz"
RESIZE_DIM   = (1280, 720)  # ⬅ 원하는 해상도로 리사이즈
SHOW         = True

def calibrate_fisheye(images):
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 1, 3), np.float32)
    objp[:, 0, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints, imgpoints = [], []
    used_images = []

    for fname in images:
        img = cv2.imread(fname)
        img = cv2.resize(img, RESIZE_DIM, interpolation=cv2.INTER_AREA)  # ⬅ 리사이즈 적용
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCornersSB(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners.reshape(-1,1,2))
            used_images.append((fname, img.copy(), corners))

            if SHOW:
                cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
                cv2.imshow("Corners", img)
                cv2.waitKey(100)
        else:
            print(f"[!] 코너 탐지 실패: {fname}")

    cv2.destroyAllWindows()
    assert objpoints, "[X] 유효한 코너를 찾지 못했습니다."

    image_size = RESIZE_DIM[::-1]  # (width, height) → (cols, rows)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))

    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, image_size, K, D, None, None,
        flags=flags, criteria=criteria)

    print("\n[✓] Fisheye calibration 완료")
    print(f"RMS reprojection error: {rms:.4f} px")
    print("카메라 행렬 (K):\n", K)
    print("왜곡 계수 (D):\n", D.ravel())

    # === 재투영 오차 계산 ===
    total_error = 0
    for i, (fname, img, corners) in enumerate(used_images):
        proj, _ = cv2.fisheye.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        error = cv2.norm(corners, proj, cv2.NORM_L2) / len(proj)
        total_error += error
        print(f" - {os.path.basename(fname)}: reprojection error = {error:.4f} px")

    mean_error = total_error / len(objpoints)
    print(f"\n[📐] Mean reprojection error (custom calc): {mean_error:.4f} px")

    np.savez_compressed(CACHE_FILE, K=K, D=D, rms=rms, size=image_size)
    print(f"[📁] 결과 저장됨 → {CACHE_FILE}")


if __name__ == "__main__":
    images = sorted(glob.glob(IMG_PATH))
    calibrate_fisheye(images)
