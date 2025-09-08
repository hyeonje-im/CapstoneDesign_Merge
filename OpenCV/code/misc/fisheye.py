import cv2
import numpy as np
import os

# === 사용자 설정 ===
CHECKERBOARD = (10, 7)
SQUARE_SIZE  = 0.025
CACHE_FILE   = "calib_cache.npz"
VIEW_WIDTH   = 640
VIEW_HEIGHT  = 480

def load_calibration():
    if not os.path.exists(CACHE_FILE):
        print("[!] 보정 캐시가 없습니다. 보정 이미지를 통한 캘리브레이션 먼저 수행 필요.")
        exit(1)
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

def overlay_grid(image, rows=10, cols=10, color=(0, 0, 255), alpha=0.3):
    """영상 위에 반투명 격자 그리기"""
    overlay = image.copy()
    h, w = image.shape[:2]
    
    # 수직선
    for i in range(1, cols):
        x = int(w * i / cols)
        cv2.line(overlay, (x, 0), (x, h), color, 1)
    
    # 수평선
    for i in range(1, rows):
        y = int(h * i / rows)
        cv2.line(overlay, (0, y), (w, y), color, 1)

    return cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

def main():
    data = load_calibration()
    K_f, D_f, rms_f = data["K_f"], data["D_f"], data["rms_f"]
    alt_data = np.load("fisheye_calib_balance0.npz")
    K_alt, D_alt = alt_data["K"], alt_data["D"]
    window_name = "Fisheye Undistortion View"
    cv2.namedWindow(window_name)

    cap = cv2.VideoCapture(1)  # 카메라 인덱스 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("[!] 카메라 열기 실패")
        return

    cv2.createTrackbar("Fisheye Balance", window_name, 20, 100, lambda x: None)

    last_balance_val = -1.0
    map1, map2 = None, None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[!] 프레임 수신 실패")
            break

        h, w = frame.shape[:2]
        balance_val = cv2.getTrackbarPos("Fisheye Balance", window_name) / 100.0

        if balance_val != last_balance_val:
            newK_f = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_f, D_f, (w, h), np.eye(3), balance=balance_val)
            map1_f, map2_f = cv2.fisheye.initUndistortRectifyMap(K_f, D_f, np.eye(3), newK_f, (w, h), cv2.CV_16SC2)

            newK_alt = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_alt, D_alt, (w, h), np.eye(3), balance=balance_val)
            map1_alt, map2_alt = cv2.fisheye.initUndistortRectifyMap(K_alt, D_alt, np.eye(3), newK_alt, (w, h), cv2.CV_16SC2)

            last_balance_val = balance_val

        # 두 보정 결과 생성
        undist_f = cv2.remap(frame, map1_f, map2_f, interpolation=cv2.INTER_LINEAR)
        undist_alt = cv2.remap(frame, map1_alt, map2_alt, interpolation=cv2.INTER_LINEAR)

        # 크기 맞추고 나란히 결합
        view1 = pad_and_resize(undist_f)
        view2 = pad_and_resize(undist_alt)
        comparison = np.hstack((view1, view2))

        # 텍스트 오버레이
        cv2.putText(comparison, "calib_cache.npz", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(comparison, "fisheye_calib_balance0.npz", (VIEW_WIDTH + 30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow(window_name, comparison)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
