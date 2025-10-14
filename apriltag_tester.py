# test_apriltag_roi_multiscale.py
import cv2
import numpy as np
from vision.camera import camera_open, Undistorter
from vision.apriltag import AprilTagDetector
from config import camera_cfg

# ROI 상태
roi_pts = []
roi_box = None
selecting_roi = False

def mouse_callback(event, x, y, flags, param):
    global roi_pts, roi_box, selecting_roi
    if selecting_roi and event == cv2.EVENT_LBUTTONDOWN:
        roi_pts.append((x, y))
        print(f"[ROI] Point {len(roi_pts)}: {(x,y)}")
        if len(roi_pts) == 2:
            (x1,y1), (x2,y2) = roi_pts
            x0, y0 = min(x1,x2), min(y1,y2)
            w, h    = abs(x2-x1), abs(y2-y1)
            roi_box = (x0, y0, w, h)
            selecting_roi = False
            print(f"[ROI] Defined: x={x0}, y={y0}, w={w}, h={h}")

def unsharp_mask(gray):
    blur = cv2.GaussianBlur(gray, (9,9), 10)
    return cv2.addWeighted(gray, 1.5, blur, -0.5, 0)

def apply_clahe(gray):
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    return clahe.apply(gray)

def preprocess_up(roi_bgr, scale=2):
    # 1) 업스케일
    up = cv2.resize(roi_bgr, None, fx=scale, fy=scale,
                    interpolation=cv2.INTER_CUBIC)
    # 2) 그레이
    gray = cv2.cvtColor(up, cv2.COLOR_BGR2GRAY)
    # 3) Unsharp Mask
    sharp = unsharp_mask(gray)
    # 4) CLAHE
    cl = apply_clahe(sharp)
    # 5) Bilateral Filter (노이즈↓, 에지↗)
    return cv2.bilateralFilter(cl, d=9, sigmaColor=75, sigmaSpace=75)

def main():
    global selecting_roi, roi_pts, roi_box

    cap, fps = camera_open()
    print(f"[INFO] Camera opened, FPS: {fps}")

    undistorter = Undistorter(
        camera_cfg['type'], camera_cfg['matrix'],
        camera_cfg['dist'], camera_cfg['size']
    )
    detector   = AprilTagDetector()
    frame_cnt  = 0

    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret: continue
        frame_cnt += 1

        # 1) 왜곡 보정
        frame_u, new_cam = undistorter.undistort(frame)

        # 2) 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            selecting_roi = True
            roi_pts, roi_box = [], None
            print("[INFO] ROI selection ON")
        elif key == ord('q'):
            break

        # 3) 전체 프레임 샤프닝(언샤프)
        gray_full  = cv2.cvtColor(frame_u, cv2.COLOR_BGR2GRAY)
        sharp_full = unsharp_mask(gray_full)
        proc       = cv2.cvtColor(sharp_full, cv2.COLOR_GRAY2BGR)

        # 4) ROI 처리 (업스케일 + 전처리 → 디텍션 → 매핑)
        if roi_box:
            x,y,w,h = roi_box
            roi      = proc[y:y+h, x:x+w]
            proc_up  = preprocess_up(roi, scale=3)

            # AprilTag 검출 (업스케일 이미지에서)
            tags_up = detector.detect(proc_up)
            for tag in tags_up:
                # tag.corners 는 업스케일 좌표 → 원본 ROI 좌표로 리매핑
                corners_up = tag.corners / 2.0  
                corners    = (corners_up + np.array([x,y])).astype(int)
                cv2.polylines(proc, [corners.reshape(-1,1,2)],
                              isClosed=True, color=(0,255,0), thickness=2)
                # 중심점
                c = np.mean(corners, axis=0).astype(int)
                cv2.putText(proc, f"ID:{tag.tag_id}",
                            tuple(c), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0,255,0), 2)
            # ROI 박스 표시
            cv2.rectangle(proc, (x,y), (x+w,y+h),
                          (0,255,255), 2)

        else:
            # ROI 미지정 시, 전체 프레임에서 디텍션
            tags = detector.detect(cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY))
            detector.update(tags, frame_cnt, new_cam)
            for tid, info in detector.update_tag_info().items():
                if "corners" in info:
                    pts = info["corners"].astype(int).reshape(-1,1,2)
                    cv2.polylines(proc, [pts], True, (0,255,0), 2)
                    cx,cy = map(int, info["corrected_center"])
                    cv2.putText(proc, f"ID:{tid}", (cx,cy-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # 5) 화면 출력
        cv2.imshow("Frame", proc)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
