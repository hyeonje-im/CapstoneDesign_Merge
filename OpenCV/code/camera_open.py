import cv2
import os
from pathlib import Path
from datetime import datetime
from vision.camera import camera_open  # vision/camera.py 참고

# ===== 사용자 설정 =====
SAVE_DIR = Path("captures")   # 저장 폴더
FNAME_PREFIX = "cap"          # 파일명 접두어
EXT = ".jpg"                  # ".jpg" 또는 ".png"
# =======================

def main():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    # 1번 카메라 우선(필요하면 source=None 으로 1→0 자동 시도)
    cap, fps = camera_open(source=1)
    print(f"[INFO] FPS: {fps}")

    counter = 0
    print("[INFO] q/ESC: 종료, c: 캡처 저장")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("[WARN] 프레임 읽기 실패")
            continue

        # 화면 안내 오버레이
        vis = frame.copy()
        cv2.putText(vis, "Press 'c' to capture, 'q'/ESC to quit",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
        cv2.imshow("Camera", vis)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # q 또는 ESC
            break
        elif key == ord('c'):
            # 파일명: cap_YYYYmmdd_HHMMSS_counter.jpg
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"{FNAME_PREFIX}_{ts}_{counter:03d}{EXT}"
            out_path = SAVE_DIR / fname
            ok = cv2.imwrite(str(out_path), frame)
            if ok:
                print(f"[SAVE] {out_path}")
                counter += 1
            else:
                print(f"[ERR] 저장 실패: {out_path}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
