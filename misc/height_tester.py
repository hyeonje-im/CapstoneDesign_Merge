import cv2
import numpy as np
import math
from config import ROBOT_HEIGHT, CORRECTION_COEF
from vision.camera import camera_open
from vision.apriltag import Detector

class DistanceCorrectedTagDetector:
    """
    Detect AprilTags and draw markers:
      - Green polygon: original tag corners
      - Red dot: original tag center
      - Blue dot: corrected tag center (r * coef)
    Display coef slider value and computed camera height in controls window.
    """
    def __init__(self):
        self.detector = Detector(families="tag36h11")
        self.max_distance = None
        self.robot_height = ROBOT_HEIGHT
        # current coef (slider-driven)
        self.current_coef = CORRECTION_COEF

    def get_height(self):
        # Compute camera height from coef: height = robot_height / (1 - coef)
        coef = self.current_coef
        if coef < 1.0:
            return self.robot_height / (1.0 - coef)
        return float('inf')

    def correct_position(self, x, y, Cx, Cy):
        coef = self.current_coef
        dx = x - Cx
        dy = y - Cy
        r = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy, dx)
        r_prime = r * coef
        X_prime = Cx + r_prime * math.cos(theta)
        Y_prime = Cy + r_prime * math.sin(theta)
        return X_prime, Y_prime

    def detect_and_draw(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)
        h, w = frame.shape[:2]
        Cx, Cy = w / 2, h / 2
        if self.max_distance is None:
            self.max_distance = math.hypot(Cx, Cy)

        for tag in tags:
            # draw original polygon
            corners = np.array(tag.corners, dtype=np.int32)
            cv2.polylines(frame, [corners.reshape(-1,1,2)], True, (0,255,0), 2)
            # original center (red)
            ox, oy = tag.center
            cv2.circle(frame, (int(ox), int(oy)), 5, (0,0,255), -1)
            # corrected center (blue)
            cx, cy = self.correct_position(ox, oy, Cx, Cy)
            cv2.circle(frame, (int(cx), int(cy)), 5, (255,0,0), -1)

        # overlay text on frame
        coef = self.current_coef
        height = self.get_height()
        txt_c = f"Coef: {coef:.3f}"
        if height != float('inf'):
            txt_h = f"Height: {height:.1f} cm"
        else:
            txt_h = "Height: inf"
        cv2.putText(frame, txt_c, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(frame, txt_h, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        return frame


def main():
    detector = DistanceCorrectedTagDetector()

    ctrl_win = "Controls"
    cv2.namedWindow(ctrl_win)
    # only Coef slider: 0~1.0 mapped to 0~1000
    cv2.createTrackbar("Coef x1000", ctrl_win, int(detector.current_coef * 1000), 1000,
                       lambda v: setattr(detector, 'current_coef', v / 1000.0))

    cap, fps = camera_open()
    print(f"[INFO] Camera opened at {fps:.2f} FPS")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        output = detector.detect_and_draw(frame)
        cv2.imshow("AprilTag Detection", output)

        # build controls display with coef & height
        ctrl_img = np.zeros((100, 400, 3), dtype=np.uint8)
        coef = detector.current_coef
        height = detector.get_height()
        cv2.putText(ctrl_img, f"Coef: {coef:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        if height != float('inf'):
            cv2.putText(ctrl_img, f"Height: {height:.1f} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        else:
            cv2.putText(ctrl_img, "Height: inf", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.imshow(ctrl_win, ctrl_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
