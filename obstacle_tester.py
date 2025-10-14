import os
import sys
import cv2
import numpy as np
from dataclasses import dataclass

from interface import grid_visual, slider_create, slider_value, draw_agent_points, draw_paths
from config import (
    grid_row, grid_col, cell_size, camera_cfg,
    CORRECTION_COEF, NORTH_TAG_ID
)
from vision.camera import camera_open, Undistorter
from vision.visionsystem import VisionSystem

@dataclass
class ObstacleParams:
    min_cell_black_ratio: float = 0.12
    min_blob_area_px: int = 600
    open_kernel: int = 3
    close_kernel: int = 5
    blur_ksize: int = 5

def detect_black_cells(warp_bgr: np.ndarray, rows: int, cols: int, p: ObstacleParams):
    if warp_bgr is None:
        return np.zeros((rows, cols), dtype=np.uint8), None
    h, w = warp_bgr.shape[:2]
    gray = cv2.cvtColor(warp_bgr, cv2.COLOR_BGR2GRAY)
    if p.blur_ksize > 1:
        gray = cv2.GaussianBlur(gray, (p.blur_ksize, p.blur_ksize), 0)
    th_adapt = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 21, 5)
    _, th_otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.bitwise_and(th_adapt, th_otsu)
    if p.open_kernel > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p.open_kernel, p.open_kernel)))
    if p.close_kernel > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p.close_kernel, p.close_kernel)))
    occ = np.zeros((rows, cols), dtype=np.uint8)
    cell_w = w / cols
    cell_h = h / rows
    for r in range(rows):
        for c in range(cols):
            x1 = int(c * cell_w)
            y1 = int(r * cell_h)
            x2 = int((c + 1) * cell_w)
            y2 = int((r + 1) * cell_h)
            roi = mask[y1:y2, x1:x2]
            if roi.size == 0:
                continue
            black_ratio = float(np.count_nonzero(roi)) / roi.size
            if black_ratio < p.min_cell_black_ratio:
                continue
            num, labels, stats, _ = cv2.connectedComponentsWithStats(roi, connectivity=8)
            if num <= 1:
                continue
            largest = stats[1:, cv2.CC_STAT_AREA].max()
            if largest >= p.min_blob_area_px:
                occ[r, c] = 1
    return occ, mask

# === 비전/카메라 ===
cap, fps = camera_open(source=None)
undistorter = Undistorter(
    camera_cfg['type'], camera_cfg['matrix'], camera_cfg['dist'], camera_cfg['size']
)
vision = VisionSystem(undistorter=undistorter, visualize=True)

blank_grid = np.zeros((grid_row, grid_col), dtype=np.int32)

cv2.namedWindow("Video_display", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Video_display", vision.mouse_callback)
cv2.namedWindow("CBS Grid")

cv2.namedWindow("CorrectionPanel", cv2.WINDOW_NORMAL)

slider_create()

cv2.namedWindow("ObstaclePanel", cv2.WINDOW_NORMAL)
cv2.createTrackbar("MinRatio%", "ObstaclePanel", 12, 100, lambda v: None)
cv2.createTrackbar("MinBlobArea", "ObstaclePanel", 600, 5000, lambda v: None)

print("[INFO] Controls: q=quit | r=reset | n=lock board | b=unlock | v=viz toggle | s=ROI select | m=mode switch(tag/contour) | o=toggle obstacle detect")

def _get_warped_from_vision(vision: VisionSystem, output):
    if hasattr(vision, "board_result") and vision.board_result is not None:
        if hasattr(vision.board_result, "warped_resized"):
            return vision.board_result.warped_resized
    return None


# === 루프 ===
obstacle_on = True
last_occ = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARN] 프레임 획득 실패")
        continue
    detect_params = slider_value()
    output = vision.process_frame(frame, detect_params)
    disp = frame
    if isinstance(output, dict):
        disp = output.get("frame", disp)
    warped = _get_warped_from_vision(vision, output)

    vis_grid = grid_visual(blank_grid.copy())

    if obstacle_on and warped is not None:
        params = ObstacleParams(
            min_cell_black_ratio=max(0.01, cv2.getTrackbarPos("MinRatio%", "ObstaclePanel") / 100.0),
            min_blob_area_px=max(0, cv2.getTrackbarPos("MinBlobArea", "ObstaclePanel"))
        )
        occ, mask = detect_black_cells(warped, grid_row, grid_col, params)
        last_occ = occ
        cell_h = vis_grid.shape[0] // grid_row
        cell_w = vis_grid.shape[1] // grid_col
        for r in range(grid_row):
            for c in range(grid_col):
                if occ[r, c] == 1:
                    y1, y2 = r * cell_h, (r + 1) * cell_h
                    x1, x2 = c * cell_w, (c + 1) * cell_w
                    cv2.rectangle(vis_grid, (x1, y1), (x2, y2), (0, 0, 255), thickness=-1)
        cv2.imshow("Warp (top-down)", warped)
        if mask is not None:
            cv2.imshow("BlackMask", mask)
    elif obstacle_on and warped is None:
        cv2.putText(disp, "No warped frame (press 'n' to lock, or VisionSystem lacks warped)",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2, cv2.LINE_AA)

    cv2.imshow("CBS Grid", vis_grid)
    cv2.imshow("Video_display", disp)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('n'):
        vision.lock_board()
        print("[INFO] 보드 고정됨")
    elif key == ord('b'):
        vision.reset_board()
        print("[INFO] 🔄 고정된 보드 해제")
    elif key == ord('v'):
        vision.toggle_visualization()
        print(f"[INFO] 시각화 모드: {'ON' if vision.visualize else 'OFF'}")
    elif key == ord('s'):
        vision.start_roi_selection()
    elif key == ord('o'):
        obstacle_on = not obstacle_on
        print(f"[INFO] Obstacle detection: {'ON' if obstacle_on else 'OFF'}")

cap.release()
cv2.destroyAllWindows()
