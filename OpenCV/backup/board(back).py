import cv2
import numpy as np
from dataclasses import dataclass

@dataclass
class BoardDetectionResult:
    corners: np.ndarray
    origin: np.ndarray
    width_px: float
    height_px: float
    cm_per_px: tuple
    perspective_matrix: np.ndarray
    warped: np.ndarray
    warped_resized: np.ndarray
    found: bool = True

def board_detect(gray):
    brightness_threshold, min_aspect_ratio, max_aspect_ratio = slider_value()
    _, thresh = cv2.threshold(gray, brightness_threshold, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_rect = None
    largest_area = 0

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                
        if len(approx) == 4:
            area = cv2.contourArea(approx)
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            if area > 2000 and min_aspect_ratio < aspect_ratio < max_aspect_ratio:
                largest_area = area
                largest_rect = approx

    return largest_rect


def draw(frame, largest_rect):
    cv2.drawContours(frame, [largest_rect], -1, (255, 0, 0), 3)
        
def board_pts(largest_rect):
    pts = largest_rect.reshape(4, 2).astype(np.float32)
    sum_pts = pts.sum(axis=1)
    diff_pts = np.diff(pts, axis=1)
    top_left = pts[np.argmin(sum_pts)]
    bottom_right = pts[np.argmax(sum_pts)]
    top_right = pts[np.argmin(diff_pts)]
    bottom_left = pts[np.argmax(diff_pts)]
    
    board_width_px = np.linalg.norm(top_right - top_left)
    board_height_px = np.linalg.norm(bottom_left - top_left)

    return np.array([top_left, top_right, bottom_right, bottom_left]), board_width_px, board_height_px


def perspective_transform(frame, rect, board_width_px, board_height_px):
    dst = np.array([[0, 0], [board_width_px - 1, 0], 
                    [board_width_px - 1, board_height_px - 1], [0, board_height_px - 1]], dtype="float32")
    matrix = cv2.getPerspectiveTransform(rect, dst)
    warped=cv2.warpPerspective(frame, matrix, (int(board_width_px), int(board_height_px)))
    
    warped_board_width_px = int(np.linalg.norm(dst[1] - dst[0]))  # (top_right - top_left)
    warped_board_height_px = int(np.linalg.norm(dst[3] - dst[0])) # (bottom_left - top_left)
    
    warped_resized = cv2.resize(warped, (frame.shape[1] // 2, frame.shape[1] // 2))
    
    return warped, warped_board_width_px, warped_board_height_px, warped_resized

def board_origin(frame, top_left):
    board_origin_tvec = np.array([top_left[0], top_left[1], 0], dtype=np.float32)
    cv2.circle(frame, tuple(board_origin_tvec[:2].astype(int)), 5, (0, 0, 255), -1)
    
    return board_origin_tvec
