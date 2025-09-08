# obstacle.py
# - BoardDetectionResult(warped, cm_per_px, grid_reference)를 받아
#   각 셀 중심 ROI의 "이진화(명암) → 검정 비율"로 즉시 판정(신뢰도 누적 없음)
# - 원본 프레임 오버레이 + 워프 디버그 제공
# - arm_autosave(): 다음 update 시 자동으로 grid/MMDDgrid.json 저장

import cv2
import numpy as np
import os, time, json
from typing import Optional, Tuple, List

class ObstacleDetector:
    def __init__(
        self,
        grid_rows: int,
        grid_cols: int,
        block_size_cm: float = 10.0,   # 블록 윗면 한 변(cm)
        thresh_gray: int = 120,         # gray <= T → 검정
        roi_scale: float = 1.3,        # ROI 스케일(1.0~3.0)
        min_black_ratio: float = 0.50  # ROI 내 검정 비율 임계
    ):
        self.grid_rows = int(grid_rows)
        self.grid_cols = int(grid_cols)

        self.block_size_cm    = float(block_size_cm)
        self.thresh_gray      = int(thresh_gray)
        self.roi_scale        = float(roi_scale)
        self.min_black_ratio  = float(min_black_ratio)

        self.last_occupancy: Optional[np.ndarray] = None   # (rows, cols) bool
        self.last_debug_warped: Optional[np.ndarray] = None

        self.last_saved_path: Optional[str] = None

    # -------- Parameter setters --------
    def set_threshold(self, thresh_gray: int): self.thresh_gray = int(thresh_gray)
    def set_roi_scale(self, roi_scale: float): self.roi_scale = float(roi_scale)
    def set_min_black_ratio(self, ratio: float): self.min_black_ratio = float(ratio)
    def set_block_size_cm(self, size_cm: float): self.block_size_cm = float(size_cm)

    # # -------- Autosave control --------
    # def arm_autosave(self, save_dir: str = "grid", filename: Optional[str] = None):
    #     """다음 update_from_board에서 유효 결과가 나오면 1회 저장"""
    #     self._autosave_req = (save_dir, filename)

    # -------- Utils --------
    @staticmethod
    def _clamp(v, a, b): return max(a, min(b, v))

    @staticmethod
    def _crop(img, cx, cy, hw, hh):
        H, W = img.shape[:2]
        x1 = int(max(0, np.floor(cx - hw))); x2 = int(min(W, np.ceil (cx + hw)))
        y1 = int(max(0, np.floor(cy - hh))); y2 = int(min(H, np.ceil (cy + hh)))
        if x2 <= x1 or y2 <= y1: return None, (0,0,0,0)
        return img[y1:y2, x1:x2], (x1,y1,x2,y2)

    @staticmethod
    def _cm_to_warp_px(cm_per_px_xy, Xcm, Ycm):
        cpx_x, cpx_y = cm_per_px_xy
        return float(Xcm / max(cpx_x,1e-6)), float(Ycm / max(cpx_y,1e-6))

    # -------- Core: update from board --------
    def update_from_board(self, board_result) -> Optional[np.ndarray]:
        """
        board_result: BoardDetectionResult
          - warped(gray), cm_per_px(tuple), grid_reference(dict: H_metric, cell_centers)
        반환: (rows, cols) bool 즉시 판정 맵. None이면 아직 판단 불가.
        """
        self.last_saved_path = None

        if board_result is None or board_result.warped is None:
            self.last_occupancy = None
            self.last_debug_warped = None
            return None

        warped_gray = (cv2.cvtColor(board_result.warped, cv2.COLOR_BGR2GRAY)
                       if board_result.warped.ndim == 3 else board_result.warped)
        cm_per_px   = board_result.cm_per_px
        grid_ref    = getattr(board_result, "grid_reference", None)

        warped_bgr = cv2.cvtColor(warped_gray, cv2.COLOR_GRAY2BGR)

        # 셀 중심 (가능하면 grid_reference 사용)
        if grid_ref is not None and "cell_centers" in grid_ref:
            centers_cm = grid_ref["cell_centers"]
        else:
            # grid_ref 없으면 균등분할(권장X)
            w_cm = cm_per_px[0] * warped_gray.shape[1]
            h_cm = cm_per_px[1] * warped_gray.shape[0]
            cw = w_cm / self.grid_cols
            ch = h_cm / self.grid_rows
            centers_cm = [((c+0.5)*cw, (r+0.5)*ch)
                          for r in range(self.grid_rows)
                          for c in range(self.grid_cols)]

        # ROI 크기(픽셀) : 10cm 환산 × 스케일
        blk_w_px = max(1.0, self.block_size_cm / max(cm_per_px[0], 1e-6))
        blk_h_px = max(1.0, self.block_size_cm / max(cm_per_px[1], 1e-6))
        roi_hw = 0.5 * blk_w_px * self._clamp(self.roi_scale, 0.5, 4.0)
        roi_hh = 0.5 * blk_h_px * self._clamp(self.roi_scale, 0.5, 4.0)

        occ = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        k = 0
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                Xcm, Ycm = centers_cm[k]; k += 1
                cx, cy = self._cm_to_warp_px(cm_per_px, Xcm, Ycm)
                roi, rect = self._crop(warped_gray, cx, cy, roi_hw, roi_hh)
                if roi is None or roi.size == 0:
                    occ[r, c] = False
                    continue

                # 단순 이진화 → 검정 비율
                _, mask = cv2.threshold(roi, int(self.thresh_gray), 255, cv2.THRESH_BINARY_INV)
                mask = cv2.medianBlur(mask, 3)
                black_ratio = float(np.count_nonzero(mask)) / float(mask.size)
                occ[r, c] = (black_ratio >= self.min_black_ratio)

                # 디버그 표기
                x1,y1,x2,y2 = rect
                color = (0, 0, 255) if occ[r,c] else (90,90,90)
                cv2.rectangle(warped_bgr, (x1,y1), (x2,y2), color, 1)
                cv2.putText(warped_bgr, f"{black_ratio:.2f}",
                            (int(cx)+6, int(cy)-6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0,255,0) if occ[r,c] else (150,150,150), 1, cv2.LINE_AA)
                cv2.circle(warped_bgr, (int(cx), int(cy)), 5,
                           (0,255,0) if occ[r,c] else (130,130,130), 2)

        self.last_occupancy = occ
        self.last_debug_warped = warped_bgr

        return occ

    def get_debug_warped(self) -> Optional[np.ndarray]:
        return self.last_debug_warped

    # -------- Save grid (JSON) --------
    def get_grid_payload(self):
        """
        저장 없이도 기존 JSON 파일과 동일한 딕셔너리 형태를 반환.
        예: {"grid": [[0,1,...], ...]}
        """
        grid_int = self.get_grid_int()  # [[0/1,...], ...]
        if grid_int is None:
            return None
        return {"grid": grid_int}

    def get_grid_json_str(self) -> Optional[str]:
        """
        파일로 저장하지 않고 JSON 문자열만 생성(프린트/소켓 전송용).
        """
        payload = self.get_grid_payload()
        if payload is None:
            return None
        import json
        return json.dumps(payload, ensure_ascii=False)
    
    def get_grid_int(self) -> Optional[List[List[int]]]:
        if self.last_occupancy is None:
            return None
        return self.last_occupancy.astype(int).tolist()

    def save_grid(self, save_dir: str = "grid", filename: Optional[str] = None) -> Optional[str]:
        """
        {"grid": [[...],[...],...]} 형식으로 저장.
        filename 없으면 MMDDgrid.json 생성(예: 0828grid.json)
        """
        grid_int = self.get_grid_int()
        if grid_int is None:
            return None

        os.makedirs(save_dir, exist_ok=True)
        if not filename:
            filename = time.strftime("%m%d") + "grid.json"
        path = os.path.join(save_dir, filename)

        with open(path, "w", encoding="utf-8") as f:
            json.dump({"grid": grid_int}, f, ensure_ascii=False, indent=2)
        return path

    # -------- Reset --------
    def reset(self):
        self.last_occupancy = None
        self.last_debug_warped = None