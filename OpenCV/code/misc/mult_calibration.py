import os, csv, re, time
from pathlib import Path
from datetime import datetime
import numpy as np
import cv2 as cv

# ===== 사용자 설정 =====
CALIB_DIR = Path(r"C:\Users\ubeau\calib_out")   # summary.csv가 있는 폴더
CSV_PATH  = CALIB_DIR / "summary.csv"
IMAGE_PATH = Path(r"C:\Users\ubeau\captures\cap_20250811_191153_000.jpg")  # ← 이 사진 1장
TOP_N     = None       # None=전부, 숫자=상위 N개만 적용
ALPHA     = 0.0        # 0=크롭↑(직선성↑), 1=FOV↑
# =======================

def log(m): print(m, flush=True)

def parse_rows(csv_path: Path):
    if not csv_path.exists():
        raise SystemExit(f"summary.csv가 없습니다: {csv_path}")
    rows = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        r = csv.DictReader(f)
        for row in r:
            for k in ["RMS(pixel)", "mean_per_view_err(pixel)"]:
                if k in row and row[k] != "": row[k] = float(row[k])
            if "num_images" in row and row["num_images"] != "": row["num_images"] = int(row["num_images"])
            for k in ["fx","fy","cx","cy"]:
                row[k] = float(row[k])
            d = row.get("distCoeffs_or_D","").strip()
            row["_dist"] = np.array([float(x) for x in d.split(";") if x!=""], dtype=np.float64)
            rows.append(row)
    rows.sort(key=lambda x: (x.get("RMS(pixel)", 1e9),
                             x.get("mean_per_view_err(pixel)", 1e9),
                             -x.get("num_images",0)))
    return rows

def detect_calib_size(calib_dir: Path, rows, fallback=(1280,720)):
    for row in rows:
        prev = row.get("preview","")
        p = (calib_dir / prev)
        if p.exists():
            img = cv.imread(str(p), cv.IMREAD_COLOR)
            if img is not None:
                h, w = img.shape[:2]
                return (w, h)
    return fallback

def scale_K(K, old_wh, new_wh):
    sx = new_wh[0] / old_wh[0]
    sy = new_wh[1] / old_wh[1]
    K2 = K.copy()
    K2[0,0] *= sx; K2[1,1] *= sy
    K2[0,2] *= sx; K2[1,2] *= sy
    return K2

def draw_grid(img, step=50, alpha=0.35):
    h, w = img.shape[:2]
    over = img.copy()
    for x in range(0, w, step): cv.line(over, (x,0), (x,h), (0,255,0), 1, cv.LINE_AA)
    for y in range(0, h, step): cv.line(over, (0,y), (w,y), (0,255,0), 1, cv.LINE_AA)
    return cv.addWeighted(over, alpha, img, 1-alpha, 0)

def sanitize(name):
    return re.sub(r"[^0-9A-Za-z._()+\-]+", "_", name)

def apply_all(rows, frame_bgr, calib_wh, out_dir: Path, alpha=0.0, top_n=None):
    out_dir.mkdir(parents=True, exist_ok=True)
    cap_res = (frame_bgr.shape[1], frame_bgr.shape[0])
    targets = rows if top_n is None else rows[:top_n]
    count = 0
    for row in targets:
        model = (row.get("model","pinhole") or "pinhole").strip().lower()
        det = row.get("detector","")
        cfg = row.get("config","")
        fx, fy, cx, cy = row["fx"], row["fy"], row["cx"], row["cy"]
        K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=np.float64)
        D = row["_dist"].reshape(-1,1).astype(np.float64)

        # 해상도 차이 있으면 K 스케일
        K_use = scale_K(K, calib_wh, cap_res) if cap_res != calib_wh else K

        if model == "fisheye":
            newK = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K_use, D, cap_res, np.eye(3), balance=alpha)
            map1, map2 = cv.fisheye.initUndistortRectifyMap(
                K_use, D, np.eye(3), newK, cap_res, cv.CV_16SC2)
            und = cv.remap(frame_bgr, map1, map2, interpolation=cv.INTER_LINEAR)
        else:
            newK, _ = cv.getOptimalNewCameraMatrix(K_use, D, cap_res, alpha=alpha)
            und = cv.undistort(frame_bgr, K_use, D, None, newK)

        fname = f"{sanitize(det)}__{sanitize(cfg)}__applied.jpg"
        cv.imwrite(str(out_dir/fname), draw_grid(und))
        count += 1
    return count

def main():
    # 1) 결과 로드
    rows = parse_rows(CSV_PATH)
    calib_wh = detect_calib_size(CALIB_DIR, rows)

    # 2) 테스트 이미지 로드
    img = cv.imread(str(IMAGE_PATH), cv.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"이미지를 열 수 없습니다: {IMAGE_PATH}")
    cap_res = (img.shape[1], img.shape[0])

    # 3) 출력 폴더
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = CALIB_DIR / f"applied_from_image_{ts}"
    out_dir.mkdir(parents=True, exist_ok=True)

    log(f"[APPLY] calib_size={calib_wh}, image_size={cap_res}, src={IMAGE_PATH}")
    cnt = apply_all(rows, img, calib_wh, out_dir, alpha=ALPHA, top_n=TOP_N)
    log(f"[DONE] saved {cnt} files under {out_dir}")

if __name__ == "__main__":
    main()
