import cv2
import numpy as np
import os
import json
from interface import mouse_callback, grid_visual
from config import grid_row, grid_col

GRID_FOLDER = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "grid"))

# 격자 배열을 저장할 JSON 파일 경로
def get_grid_filename(grid_row, grid_col):
    return os.path.join(GRID_FOLDER, f"{grid_row:02d}{grid_col:02d}grid.json")

# JSON에서 배열 불러오기
def load_grid(grid_row, grid_col):
    path = get_grid_filename(grid_row, grid_col)
    if os.path.exists(path):
        with open(path, "r") as f:
            data = json.load(f)
            return np.array(data["grid"], dtype=int)
    else:
        print(f"{path} 없음. 새 배열 생성.")
        return np.zeros((grid_row, grid_col), dtype=int)

# JSON으로 배열 저장
def save_grid(grid, grid_row, grid_col):
    path = get_grid_filename(grid_row, grid_col)
    data = {"grid": grid.tolist()}
    with open(path, "w") as f:
        json.dump(data, f, indent=4)
    print(f"저장 완료: {path}")

# grid 편집기
def main():
    grid_array = load_grid(grid_row, grid_col)

    cv2.namedWindow("Grid")
    cv2.setMouseCallback("Grid", mouse_callback, param=grid_array)
    cv2.imshow("Grid", grid_visual(grid_array))

    print("Instructions:")
    print("- 왼쪽 클릭 또는 드래그: 0과 1 반전")
    print("- s 키: 저장")
    print("- q 키: 종료")

    while True:
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_grid(grid_array,grid_row, grid_col)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
