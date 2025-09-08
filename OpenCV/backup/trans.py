import random
import cv2
import numpy as np
import subprocess
import re
import ast
import os


from config import cbs_path, board_width_cm, board_height_cm, grid_width, grid_height, cell_size
from cbs_manager import CBSManager
from path_relay import get_agent

def run_cbs_manager(grid_array, tag_info):
    """
    cbs_manager를 사용해 CBS를 직접 실행하고, 결과를 relay에 저장
    """
    # Step 1: grid_array와 tag_info로 starts, goals, ids를 만든다
    rows, cols = grid_array.shape

    start_points = []
    goal_points = []
    agent_ids = []

    valid_cells = []
    for r in range(rows):
        for c in range(cols):
            if grid_array[r, c] == 0:
                valid_cells.append((r, c))

    for idx, (tag_id, data) in enumerate(tag_info.items()):
        x_cm, y_cm = data["coordinates"]

        tag_grid_x = int(x_cm * grid_width / board_width_cm)
        tag_grid_y = int(y_cm * grid_height / board_height_cm)
        start_col = tag_grid_x // cell_size
        start_row = tag_grid_y // cell_size

        start_row = max(0, min(rows - 1, start_row))
        start_col = max(0, min(cols - 1, start_col))

        if valid_cells:
            dest_row, dest_col = random.choice(valid_cells)
        else:
            dest_row, dest_col = start_row, start_col

        start_points.append((start_row, start_col))
        goal_points.append((dest_row, dest_col))
        agent_ids.append(tag_id)  # 여기서 tag_id를 agent id로 사용

    # Step 2: CBSManager를 이용해 CBS 실행
    map_array = grid_array.astype(bool)  # 0: 이동 가능, 1: 장애물
    manager = CBSManager(solver_type="ICBS", disjoint=True, visualize_result=False)
    manager.load_instance(map_array, start_points, goal_points, agent_ids)
    manager.run()  # -> 내부에서 자동으로 path_relay에 저장

    # Step 3: 결과 가져오기
    agents = get_agent()

    print("\n=== Extracted Agents ===")
    for agent in agents:
        print(agent)

    return agents


def generate_movement_commands(agent_paths, step_size=10):
    directions = {
        (0, 1): "right",
        (0, -1): "left",
        (1, 0): "down",
        (-1, 0): "up"
    }

    for agent, path in agent_paths.items():
        print(f"\nAgent {agent} movement commands:")
        
        prev_x, prev_y = path[0]
        prev_dir = "down"  # 모든 로봇이 처음 '아래쪽'을 보고 있다고 가정

        for i in range(1, len(path)):
            x, y = path[i]
            dx, dy = x - prev_x, y - prev_y  # 방향 벡터 계산

            if (dx, dy) not in directions:
                print(f"Warning: Unexpected move ({dx}, {dy}) at step {i} for agent {agent}")
                continue

            current_dir = directions[(dx, dy)]

            # 회전 명령이 필요한 경우
            if prev_dir and prev_dir != current_dir:
                if (prev_dir, current_dir) in [("up", "right"), ("right", "down"), ("down", "left"), ("left", "up")]:
                    print(f"{agent}: rotate: right")
                else:
                    print(f"{agent}: rotate: left")

            print(f"{agent}: move: {step_size}")
            
            prev_x, prev_y = x, y
            prev_dir = current_dir  # 현재 방향 업데이트
        
        print("end")  # 한 에이전트의 명령 종료

# def run_cbs():
#     try:
#         result = subprocess.run(
#             ["python", cbs_path+"\\run_experiments.py"] + arguments,
#             capture_output=True,
#             text=True,
#             cwd=cbs_path,
#             check=True
#         )

#         output = result.stdout
#         print("=== Raw CBS Output ===")
#         print(output)

#         agent_paths = {}
#         pattern = re.findall(r"agent (\d+) :\s+(\[.*?\])", output)

#         for agent_id, path in pattern:
#             agent_paths[int(agent_id)] = ast.literal_eval(path)

#         print("\n=== Extracted Paths ===")
#         print(agent_paths)
#         return agent_paths

#     except subprocess.CalledProcessError as e:
#         print("CBS 실행 중 오류 발생:")
#         print(e.stderr)
#         return {}
    
# def export_cbs(grid_array, tag_info):
#     """
#     grid_array: 2D numpy array (예: 12x12), 0 -> '.', 1 -> '@'
#     tag_info: { tag_id: {"coordinates": (x_cm, y_cm), ...}, ... }
#     output_path: 결과를 저장할 텍스트 파일 경로
#     board_width_cm, board_height_cm: 실제 보드 크기 (cm)
#     grid_width, grid_height: 'Grid Visualization' 창의 픽셀 크기
#     cell_size: 한 칸의 픽셀 크기
#     """

#     rows, cols = grid_array.shape

#     lines = []

#     lines.append(f"{rows} {cols}")
#     for r in range(rows):
#         row_symbols = []
#         for c in range(cols):
#             if grid_array[r, c] == 1:
#                 row_symbols.append('@')
#             else:
#                 row_symbols.append('.')
#         lines.append(" ".join(row_symbols))

#     tag_count = len(tag_info)
#     lines.append(str(tag_count))

#     # 4) 0('.')인 위치(이동 가능 구역)만 모아서 랜덤 목적지로 활용
#     #    valid_cells에 (r, c)를 모으고, r, c는 0-based
#     valid_cells = []
#     for r in range(rows):
#         for c in range(cols):
#             if grid_array[r, c] == 0:  # '.'(이동 가능 구역)
#                 valid_cells.append((r, c))

#     # 각 태그마다 (시작위치) (도착위치)를 기록
#     # tag_info[tag_id]["coordinates"] = (x_cm, y_cm)
#     # 이를 grid_visual()와 동일하게 -> 픽셀 = int(x_cm * grid_width / board_width_cm)
#     #                                    셀   = 픽셀 // cell_size
#     for tag_id, data in tag_info.items():
#         x_cm, y_cm = data["coordinates"]  # 실제 보드 상 위치 (cm 단위)

#         # 4-1) 시작 위치( row, col ) 계산
#         tag_grid_x = int(x_cm * grid_width / board_width_cm)  # 픽셀 좌표
#         tag_grid_y = int(y_cm * grid_height / board_height_cm) 
#         start_col = tag_grid_x // cell_size  # 0-based
#         start_row = tag_grid_y // cell_size  # 0-based

#         # 그리드 범위를 벗어나지 않도록 최소/최대값 보정
#         start_row = max(0, min(rows - 1, start_row))
#         start_col = max(0, min(cols - 1, start_col))

#         # 4-2) 목적지( row, col )는 valid_cells에서 랜덤하게 선택 (0-based)
#         if valid_cells:  # 빈 칸이 하나라도 있으면
#             dest_row, dest_col = random.choice(valid_cells)
#         else:
#             # 빈 칸이 없다면 어쩔 수 없이 시작 위치를 목적지로 삼아도 됨
#             dest_row, dest_col = start_row, start_col

#         # 4-3) 출력은 모두 1-based 인덱스로 기록 (예시 형식 따름)
#         s_row_1 = start_row 
#         s_col_1 = start_col
#         d_row_1 = dest_row
#         d_col_1 = dest_col

#         # "start_row start_col dest_row dest_col"
#         lines.append(f"{s_row_1} {s_col_1} {d_row_1} {d_col_1}")

#     # 5) 결과를 파일로 저장
#     output_path=cbs_path+"\\instances\\map.txt"
#     with open(output_path, "w", encoding="utf-8") as f:
#         for line in lines:
#             f.write(line + "\n")

#     print(f"'{output_path}'에 가상맵이 성공적으로 저장되었습니다.")
