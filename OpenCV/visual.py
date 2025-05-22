import cv2
import numpy as np

from .config import board_height_cm, board_width_cm, grid_width, grid_height,cell_size

def grid_ini(rows=12, cols=12):
    return np.zeros((rows, cols), dtype=int)

def grid_tag_visual(grid_visual, tag_info):
    pass

def info_tag(frame, tag_info):
    """
    화면 좌측 상단에 태그별 속도/방향 정보를 표시
    """
    base_x = 10
    base_y = 30
    line_height = 20

    for idx, (tag_id, data) in enumerate(sorted(tag_info.items())):
        if data["status"] != "On":
            continue
        v = data.get("velocity", (0, 0))
        vx, vy = v
        text = f"ID {tag_id}: V=({vx:.2f}, {vy:.2f})"
        pos = (base_x, base_y + idx * line_height)
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        smoothed = data.get("smoothed_coordinates")
        if smoothed:
            sx, sy = int(smoothed[0]), int(smoothed[1])
            ex, ey = int(sx + vx * 50), int(sy + vy * 50)
            cv2.arrowedLine(frame, (sx, sy), (ex, ey), (255, 0, 0), 2)



def trackbar(val):
    pass

def slider_create():
    aspect_ratio = board_width_cm / board_height_cm # 가로 세로 비율 계산
    min_ratio = max(1.0, aspect_ratio - 0.25)  #오차 0.25
    max_ratio = min(2.0, aspect_ratio + 0.25)

    # 슬라이더 생성 (정수형이므로 10x 스케일 사용)
    cv2.namedWindow("Detected Rectangle")
    cv2.createTrackbar("Brightness Threshold", "Detected Rectangle", 120, 255, trackbar)
    cv2.createTrackbar("Min W/H Ratio", "Detected Rectangle", int(min_ratio * 10), 20, trackbar)
    cv2.createTrackbar("Max W/H Ratio", "Detected Rectangle", int(max_ratio * 10), 20, trackbar)

def slider_value():
    brightness_threshold = cv2.getTrackbarPos("Brightness Threshold", "Detected Rectangle")
    min_aspect_ratio = cv2.getTrackbarPos("Min W/H Ratio", "Detected Rectangle") / 10.0
    max_aspect_ratio = cv2.getTrackbarPos("Max W/H Ratio", "Detected Rectangle") / 10.0
    return brightness_threshold, min_aspect_ratio, max_aspect_ratio

def grid_visual(grid_array):
    visual = np.ones((grid_height, grid_width, 3), dtype=np.uint8) * 255

    for i in range(grid_array.shape[0]):
        for j in range(grid_array.shape[1]):
            cell_x = j * cell_size
            cell_y = i * cell_size
            color = (0, 0, 0) if grid_array[i, j] == 1 else (255, 255, 255)
            cv2.rectangle(visual, (cell_x, cell_y), (cell_x + cell_size, cell_y + cell_size), color, -1)

    # 격자선 그리기
    for i in range(grid_array.shape[0] + 1):
        cv2.line(visual, (0, i * cell_size), (grid_width, i * cell_size), (200, 200, 200), 1)

    for j in range(grid_array.shape[1] + 1):
        cv2.line(visual, (j * cell_size, 0), (j * cell_size, grid_height), (200, 200, 200), 1)

    return visual

# 마우스 상태 변수
is_mouse_pressed = False
last_toggled = None

# mouse_callback(grid_array)를 받게 수정 ✅
def mouse_callback(event, x, y, flags, param):
    global is_mouse_pressed, last_toggled

    grid_array = param   # 여기서 param으로 grid_array 받아쓰기
    row, col = y // cell_size, x // cell_size
    if not (0 <= row < grid_array.shape[0] and 0 <= col < grid_array.shape[1]):
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        is_mouse_pressed = True
        grid_array[row, col] = 1 - grid_array[row, col]
        last_toggled = (row, col)
        cv2.imshow("Grid", grid_visual(grid_array))

    elif event == cv2.EVENT_MOUSEMOVE and is_mouse_pressed:
        if last_toggled != (row, col):
            grid_array[row, col] = 1 - grid_array[row, col]
            last_toggled = (row, col)
            cv2.imshow("Grid", grid_visual(grid_array))

    elif event == cv2.EVENT_LBUTTONUP:
        is_mouse_pressed = False
        last_toggled = None

def draw_agent_info_window(agents, preset_ids, total_height, selected_robot_id=None,
                           delay_input_mode=False, delay_input_buffer="", cell_size=50):
    """
    ID 목록(PRESET_IDS)에 맞게 고정된 행 수를 유지하고, 가로 폭은 350으로 고정함.
    - agents: 현재 존재하는 agent 리스트
    - preset_ids: 허용된 전체 ID 리스트 (e.g. [1,2,3,...])
    - total_height: CBS Grid의 높이 (픽셀 기준)
    """
    rows = len(preset_ids) + 1  # 헤더 포함
    cols = 4

    widths = [50, 100, 100, 100]  # ID, Start, Goal, Delay
    cum_widths = [sum(widths[:i]) for i in range(len(widths)+1)]
    table_w = sum(widths)
    row_h = total_height // rows
    table_h = total_height

    info_img = np.ones((table_h, table_w, 3), dtype=np.uint8) * 255

    # 헤더
    headers = ['ID', 'Start', 'Goal', 'Delay']
    for j, text in enumerate(headers):
        x = cum_widths[j] + 5
        y = int(row_h * 0.6)
        cv2.putText(info_img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # agent 정보를 ID 순서대로 채우기
    agent_dict = {a.id: a for a in agents}
    for i, aid in enumerate(preset_ids):
        agent = agent_dict.get(aid, None)
        y_base = (i + 1) * row_h + int(row_h * 0.6)

        cv2.putText(info_img, str(aid), (cum_widths[0] + 5, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        if agent:
            start_str = str(agent.start) if agent.start else "-"
            goal_str = str(agent.goal) if agent.goal else "-"
            delay_str = str(agent.delay)
            cv2.putText(info_img, start_str, (cum_widths[1] + 5, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(info_img, goal_str, (cum_widths[2] + 5, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(info_img, delay_str, (cum_widths[3] + 5, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
 
        # (A) 선택된 ID의 전체 줄 강조 (노란색)
        if aid == selected_robot_id:
            overlay = info_img.copy()
            y0 = (i + 1) * row_h
            y1 = (i + 2) * row_h
            cv2.rectangle(overlay, (0, y0), (table_w, y1), (0, 255, 255), -1)
            cv2.addWeighted(overlay, 0.3, info_img, 0.7, 0, info_img)

        # (B) 딜레이 칸만 별도 강조 (주황색)
        if aid == selected_robot_id and delay_input_mode:
            overlay = info_img.copy()
            x0 = cum_widths[3]  # Delay 열 시작
            x1 = cum_widths[4]  # Delay 열 끝
            y0 = (i + 1) * row_h
            y1 = (i + 2) * row_h
            cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 165, 255), -1)
            cv2.addWeighted(overlay, 0.5, info_img, 0.5, 0, info_img)

            if delay_input_buffer:
                cv2.putText(info_img, delay_input_buffer, (x0 + 5, y0 + int(row_h * 0.6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)



    # 라인
    for i in range(rows + 1):
        y = i * row_h
        cv2.line(info_img, (0, y), (table_w, y), (200, 200, 200), 1)
    for x in cum_widths:
        cv2.line(info_img, (x, 0), (x, table_h), (200, 200, 200), 1)

    return info_img

