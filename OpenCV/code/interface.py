import cv2
import numpy as np

from config import board_height_cm, board_width_cm, grid_width, grid_height, cell_size, COLORS

def trackbar(val):
    pass

def slider_create():
    aspect_ratio = board_width_cm / board_height_cm  # 가로 세로 비율 계산
    min_ratio = max(1.0, aspect_ratio - 0.25)
    max_ratio = min(2.0, aspect_ratio + 0.25)

    # 슬라이더 전용 창 생성
    cv2.namedWindow("Sliders", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Sliders", 500, 120)  # 원하는 고정 크기로 조정

    # 슬라이더를 "Sliders" 창에 생성
    cv2.createTrackbar("Brightness Threshold", "Sliders", 120, 255, trackbar)
    cv2.createTrackbar("Min W/H Ratio", "Sliders", int(min_ratio * 10), 20, trackbar)
    cv2.createTrackbar("Max W/H Ratio", "Sliders", int(max_ratio * 10), 20, trackbar)
    cv2.createTrackbar("Angle(cos)", "Sliders", 50, 100, trackbar)
    cv2.createTrackbar("extent", "Sliders", 50, 100, trackbar)
    cv2.createTrackbar("solidity", "Sliders", 60, 100, trackbar)


def slider_value():
    brightness_threshold = cv2.getTrackbarPos("Brightness Threshold", "Sliders")
    min_aspect_ratio = cv2.getTrackbarPos("Min W/H Ratio", "Sliders") / 10.0
    max_aspect_ratio = cv2.getTrackbarPos("Max W/H Ratio", "Sliders") / 10.0
    cos_th    = cv2.getTrackbarPos("Angle(cos)", "Sliders")  / 100.0
    extent_th = cv2.getTrackbarPos("extent", "Sliders")      / 100.0
    solid_th  = cv2.getTrackbarPos("solidity", "Sliders")    / 100.0
    return brightness_threshold, min_aspect_ratio, max_aspect_ratio, cos_th, extent_th, solid_th

# 그리드 그리기
def grid_visual(grid_array):
    visual = np.ones((grid_height, grid_width, 3), dtype=np.uint8) * 255

    for i in range(grid_array.shape[0]):
        for j in range(grid_array.shape[1]):
            cell_x = j * cell_size
            cell_y = i * cell_size
            color = (0, 0, 0) if grid_array[i, j] == 1 else (255, 255, 255)
            cv2.rectangle(visual, (cell_x, cell_y), (cell_x + cell_size, cell_y + cell_size), color, -1)

    for i in range(grid_array.shape[0] + 1):
        cv2.line(visual, (0, i * cell_size), (grid_width, i * cell_size), (200, 200, 200), 1)

    for j in range(grid_array.shape[1] + 1):
        cv2.line(visual, (j * cell_size, 0), (j * cell_size, grid_height), (200, 200, 200), 1)

    return visual

# 마우스 상태 변수
is_mouse_pressed = False
last_toggled = None

# 마우스 입력 처리
def mouse_callback(event, x, y, flags, param):
    global is_mouse_pressed, last_toggled

    grid_array = param 
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

# 에이전트 정보 창 그리기
def draw_agent_info_window(agents, preset_ids, total_height, selected_robot_id=None,
                           delay_input_mode=False, delay_input_buffer="", cell_size=50):
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

# 그리드에 에이전트 포인트 그리기
def draw_agent_points(vis_img, agents):
    for agent in agents:
        if agent.start:
            x, y = agent.start[1] * cell_size, agent.start[0] * cell_size
            cv2.circle(vis_img, (x + cell_size//2, y + cell_size//2), 5, (0, 255, 0), -1)
            cv2.putText(vis_img, f"S{agent.id}", (x + 2, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        if agent.goal:
            x, y = agent.goal[1] * cell_size, agent.goal[0] * cell_size
            cv2.circle(vis_img, (x + cell_size//2, y + cell_size//2), 5, (0, 0, 255), -1)
            cv2.putText(vis_img, f"G{agent.id}", (x + 2, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

# CBS 경로 그리기
def draw_paths(vis_img, paths):
    for idx, path in enumerate(paths):
        color = COLORS[idx % len(COLORS)]
        for pos in path:
            r, c = pos
            x, y = c * cell_size, r * cell_size
            overlay = vis_img.copy()
            cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
            cv2.addWeighted(overlay, 0.3, vis_img, 0.7, 0, vis_img)