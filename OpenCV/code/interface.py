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
        
#홈 위치 셀의 배경색을 연하게 칠해주는 함수        
def draw_home_positions(vis_img, positions_dict):
    """홈 위치 셀의 배경색을 연하게 칠하고 로봇 ID를 작게 표시하는 함수"""
    home_bg_color = (220, 220, 220) # BGR 형식의 연한 회색 배경
    text_color = (50, 50, 50)     # BGR 형식의 어두운 회색 텍스트

    # positions_dict는 {로봇ID: (행, 열)} 형태이므로, items()를 사용하여 ID와 위치를 함께 가져옵니다.
    for robot_id, pos in positions_dict.items():
        r, c = pos
        x, y = c * cell_size, r * cell_size
        
        # 1. 홈 배경색 칠하기 (기존 코드와 동일)
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), home_bg_color, -1)
        cv2.addWeighted(overlay, 0.5, vis_img, 0.5, 0, vis_img)

        # 2. 로봇 ID 텍스트 추가 (좌측 상단, 작게)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45  # 폰트 크기 조정 (작게)
        thickness = 1      # 텍스트 두께
        
        # 텍스트 크기 계산 (정확한 위치 조정을 위해)
        text_size = cv2.getTextSize(str(robot_id), font, font_scale, thickness)[0]
        
        # 텍스트 위치 조정 (좌측 상단에서 약간 안쪽으로)
        # x_offset과 y_offset을 조절하여 원하는 위치로 미세 조정할 수 있습니다.
        x_text = x + 3  # 셀의 좌측 경계에서 3픽셀 오른쪽으로
        y_text = y + text_size[1] + 3 # 셀의 상단 경계에서 텍스트 높이 + 3픽셀 아래로
        
        cv2.putText(vis_img, str(robot_id), (x_text, y_text), font, font_scale, text_color, thickness, cv2.LINE_AA)


# 에이전트 정보 창 그리기
def draw_agent_info_window(agents, preset_ids, total_height, selected_robot_id=None,
                           delay_input_mode=False, delay_input_buffer="", cell_size=50, waiting_robots=None):

    if waiting_robots is None:
        waiting_robots = {}


    rows = len(preset_ids) + 1  # 헤더 포함
    cols = 4

    widths = [50, 100, 100, 100]  # ID, Start, Goal, Delay
    cum_widths = [sum(widths[:i]) for i in range(len(widths)+1)]
    table_w = sum(widths)
    row_h = total_height // rows if rows > 0 else total_height
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
            
            # ▼▼▼▼▼ [핵심 수정] Delay 표시 로직 변경 ▼▼▼▼▼
            delay_str = "0" # 기본값은 0
            if aid in waiting_robots:
                # waiting_robots에 ID가 있으면, 남은 대기 시간을 표시
                delay_str = str(waiting_robots[aid])
            # ▲▲▲▲▲ [핵심 수정] 여기까지 ▲▲▲▲▲

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

            # 입력 중인 버퍼 표시
            # putText 위치를 delay_str과 동일하게 맞춰줌
            cv2.putText(info_img, delay_input_buffer + "_", (cum_widths[3] + 5, y_base),
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

def draw_agent_delays_on_grid(vis_img, agents, home_positions=None):
    """
    그리드 상에 각 로봇의 delay(남은 대기 스텝)를 숫자로 표시.
    - 기본: 현재 start 셀 위에 표시
    - home_positions가 주어지면, 홈 셀에 표시
    """
    import cv2
    from config import cell_size

    # 홈 위치에 찍고 싶으면 dict: {rid: (r, c)} 전달 (없으면 현재 start에 그림)
    for a in agents:
        d = getattr(a, "delay", 0) or 0
        if d <= 0:
            continue

        if home_positions and a.id in home_positions:
            r, c = home_positions[a.id]
        elif a.start:
            r, c = a.start
        else:
            continue

        x = c * cell_size + cell_size // 2
        y = r * cell_size + cell_size // 2

        # 원 안에 숫자(딜레이) 그리기
        cv2.circle(vis_img, (x, y), cell_size // 3, (0, 0, 0), 2)
        cv2.putText(vis_img, str(int(d)), (x - 6, y + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
