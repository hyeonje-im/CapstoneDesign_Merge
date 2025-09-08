import sys
import os
import random

# MAPF-ICBS\code 경로를 추가
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))

import cv2
import numpy as np
from grid import load_grid
from visual import grid_visual, draw_agent_info_window
from cbs.agent import Agent
from visualize import Animation
from simulator import Simulator
from fake_mqtt import FakeMQTTBroker
from commandSendTest3 import CommandSet
from cbs.pathfinder import PathFinder
from config import COLORS, grid_row, grid_col, cell_size
import json

# 전역 변수
agents = []
paths = []
sim = None
broker = FakeMQTTBroker()
pathfinder = None
grid_array = None
selected_robot_id = None # 생성할 때 선택된 로봇 ID

delay_input_mode = False
delay_input_buffer = ""

random_mode_enabled = False

# 사용할 ID 목록
PRESET_IDS = [0,1,2,3,4,5,6,7,8,9]  # 예시: 1~12까지의 ID 사용

# 마우스 콜백 함수
def mouse_event(event, x, y, flags, param):
    """
    좌클릭  : 출발지(start) 지정
    우클릭  : 도착지(goal)  지정
    - PRESET_IDS(예: [2, 4]) 두 개가 모두 완성되면 CBS 실행
    """
    global agents, paths, pathfinder, selected_robot_id
    row, col = y // cell_size, x // cell_size
    if not (0 <= row < grid_row and 0 <= col < grid_col):
        return

    updated = False                 # ← 변경 여부 플래그
    complete_agents = [a for a in agents if a.start and a.goal]

    # ---------- 1. 출발지 클릭 & 로봇 생성 ----------
    if event == cv2.EVENT_LBUTTONDOWN:

        if selected_robot_id is None:
            return  # 아무 것도 선택 안 된 경우 무시
        
        pos = (row, col)

        # 로봇 생성 또는 위치 초기화
        if selected_robot_id in sim.robots:
            robot = sim.robots[selected_robot_id]
            robot.position = pos
            robot.start_pos = pos
            robot.target_pos = pos
            sim.robot_info[selected_robot_id]['start'] = pos
        else:
            robot = sim.add_robot(selected_robot_id, broker, start_pos=pos)

        # 에이전트 생성 + start 설정
        if all(a.id != selected_robot_id for a in agents):
            agent = Agent(id=selected_robot_id, start=pos, goal=None, delay=0)
            agents.append(agent)
        else:
            # 이미 존재하는 agent라면 start만 업데이트 (정합성 보장)
            for agent in agents:
                if agent.id == selected_robot_id:
                    agent.start = pos
                    break

        selected_robot_id = None
        return


    # ---------- 2. 도착지 클릭 ----------
    elif event == cv2.EVENT_RBUTTONDOWN:
        if selected_robot_id is None:
            return  # 아무 것도 선택 안 된 경우 무시

        print(f"Goal set at ({row}, {col})")

        # 2‑A. 이미 완성된 agent가 한도만큼이면 생성 제한
        if len(complete_agents) >= len(PRESET_IDS):
            print("더 이상 agent를 생성할 수 없습니다.")
            return

        # 2‑B. start‑only agent에 goal 채우기
        for agent in agents:
            if agent.id == selected_robot_id and agent.goal is None and agent.start is not None:
                agent.goal = (row, col)
                updated = True
                break

        # 2‑C. goal‑only agent의 goal 덮어쓰기
        if not updated:
            for agent in agents:
                if agent.id == selected_robot_id and agent.goal is not None and agent.start is None:
                    agent.goal = (row, col)
                    updated = True
                    break

        # 2‑D. 둘 다 없으면 새 agent 생성 (goal‑only)
        if not updated:
            used_ids = {a.id for a in agents}
            if selected_robot_id in used_ids:
                # ✅ 이미 존재하는 agent의 goal을 덮어쓰기 (이동 중 goal 변경용)
                for agent in agents:
                    if agent.id == selected_robot_id:
                        agent.goal = (row, col)
                        updated = True
                        print(f"Agent {agent.id}의 도착지를 ({row}, {col})로 변경")
                        break
            else:
                if selected_robot_id not in PRESET_IDS:
                    print(f"{selected_robot_id}는 허용된 ID 목록에 없습니다.")
                    return
                agent = Agent(id=selected_robot_id, start=None, goal=(row, col), delay=0)
                agents.append(agent)
                updated = True

        selected_robot_id = None
        return


    # ---------- 3. 공통 후처리 ----------
    if updated:
        target_ids = set(PRESET_IDS)  # ← PRESET_IDS 기반으로 변경
        ready_ids  = {a.id for a in agents if a.start and a.goal and a.id in target_ids}

        if ready_ids == target_ids:
            print(f"Agent {sorted(ready_ids)} 준비 완료. CBS 실행.")
            compute_cbs()

# #에이전트 초기화
# def create_agent(start=None, goal=None, delay=None, agent_id=None):
#     if agent_id is None:
#         agent_id = len(agents)
#     if delay is None:
#         delay = random.randint(0, 5)
#     return Agent(id=agent_id, start=start, goal=goal, delay=delay)

#에이전트 시작 위치를 로봇 현재 위치로 설정
def get_start_from_robot():
    for agent in agents:
        if agent.id in sim.robots:
            robot = sim.robots[agent.id]
            pos = robot.target_pos if robot.moving else robot.position  # 핵심 변경
            int_pos = tuple(map(int, pos))
            agent.start = int_pos
            sim.robot_info[agent.id]['start'] = int_pos

# 에이전트 초기 방향을 로봇의 회전 방향으로 설정
def get_direction_from_robot():
    for agent in agents:
        if agent.id in sim.robots:
            robot = sim.robots[agent.id]
            directions = ["north", "east", "south", "west"]
            idx = directions.index(robot.direction)

            if robot.rotating and robot.rotation_dir:
                delta = 1 if robot.rotation_dir == "right" else -1
                expected_dir = directions[(idx + delta) % 4]
            else:
                expected_dir = robot.direction

            agent.initial_dir = expected_dir  # CommandSet 생성 시 참조할 수 있게 저장



#CBS 계산
def compute_cbs():
    global paths, pathfinder, grid_array

    grid_array = load_grid(grid_row, grid_col)
    get_start_from_robot()
    if random_mode_enabled:
        assigned = False
        for agent in agents:
            if agent.start and agent.goal is None:
                pos = agent.start
                empty_cells = [(r, c) for r in range(grid_array.shape[0])
                                            for c in range(grid_array.shape[1])
                                            if grid_array[r, c] == 0 and (r, c) != pos]
                if empty_cells:
                    agent.goal = random.choice(empty_cells)
                    assigned = True

    if pathfinder is None:
        pathfinder = PathFinder(grid_array)

    # # CBS 계산 전 모든 delay를 1회용으로 처리
    # for agent in agents:
    #     if agent.start is not None and agent.goal is not None:
    #         if agent.delay > 0:
    #             print(f"[딜레이 적용] Agent {agent.id} → delay {agent.delay} 적용 후 제거")
    #             delay = agent.delay
    #             agent.delay = 0  # 딜레이는 1회만 적용되도록 초기화
    #         else:
    #             delay = 0
    #         agent._applied_delay = delay  # 내부 추적용, 없어도 됨

    new_agents = pathfinder.compute_paths(agents)
    new_paths = [agent.get_final_path() for agent in new_agents]

    if not new_paths:
        print("No solution found.")
        return

    paths.clear()
    paths.extend(new_paths)

    print("Paths updated via PathFinder.")
    
    for agent in agents:
        agent.delay = 0

    # 로봇 명령 전송
    command_sets = []
    for agent in new_agents:
        print(f"[DEBUG] Agent {agent.id}: delay={agent.delay}, path={agent.get_final_path()}")
        if agent.id in sim.robots:
            robot = sim.robots[agent.id]

            # 예상 방향 계산 (회전 보간 포함)
            directions = ["north", "east", "south", "west"]
            idx = directions.index(robot.direction)

            if robot.rotating and robot.rotation_dir:
                delta = 1 if robot.rotation_dir == "right" else -1
                expected_dir = directions[(idx + delta) % 4]
            else:
                expected_dir = robot.direction

            
            command_sets.append(CommandSet(str(agent.id), agent.get_final_path(), initial_dir=expected_dir))


# 전송할 JSON 문자열을 미리 출력
    try:
        payload = json.dumps({"commands": [cs.to_dict() for cs in command_sets]}, indent=2, ensure_ascii=False)
        print("!!!전송 예정 명령 세트:")
        print(payload)
    except Exception as e:
        print(f"명령 세트 변환 중 오류 발생: {e}")

    # # 실제 전송 시도
    # try:
    #     CommandSet.send_command_sets(command_sets)
    # except Exception as e:
    #     print(f"명령 전송 중 오류 발생: {e}")

    broker.send_command_sets(command_sets)
        
    if sim:
        for agent in new_agents:
            if agent.id in sim.robots:
                sim.robot_info[agent.id]['path'] = agent.get_final_path()
                sim.robot_info[agent.id]['goal'] = agent.goal

#경로 색칠용 코드
def draw_paths(vis_img, paths):
    # 1. paths (CBS 경로) 색칠
    for idx, path in enumerate(paths):
        color = COLORS[idx % len(COLORS)]
        for pos in path:
            r, c = pos
            x, y = c * cell_size, r * cell_size
            overlay = vis_img.copy()
            cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
            cv2.addWeighted(overlay, 0.3, vis_img, 0.7, 0, vis_img)
    
    # # 2. 추가: sim.robot_past_paths에 저장된 지나간 경로도 색칠
    # if sim:
    #     for robot_id, past_path in sim.robot_past_paths.items():
    #         color = COLORS[robot_id % len(COLORS)]
    #         for pos in past_path:
    #             r, c = pos
    #             x, y = c * cell_size, r * cell_size
    #             overlay = vis_img.copy()
    #             cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
    #             cv2.addWeighted(overlay, 0.3, vis_img, 0.7, 0, vis_img)

# 로봇 도착 시 재계산
def on_robot_arrival(robot_id, pos):
    global agents, sim

    if not random_mode_enabled:
        return

    empty_cells = [(r, c) for r in range(grid_array.shape[0])
                             for c in range(grid_array.shape[1])
                             if grid_array[r, c] == 0 and (r, c) != pos]

    if not empty_cells:
        print(f"[경고] 도착지 후보가 없음 (로봇 {robot_id})")
        return

    new_goal = random.choice(empty_cells)
    print(f"[랜덤 모드] 로봇 {robot_id} 새 목표 {new_goal}")

    for agent in agents:
        if agent.id == robot_id:
            agent.start = pos
            agent.goal = new_goal
            break

    compute_cbs()

def main():
    global agents, paths, grid_array, selected_robot_id, sim, delay_input_buffer, delay_input_mode, random_mode_enabled
    grid_array = load_grid(grid_row, grid_col)
    cv2.namedWindow("CBS Grid")
    cv2.setMouseCallback("CBS Grid", mouse_event)

    sim = Simulator(grid_array.astype(bool), colors=COLORS)
    sim.register_arrival_callback(on_robot_arrival)
    sim.run_simulator(tick_interval=0.1)

    while True:
        vis = grid_visual(grid_array.copy())
        draw_paths(vis, paths)

        for agent in agents:
            if agent.id in sim.robots:
                pos = sim.robots[agent.id].get_position()
                x, y = int(pos[1] * cell_size), int(pos[0] * cell_size)
                cv2.circle(vis, (x + cell_size//2, y + cell_size//2), 5, (0, 255, 0), -1)
                cv2.putText(vis, f"S{agent.id}", (x + 2, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        for agent in agents:
            if agent.goal:
                x, y = agent.goal[1] * cell_size, agent.goal[0] * cell_size
                cv2.circle(vis, (x + cell_size//2, y + cell_size//2), 5, (0, 0, 255), -1)
                cv2.putText(vis, f"G{agent.id}", (x + 2, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        agent_info_img = draw_agent_info_window(
            agents,
            preset_ids=PRESET_IDS,
            total_height=grid_array.shape[0] * cell_size,
            selected_robot_id=selected_robot_id,
            delay_input_mode=delay_input_mode,
            delay_input_buffer= delay_input_buffer,
            cell_size=cell_size
        )

        combined = cv2.hconcat([vis, agent_info_img])
        cv2.imshow("CBS Grid", combined)
        
        sim.draw_frame()
        
        # 키보드 입력 처리
        key = cv2.waitKey(100)
        if key != -1:
            key_char = chr(key & 0xFF)
            if delay_input_mode:
                if key_char.isdigit():
                    delay_input_buffer += key_char
                elif key == 8:  # Backspace
                    delay_input_buffer = delay_input_buffer[:-1]
                elif key == 13 or key == 10:  # Enter
                    if selected_robot_id is not None and delay_input_buffer.isdigit():
                        delay_val = int(delay_input_buffer)
                        existing = next((a for a in agents if a.id == selected_robot_id), None)
                        if existing:
                            existing.delay = delay_val
                        else:
                            agent = Agent(id=selected_robot_id, start=None, goal=None, delay=delay_val)
                            agents.append(agent)
                    delay_input_mode = False
                    delay_input_buffer = ""

            else:
                if key_char.isdigit():
                    selected_robot_id = int(key_char)
                    if selected_robot_id in PRESET_IDS:
                        print(f"로봇 ID {selected_robot_id} 선택됨.")
                elif key == ord('d') and selected_robot_id in PRESET_IDS:
                    print(f"Delay 입력 모드 진입 (ID {selected_robot_id})")
                    delay_input_mode = True
                    delay_input_buffer = ""

        if key == ord('q'): 
            sim.stop()
            break
        elif key == ord('z'):
            print("Reset all")
            agents.clear()
            paths.clear()
        elif key == ord('a'):
            if paths:
                print("Playing animation of last CBS result...")
                animation = Animation(grid_array.astype(bool),
                              [agent.start for agent in agents],
                              [agent.goal for agent in agents],
                              [agent.get_final_path() for agent in agents])
                animation.show()
                animation.save("demo.gif", speed=1.0)
            else:
                print("No paths available to animate.")
        elif key == ord(' '):  # ✅ Spacebar 눌러서 일시정지
            sim.paused = not sim.paused
            print("Paused" if sim.paused else "Resumed")
        
        elif key == ord('c'):  # 'c' 키로 CBS 재계산
            compute_cbs()

        elif key == ord('x'):
            selected_robot_id = None
            delay_input_mode = False
            delay_input_buffer = ""

        elif key == ord('r'):
            random_mode_enabled = not random_mode_enabled
            sim.random_mode_enabled = random_mode_enabled
            
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
