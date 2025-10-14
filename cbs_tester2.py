import sys
import os
import random
from collections import deque

# MAPF-ICBS\code 경로를 추가
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))


import cv2
import numpy as np
from grid import load_grid
from interface import grid_visual, draw_agent_info_window
from simulator import Simulator
from fake_mqtt import FakeMQTTBroker
from commandSendTest3 import CommandSet
from cbs.pathfinder import PathFinder, Agent
from config import COLORS, grid_row, grid_col, cell_size
import json

# 전역 변수

# ▼▼▼▼▼ [수정] 시작 위치와 Home 위치를 명확히 분리하여 정의 ▼▼▼▼▼
# 로봇별 고정 시작 위치 정의
ROBOT_START_POSITIONS = {
    1: (1, 0),
    2: (3, 0),
    3: (5, 0),
}
# 로봇별 고정 'home' 위치 정의 (시작 위치와 동일하게 설정)
ROBOT_HOME_POSITIONS = {
    1: (1, 0),
    2: (3, 0),
    3: (5, 0),
}
# ▲▲▲▲▲ [수정] 여기까지 ▲▲▲▲▲


agents = []
paths = []
sim = None
broker = FakeMQTTBroker()
pathfinder = None
grid_array = None
selected_robot_id = None # 생성할 때 선택된 로봇 ID

pending_steps = {}          # { robot_id: deque([(r,c), ...]) }
barrier_inflight = {}    # 직전에 보낸 스텝을 아직 수행 중인 로봇들
BARRIER_MODE = True         # 끄고 싶으면 False

delay_input_mode = False
delay_input_buffer = ""

random_mode_enabled = False

robot_next_phase = {}  # { robot_id: 'to_table' | 'to_home' }
replan_pending = False

# 사용할 ID 목록
PRESET_IDS = [0,1,2,3,4,5,6,7,8,9]

# 마우스 콜백 함수
def mouse_event(event, x, y, flags, param):
    """
    (수동 테스트용으로 기능 유지)
    좌클릭  : 출발지(start) 지정
    우클릭  : 도착지(goal)  지정
    """
    global agents, paths, pathfinder, selected_robot_id
    row, col = y // cell_size, x // cell_size
    if not (0 <= row < grid_row and 0 <= col < grid_col):
        return

    updated = False
    complete_agents = [a for a in agents if a.start and a.goal]

    if event == cv2.EVENT_LBUTTONDOWN:
        if selected_robot_id is None: return
        pos = (row, col)
        if selected_robot_id in sim.robots:
            robot = sim.robots[selected_robot_id]
            robot.position = pos
            robot.start_pos = pos
            robot.target_pos = pos
            sim.robot_info[selected_robot_id]['start'] = pos
        else:
            robot = sim.add_robot(selected_robot_id, broker, start_pos=pos)

        if all(a.id != selected_robot_id for a in agents):
            agents.append(Agent(id=selected_robot_id, start=pos, goal=None, delay=0))
        else:
            for agent in agents:
                if agent.id == selected_robot_id:
                    agent.start = pos
                    break
        selected_robot_id = None
        return

    elif event == cv2.EVENT_RBUTTONDOWN:
        if selected_robot_id is None: return
        print(f"Goal set at ({row}, {col})")
        
        # ✅ 이미 존재하는 agent의 goal을 덮어쓰기 (이동 중 goal 변경용)
        for agent in agents:
            if agent.id == selected_robot_id:
                agent.goal = (row, col)
                updated = True
                print(f"Agent {agent.id}의 도착지를 ({row}, {col})로 변경")
                break
        
        # ✅ 존재하지 않으면 새로 생성 (수동 테스트용)
        if not updated:
            agent = Agent(id=selected_robot_id, start=None, goal=(row, col), delay=0)
            agents.append(agent)
            updated = True

        selected_robot_id = None
        
        if updated:
            print(f"Agent {selected_robot_id} 준비 완료. CBS 실행을 위해 'c'를 누르세요.")

        return

# ... (이하 _neighbors4, _tables_with_adjacent_free 등 헬퍼 함수들은 변경 없음) ...
def _neighbors4(r, c, H, W):
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        rr, cc = r+dr, c+dc
        if 0 <= rr < H and 0 <= cc < W:
            yield rr, cc

def _tables_with_adjacent_free(grid):
    H, W = grid.shape
    out = []
    for r in range(H):
        for c in range(W):
            if grid[r, c] != 0:
                adj = [(rr, cc) for (rr, cc) in _neighbors4(r, c, H, W) if grid[rr, cc] == 0]
                if adj:
                    out.append(((r, c), adj))
    return out

def _left_column_free_cells(grid):
    H, W = grid.shape
    return [(r, 0) for r in range(H) if grid[r, 0] == 0]

def get_start_from_robot():
    for agent in agents:
        if agent.id in sim.robots:
            robot = sim.robots[agent.id]
            pos = robot.position
            int_pos = tuple(map(int, pos))
            agent.start = int_pos
            sim.robot_info[agent.id]['start'] = int_pos

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
            agent.initial_dir = expected_dir

def _expected_dir(robot):
    directions = ["north", "east", "south", "west"]
    idx = directions.index(robot.direction)
    if robot.rotating and getattr(robot, "rotation_dir", None):
        delta = 1 if robot.rotation_dir == "right" else -1
        return directions[(idx + delta) % 4]
    return robot.direction

def send_next_step(robot_id):
    if robot_id not in pending_steps or not pending_steps[robot_id]: return False
    if robot_id not in sim.robots: return False
    robot = sim.robots[robot_id]
    if robot.moving or robot.rotating: return False
    cur_pos = tuple(map(int, sim.robots[robot_id].get_position()))
    while pending_steps[robot_id] and tuple(pending_steps[robot_id][0]) == cur_pos:
        pending_steps[robot_id].popleft()
    if not pending_steps[robot_id]: return False
    target = tuple(pending_steps[robot_id][0])
    dr = 1 if target[0] > cur_pos[0] else -1 if target[0] < cur_pos[0] else 0
    dc = 1 if target[1] > cur_pos[1] else -1 if target[1] < cur_pos[1] else 0
    if abs(target[0]-cur_pos[0]) + abs(target[1]-cur_pos[1]) > 1:
        step = (cur_pos[0] + dr, cur_pos[1]) if dr != 0 else (cur_pos[0], cur_pos[1] + dc)
    else:
        step = pending_steps[robot_id].popleft()
    if abs(step[0]-cur_pos[0]) + abs(step[1]-cur_pos[1]) != 1:
        print(f"[WARN] non-unit step {cur_pos}->{step}, target={target}; forcing axis step")
        step = (cur_pos[0] + (1 if dr != 0 else 0), cur_pos[1] + (1 if (dr == 0 and dc != 0) else 0))
    cs = CommandSet(str(robot_id), [cur_pos, step], initial_dir=_expected_dir(robot))
    broker.send_command_sets([cs])
    barrier_inflight[robot_id] = step
    return True

def _all_idle(ids):
    for rid in ids:
        if rid not in sim.robots: return False
        r = sim.robots[rid]
        if r.moving or r.rotating: return False
    return True

def dispatch_if_barrier_ready():
    for rid, tgt in list(barrier_inflight.items()):
        if rid not in sim.robots:
            barrier_inflight.pop(rid, None)
            continue
        r = sim.robots[rid]
        pos = tuple(map(int, r.get_position()))
        if (not r.moving and not r.rotating) and pos == tgt:
            barrier_inflight.pop(rid, None)
    if barrier_inflight: return False
    active = [rid for rid, dq in pending_steps.items() if dq]
    if not active: return False
    if not _all_idle(active): return False
    for rid in active:
        send_next_step(rid)
    return True

def expand_to_unit_steps(path):
    out = []
    for i in range(len(path) - 1):
        r1, c1 = path[i]; r2, c2 = path[i + 1]
        dr = 0 if r2 == r1 else (1 if r2 > r1 else -1)
        dc = 0 if c2 == c1 else (1 if c2 > c1 else -1)
        if dr != 0 and dc != 0: raise ValueError(f"Diagonal segment in path: {path[i]}->{path[i+1]}")
        rr, cc = r1, c1
        while (rr, cc) != (r2, c2):
            rr += dr; cc += dc
            out.append((rr, cc))
    return out

def compute_cbs():
    global paths, pathfinder, grid_array, pending_steps, barrier_inflight
    grid_array = load_grid(grid_row, grid_col)
    get_start_from_robot()
    if pathfinder is None:
        pathfinder = PathFinder(grid_array)
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
    pending_steps.clear()
    barrier_inflight.clear()
    for agent in new_agents:
        if agent.id in sim.robots:
            fp = agent.get_final_path() or []
            unit_steps = expand_to_unit_steps(fp) if len(fp) > 1 else []
            pending_steps[agent.id] = deque(unit_steps)
    if sim:
        for agent in new_agents:
            if agent.id in sim.robots:
                sim.robot_info[agent.id]['path'] = agent.get_final_path()
                sim.robot_info[agent.id]['goal'] = agent.goal

def draw_paths(vis_img, paths):
    for idx, path in enumerate(paths):
        color = COLORS[idx % len(COLORS)]
        for pos in path:
            r, c = pos
            x, y = c * cell_size, r * cell_size
            overlay = vis_img.copy()
            cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
            cv2.addWeighted(overlay, 0.3, vis_img, 0.7, 0, vis_img)
            
def draw_home_positions(vis_img, positions_dict):
    """지정된 위치에 은은한 배경색을 칠합니다."""
    color = (220, 220, 220)  # 연한 회색 (B, G, R)
    for pos in positions_dict.values():
        r, c = pos
        x, y = c * cell_size, r * cell_size
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
        # addWeighted를 사용하여 반투명 효과 적용
        cv2.addWeighted(overlay, 0.5, vis_img, 0.5, 0, vis_img)
           

# ... (이하 _goal_cells_except 등 on_robot_arrival을 위한 헬퍼 함수들은 변경 없음) ...
def _goal_cells_except(robot_id):
    return {a.goal for a in agents if a.id != robot_id and a.goal}

def _occupied_cells_now():
    occ = set()
    for rid, rob in sim.robots.items():
        pos = tuple(map(int, rob.get_position()))
        occ.add(pos)
    return occ

def _reserved_tables_except(robot_id, grid):
    H, W = grid.shape
    reserved = set()
    for a in agents:
        if a.id == robot_id or not a.goal: continue
        r, c = a.goal
        for rr, cc in _neighbors4(r, c, H, W):
            if grid[rr, cc] != 0:
                reserved.add((rr, cc))
                break
    return reserved

def _reserved_left_cells_except(robot_id, grid):
    return {a.goal for a in agents
            if a.id != robot_id and a.goal and a.goal[1] == 0 and grid[a.goal[0], 0] == 0}

# 로봇 도착 시 재계산 (이 함수는 변경 없음)

def on_robot_arrival(robot_id, pos):
    global agents, sim, robot_next_phase, replan_pending, grid_array
    if not random_mode_enabled: return
    pos = tuple(map(int, pos))
    ag = next((a for a in agents if a.id == robot_id), None)
    if not ag or not ag.goal or tuple(ag.goal) != pos: return
    if pending_steps.get(robot_id) and len(pending_steps[robot_id]) > 0: return

    phase = robot_next_phase.get(robot_id, 'to_table')
    goals_others = _goal_cells_except(robot_id)
    occupied_now = _occupied_cells_now()
    new_goal = None

    if phase == 'to_table':
        tables = _tables_with_adjacent_free(grid_array)
        reserved_tables = _reserved_tables_except(robot_id, grid_array)
        available_tables = [(tbl, adj) for (tbl, adj) in tables if tbl not in reserved_tables]
        if not available_tables: available_tables = tables
        if not available_tables:
            print(f"[경고] 식탁 후보가 없음 (로봇 {robot_id})")
            return
        table_pos, adj_list = random.choice(available_tables)
        candidates = [p for p in adj_list if p != pos and p not in goals_others and p not in occupied_now]
        if not candidates: candidates = [p for p in adj_list if p != pos]
        if not candidates:
            print(f"[경고] 식탁 {table_pos} 주변에 할당 가능한 빈칸이 없음 (로봇 {robot_id})")
            return
        new_goal = random.choice(candidates)
        robot_next_phase[robot_id] = 'to_home'
    else:  # phase == 'to_home'
        if robot_id in ROBOT_HOME_POSITIONS:
            new_goal = ROBOT_HOME_POSITIONS[robot_id]
        else:
            left_cells = _left_column_free_cells(grid_array)
            reserved_left = _reserved_left_cells_except(robot_id, grid_array)
            candidates = [p for p in left_cells if p != pos and p not in reserved_left and p not in goals_others and p not in occupied_now]
            if not candidates: candidates = [p for p in left_cells if p != pos]
            if not candidates:
                print(f"[경고] 왼쪽 1열에 할당 가능한 빈칸이 없음 (로봇 {robot_id})")
                return
            new_goal = random.choice(candidates)
        robot_next_phase[robot_id] = 'to_table'

    print(f"[랜덤 모드] 로봇 {robot_id} 새 목표 {new_goal} (phase→ {robot_next_phase[robot_id]})")
    for a in agents:
        if a.id == robot_id:
            a.start = pos
            a.goal = new_goal
            break

    # --- [하이브리드 재계획 로직] ---
    # 1. 모든 로봇의 현재 위치를 즉시 갱신
    get_start_from_robot()

    # 2. 안전 검사: 모든 로봇이 장애물이 아닌 유효한 셀에 있는지 확인
    is_safe_to_replan_immediately = True
    for agent in agents:
        start_pos = agent.start
        # grid_array가 None인 경우를 방지하기 위해 로드
        if grid_array is None:
            grid_array = load_grid(grid_row, grid_col)
        
        if grid_array[start_pos[0], start_pos[1]] != 0: # 0이 아니면 장애물
            print(f"[안전 검사] 로봇 {agent.id}가 장애물 위치({start_pos})에 있어 즉시 재계획을 보류합니다.")
            is_safe_to_replan_immediately = False
            break

    # 3. 조건에 따라 재계획 방식 결정
    if is_safe_to_replan_immediately:
        print("[정보] 모든 로봇이 안전한 위치에 있어 즉시 재계산을 실행합니다.")
        compute_cbs()  # 안전하므로 즉시 실행
    else:
        replan_pending = True  # 안전하지 않으므로 동기화 대기
        
def main():
    global agents, paths, grid_array, selected_robot_id, sim
    global delay_input_buffer, delay_input_mode, random_mode_enabled, replan_pending
    grid_array = load_grid(grid_row, grid_col)
    cv2.namedWindow("CBS Grid")
    cv2.setMouseCallback("CBS Grid", mouse_event)

    sim = Simulator(grid_array.astype(bool), colors=COLORS)
    sim.register_arrival_callback(on_robot_arrival)
    
    # ▼▼▼▼▼ [추가] 프로그램 시작 시 로봇 자동 생성 ▼▼▼▼▼
    for robot_id, start_pos in ROBOT_START_POSITIONS.items():
        if robot_id not in PRESET_IDS:
            continue
        # 시뮬레이터에 로봇 추가
        sim.add_robot(robot_id, broker, start_pos=start_pos)
        # CBS 에이전트 생성
        agent = Agent(id=robot_id, start=start_pos, goal=None, delay=0)
        agents.append(agent)
    print(f"{len(agents)}개의 로봇이 초기 위치에 자동 생성되었습니다.")
    # ▲▲▲▲▲ [추가] 여기까지 ▲▲▲▲▲

    while True:
        vis = grid_visual(grid_array.copy())
        draw_home_positions(vis, ROBOT_HOME_POSITIONS)
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
            agents, preset_ids=PRESET_IDS, total_height=grid_array.shape[0] * cell_size,
            selected_robot_id=selected_robot_id, delay_input_mode=delay_input_mode,
            delay_input_buffer=delay_input_buffer, cell_size=cell_size
        )

        combined = cv2.hconcat([vis, agent_info_img])
        cv2.imshow("CBS Grid", combined)
        
        sim.run_once()
        dispatch_if_barrier_ready()
        
        if not barrier_inflight and replan_pending:
            replan_pending = False
            compute_cbs()
        
        key = cv2.waitKey(300) & 0xFF
        if key == 255: continue

        key_char = chr(key)
        if delay_input_mode:
            if key_char.isdigit(): delay_input_buffer += key_char
            elif key == 8: delay_input_buffer = delay_input_buffer[:-1]
            elif key == 13 or key == 10:
                if selected_robot_id is not None and delay_input_buffer.isdigit():
                    delay_val = int(delay_input_buffer)
                    existing = next((a for a in agents if a.id == selected_robot_id), None)
                    if existing: existing.delay = delay_val
                delay_input_mode = False
                delay_input_buffer = ""
        else:
            if key_char.isdigit():
                sid = int(key_char)
                if sid in PRESET_IDS:
                    selected_robot_id = sid
                    print(f"로봇 ID {selected_robot_id} 선택됨.")
            elif key == ord('d') and selected_robot_id in PRESET_IDS:
                delay_input_mode = True
                delay_input_buffer = ""

        if key == ord('q'): break
        elif key == ord('z'):
            print("Reset all"); agents.clear(); paths.clear()
        elif key == ord(' '):
            sim.paused = not sim.paused
            print("Paused" if sim.paused else "Resumed")
        elif key == ord('c'):
            compute_cbs()
        elif key == ord('x'):
            selected_robot_id = None
            delay_input_mode = False
            delay_input_buffer = ""

        # ▼▼▼▼▼ [수정] 'r' 키를 누르면 랜덤 모드 시작 + 자동 목표 할당 + CBS 실행 ▼▼▼▼▼
        elif key == ord('r'):
            if not random_mode_enabled:
                print("랜덤 모드 시작. 초기 목표를 설정하고 CBS를 실행합니다.")
                random_mode_enabled = True
                sim.random_mode_enabled = True

                # 1. 모든 테이블 옆 빈칸을 후보로 수집
                all_table_spots = []
                tables = _tables_with_adjacent_free(grid_array)
                for _, adj_list in tables:
                    all_table_spots.extend(adj_list)
                
                # 2. 후보지가 로봇 수보다 적으면 경고 후 중단
                if len(all_table_spots) < len(agents):
                    print(f"[경고] 목표 후보지({len(all_table_spots)}개)가 로봇 수({len(agents)}개)보다 적어 랜덤 모드를 시작할 수 없습니다.")
                    random_mode_enabled = False # 원상 복구
                    sim.random_mode_enabled = False
                    continue

                # 3. 후보지를 섞어서 각 에이전트에 중복 없이 할당
                random.shuffle(all_table_spots)
                for agent in agents:
                    # 현재 위치와 다른 목표를 할당
                    new_goal = all_table_spots.pop()
                    while new_goal == agent.start and all_table_spots:
                         new_goal = all_table_spots.pop()
                    
                    agent.goal = new_goal
                    robot_next_phase[agent.id] = 'to_home' # 도착하면 home으로 가도록 설정
                    print(f"  -> 로봇 {agent.id} 초기 목표: {agent.goal}")

                # 4. CBS 계산 실행
                compute_cbs()
            else:
                # 이미 켜져 있을 때는 끄는 기능으로 유지 (토글)
                random_mode_enabled = False
                sim.random_mode_enabled = False
                print("랜덤 모드 비활성화.")
        # ▲▲▲▲▲ [수정] 여기까지 ▲▲▲▲▲
            
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()