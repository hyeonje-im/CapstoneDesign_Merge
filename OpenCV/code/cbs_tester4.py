import sys
import os
import random
from collections import deque
import time # 시간 관련 기능을 위해 추가

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


class RandomModeManager:
    """
    cbs_tester4의 랜덤 목적지/대기 로직을 실제 로봇 제어와 연동하기 위한 관리 클래스.
    main.py에서 생성되어 사용됩니다.
    """
    ### 1. [수정] 생성자: main.py의 agents와 paths 리스트를 직접 참조하도록 변경
    def __init__(self, robot_controller, agents_ref, paths_ref, get_grid_func, get_tag_info_func, path_to_commands_func, get_initial_hd_func):
        # 외부 의존성 주입 (main.py의 객체 및 함수)
        self.controller = robot_controller
        self.agents_ref = agents_ref      # agents 리스트 참조
        self.paths_ref = paths_ref        # paths 리스트 참조
        self.get_grid = get_grid_func
        self.get_tag_info = get_tag_info_func
        self.path_to_commands = path_to_commands_func
        self.get_initial_hd = get_initial_hd_func

        # 랜덤 모드 상태 변수
        self.is_active = False
        self.robot_next_phase = {}
        self.waiting_robots = {}
        self.is_first_run = True
        self.immune_to_delay = set()
        
        self.ROBOT_HOME_POSITIONS = {
            1: (0, 0), 2: (2, 0), 3: (0, 5)
        }

    def toggle_mode(self):
        """랜덤 모드를 켜고 끕니다."""
        self.is_active = not self.is_active
        if self.is_active:
            print("🤖 [Random Mode] 활성화. 모든 로봇의 작업이 완료되면 자동으로 새 목표를 할당합니다.")
            # 모드 활성화 시, 현재 로봇 상태를 즉시 시작점으로 갱신
            tag_info = self.get_tag_info()
            for agent in self.agents_ref:
                if agent.id in tag_info and "grid_position" in tag_info[agent.id]:
                    agent.start = tag_info[agent.id]["grid_position"]
            self.update() # 활성화 즉시 첫 계획 시도
        else:
            print("🛑 [Random Mode] 비활성화.")
            self.waiting_robots.clear()
            self.immune_to_delay.clear()

    def update(self):
        """main.py의 메인 루프에서 매 프레임 호출되어야 하는 함수."""
        if not self.is_active:
            return

        replan_needed_after_wait = False
        if self.waiting_robots:
            for r_id in list(self.waiting_robots.keys()):
                self.waiting_robots[r_id] -= 1
                if self.waiting_robots[r_id] <= 0:
                    print(f"  -> 로봇 {r_id} 대기 완료! 재탐색을 준비합니다.")
                    self.immune_to_delay.add(r_id)
                    del self.waiting_robots[r_id]
                    replan_needed_after_wait = True
        
        if replan_needed_after_wait or (self.is_active and self.controller.check_all_completed()):
            tag_info = self.get_tag_info()
            for agent in self.agents_ref:
                if agent.id in tag_info and "grid_position" in tag_info[agent.id]:
                    agent.start = tag_info[agent.id]["grid_position"]

            print("\n[Random Mode] 새 목표 할당 및 재계획을 시작합니다.")
            self._assign_new_goals_and_replan()


    def _assign_new_goals_and_replan(self):
        """
        도착한 로봇에게 새 목표를 할당하고, CBS 경로 계산 및 실제 명령 전송을 수행합니다.
        """
        agents = self.agents_ref # 참조된 리스트 사용

        for agent in agents:
            if agent.id not in self.waiting_robots:
                self._generate_next_goal_for(agent)

        self._compute_and_send_paths()

    def _generate_next_goal_for(self, agent):
        """특정 에이전트에 대한 다음 목표 지점을 생성합니다."""
        grid = self.get_grid()
        agents = self.agents_ref
        
        phase = self.robot_next_phase.get(agent.id, 'to_table')
        goals_others = {a.goal for a in agents if a.id != agent.id and a.goal}
        
        occupied_now = set()
        tag_info = self.get_tag_info()
        for rid, data in tag_info.items():
            if data.get("status") == "On" and "grid_position" in data:
                occupied_now.add(data["grid_position"])

        new_goal = None
        if phase == 'to_table':
            tables = _tables_with_adjacent_free(grid)
            if not tables:
                print(f"[경고] 사용 가능한 테이블이 없습니다.")
                return
                
            available_tables = [tbl for (tbl, adj) in tables if tbl not in goals_others]
            if not available_tables: available_tables = [tbl for (tbl, adj) in tables]
            
            if available_tables:
                table_pos = random.choice(available_tables)
                adj_list = next(adj for tbl, adj in tables if tbl == table_pos)
                candidates = [p for p in adj_list if p not in goals_others and p not in occupied_now]
                if candidates:
                    new_goal = random.choice(candidates)
            
            if new_goal: self.robot_next_phase[agent.id] = 'to_home'

        else: # 'to_home' phase
            if agent.id in self.ROBOT_HOME_POSITIONS:
                home_pos = self.ROBOT_HOME_POSITIONS[agent.id]
                if home_pos not in goals_others and home_pos not in occupied_now:
                    new_goal = home_pos
            
            if new_goal: self.robot_next_phase[agent.id] = 'to_table'

        if new_goal:
            print(f"[Random Mode] 로봇 {agent.id}의 새 목표: {new_goal} (다음 단계: {self.robot_next_phase.get(agent.id)})")
            agent.goal = new_goal
        else:
            # 목표 설정 실패 시 현재 위치를 목표로 설정하여 대기
            agent.goal = agent.start
            print(f"[Random Mode] 로봇 {agent.id}의 새 목표를 찾지 못해 현재 위치({agent.start})에서 대기합니다.")

    def _compute_and_send_paths(self):
        """CBS를 계산하고 RobotController를 통해 실제 명령을 전송합니다."""
        grid = self.get_grid()
        agents = self.agents_ref

        home_positions = set(self.ROBOT_HOME_POSITIONS.values())
        for agent in agents:
            if agent.start in home_positions and agent.goal and agent.goal not in home_positions and agent.id not in self.immune_to_delay:
                delay = random.randint(0, 3) if not self.is_first_run else 0
                if delay > 0:
                    self.waiting_robots[agent.id] = delay * 10 # 프레임 기반 대기 (약 0.3초)
                    print(f"  -> 로봇 {agent.id} (홈 출발): {self.waiting_robots[agent.id]} 프레임 대기시간 부여.")

        if self.is_first_run:
            self.is_first_run = False
        self.immune_to_delay.clear()

        moving_agents = [a for a in agents if a.id not in self.waiting_robots and a.start and a.goal]
        waiters = [a for a in agents if a.id in self.waiting_robots]
        
        if not moving_agents:
            print("[Random Mode] 움직일 로봇이 없습니다. 대기 로봇의 턴을 기다립니다.")
            return

        grid_with_obstacles = grid.copy()
        for w in waiters:
            if w.start:
                r, c = w.start
                grid_with_obstacles[r, c] = 1

        pathfinder = PathFinder(grid_with_obstacles)
        solved_agents = pathfinder.compute_paths(moving_agents)

        # 시각화를 위해 main.py의 paths와 agents 리스트를 업데이트
        self.paths_ref.clear()
        agent_map = {a.id: a for a in agents}
        for solved_agent in solved_agents:
            if solved_agent.id in agent_map:
                agent_map[solved_agent.id].start = solved_agent.start
                agent_map[solved_agent.id].goal = solved_agent.goal
            
            final_path = solved_agent.get_final_path()
            if final_path:
                self.paths_ref.append(final_path)

        # 실제 명령 생성 및 전송
        payload_commands = []
        step_cell_plan = {}
        for agent in solved_agents:
            raw_path = agent.get_final_path()
            if not raw_path: continue

            hd0 = self.get_initial_hd(agent.id)
            command_set = [c["command"] for c in self.path_to_commands(raw_path, hd0)]

            payload_commands.append({"robot_id": str(agent.id), "command_set": command_set})
            
            for i in range(len(raw_path) - 1):
                step_cell_plan.setdefault(i, {})
                step_cell_plan[i][str(agent.id)] = {"src": tuple(raw_path[i]), "dst": tuple(raw_path[i + 1])}

        cmd_map = {p["robot_id"]: p["command_set"] for p in payload_commands}
        if cmd_map:
            print("[Random Mode] 계산된 경로를 로봇에게 전송합니다:", {k: v for k, v in cmd_map.items() if v})
            # 경로가 없는 (Stay) 로봇은 전송에서 제외
            self.controller.start_sequence({k: v for k, v in cmd_map.items() if v}, step_cell_plan=step_cell_plan)
        else:
            print("[Random Mode] 유효한 경로가 생성되지 않았습니다.")



# 전역 변수
ROBOT_HOME_POSITIONS = {
    1: (0, 0),
    2: (2,0),
    3: (0, 5)
}

agents = []
paths = []
sim = None
broker = FakeMQTTBroker()
pathfinder = None
grid_array = None
selected_robot_id = None

pending_steps = {}          # { robot_id: deque([(r,c), ...]) }

replan_paused = False
replan_pause_end_time = 0


delay_input_mode = False
delay_input_buffer = ""

random_mode_enabled = False
robot_next_phase = {}
replan_pending = False
PRESET_IDS = [0,1,2,3,4,5,6,7,8,9]
waiting_robots = {}  # { robot_id: delay_steps_remaining }
is_first_run = True  # 프로그램 첫 실행 여부를 확인하는 플래그
immune_to_delay = set() # 방금 대기가 끝나서 딜레이 부여에 면역이 된 로봇 ID 목록

def mouse_event(event, x, y, flags, param):
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
        
        for agent in agents:
            if agent.id == selected_robot_id:
                agent.goal = (row, col)
                updated = True
                print(f"Agent {agent.id}의 도착지를 ({row}, {col})로 변경")
                break
        
        if not updated:
            agent = Agent(id=selected_robot_id, start=None, goal=(row, col), delay=0)
            agents.append(agent)
            updated = True

        selected_robot_id = None
        
        if updated:
            print(f"Agent {selected_robot_id} 준비 완료. CBS 실행을 위해 'c'를 누르세요.")

        return

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
    cur_pos = tuple(map(int, sim.robots[robot_id].get_position()))

    # --- [핵심 수정] ---
    # "대기 건너뛰기" 로직을 제거하고, "대기"를 하나의 명확한 행동으로 처리합니다.
    
    # 1. 다음 행동을 큐에서 꺼냅니다.
    target_step = pending_steps[robot_id].popleft()

    # 2. 이동 명령인지, 대기 명령인지 확인합니다.
    is_move_command = (target_step[0] != cur_pos[0]) or (target_step[1] != cur_pos[1])
    
    # 3. 이동 명령인데 한 칸 이동이 아니면 오류 처리 (기존 로직)
    if is_move_command and abs(target_step[0]-cur_pos[0]) + abs(target_step[1]-cur_pos[1]) != 1:
        print(f"[WARN] 비정상적인 스텝 시도: {cur_pos}->{target_step}. 명령을 무시합니다.")
        pending_steps[robot_id].appendleft(target_step) 
        return False
        
    # 4. 시뮬레이터에 명령 전송 (대기 명령일 경우 cur_pos -> target_step(==cur_pos) 으로 전송됨)
    cs = CommandSet(str(robot_id), [cur_pos, target_step], initial_dir=_expected_dir(robot))
    broker.send_command_sets([cs])
    return True

def execute_next_synchronized_step():
    global replan_pending, waiting_robots, immune_to_delay

    if replan_pending:
        return
    if replan_paused:
        return
    if any(robot.moving or robot.rotating for robot in sim.robots.values()):
        return

    # 대기 로봇 타이머 감소 로직
    replan_needed_after_wait = False
    if waiting_robots:
        for r_id in list(waiting_robots.keys()):
            waiting_robots[r_id] -= 1
            if waiting_robots[r_id] <= 0:
                print(f"  -> 로봇 {r_id} 대기 완료! 전체 재탐색을 준비합니다.")
                # ▼▼▼▼▼ [핵심 수정] 대기 완료된 로봇을 '면역' 목록에 추가 ▼▼▼▼▼
                immune_to_delay.add(r_id)
                # ▲▲▲▲▲ [핵심 수정] 여기까지 ▲▲▲▲▲
                del waiting_robots[r_id]
                replan_needed_after_wait = True
    
    if replan_needed_after_wait:
        replan_pending = True
        return # 즉시 재탐색 상태로 전환하고, 이번 턴에는 움직이지 않음

    # 기존의 이동 명령 전송 로직
    for robot_id in sim.robots.keys():
        if robot_id in pending_steps and pending_steps[robot_id]:
            send_next_step(robot_id)

# ▼▼▼▼▼ [핵심 수정] "대기 명령"이 누락되지 않도록 수정한 함수 ▼▼▼▼▼
def expand_to_unit_steps(path):
    out = []
    if not path: return out
    out.append(path[0]) # 시작점은 항상 추가
    
    for i in range(len(path) - 1):
        r1, c1 = path[i]
        r2, c2 = path[i+1]

        # 만약 시작점과 도착점이 같다면 (제자리 대기 명령)
        if (r1, c1) == (r2, c2):
            out.append((r2, c2)) # 대기 위치를 경로에 명시적으로 추가
            continue # 다음 루프로 이동

        # 기존의 이동 경로 확장 로직
        dr = 0 if r2 == r1 else (1 if r2 > r1 else -1)
        dc = 0 if c2 == c1 else (1 if c2 > c1 else -1)
        
        if dr != 0 and dc != 0: 
            raise ValueError(f"Diagonal segment in path: {path[i]}->{path[i+1]}")
            
        rr, cc = r1, c1
        while (rr, cc) != (r2, c2):
            rr += dr
            cc += dc
            out.append((rr, cc))
            
    return out


def compute_cbs(is_initial_plan=False):
    global paths, pathfinder, grid_array, pending_steps, waiting_robots, replan_paused, replan_pause_end_time, is_first_run, immune_to_delay
    
    print("\nCBS 경로 계산 시작...")
    grid_array = load_grid(grid_row, grid_col)
    get_start_from_robot()

    moving_agents = []
    temp_obstacles = []

    if is_initial_plan:
        waiting_robots.clear()

    # ▼▼▼▼▼ [핵심 수정] '면역' 목록에 있는 로봇은 대기시간 부여에서 제외 ▼▼▼▼▼
    home_positions = set(ROBOT_HOME_POSITIONS.values())
    for agent in agents:
        is_starting_from_home = agent.start in home_positions and \
                                (not agent.goal or agent.goal not in home_positions) and \
                                agent.id not in waiting_robots

        # 면역 목록에 있는 로봇인지 확인하는 조건 추가
        if is_starting_from_home and agent.id not in immune_to_delay:
            if is_first_run:
                agent.delay = 0
            else:
                agent.delay = random.randint(0, 3)
                if agent.delay > 0:
                    waiting_robots[agent.id] = agent.delay
                    print(f"  -> 로봇 {agent.id} (홈 출발): {agent.delay} 스텝 대기시간 부여.")
        else:
            agent.delay = 0
    
    if is_first_run and is_initial_plan:
        print("   - 첫 실행이므로 모든 로봇의 대기시간을 0으로 강제합니다.")
        is_first_run = False
    
    # 재탐색이 끝나면 면역 목록을 비워 다음을 준비
    immune_to_delay.clear()
    # ▲▲▲▲▲ [핵심 수정] 여기까지 ▲▲▲▲▲

    for agent in agents:
        if agent.id in waiting_robots:
            temp_obstacles.append(agent.start)
            print(f"   - 로봇 {agent.id} at {agent.start} ... 임시 장애물로 처리됩니다.")
        elif agent.start and agent.goal and agent.start != agent.goal:
            moving_agents.append(agent)
        
    grid_with_temp_obstacles = grid_array.copy()
    for r, c in temp_obstacles:
        if 0 <= r < grid_row and 0 <= c < grid_col:
            grid_with_temp_obstacles[r, c] = 1
    pathfinder = PathFinder(grid_with_temp_obstacles)

    if not moving_agents:
        print("움직일 로봇이 없습니다. 대기 로봇의 턴을 기다립니다.")
        paths.clear()
        pending_steps.clear()
        replan_paused = True
        replan_pause_end_time = time.time()
        return

    print(f"CBS 계산 대상: {len(moving_agents)}대의 '움직이는' 로봇")
    
    for agent in moving_agents:
        agent.delay = 0

    try:
        new_agents = pathfinder.compute_paths(moving_agents)
    except Exception as e:
        print(f"[CBS] 예외 발생: {e}. 이번 턴을 스킵하고 재시도합니다.")
        paths.clear(); pending_steps.clear()
        replan_paused = True
        replan_pause_end_time = time.time()
        return

    if not new_agents:  # None 또는 빈 리스트
        print("[CBS] 경로 계산 결과가 없습니다(None/empty). 이번 턴을 스킵하고 재시도합니다.")
        paths.clear(); pending_steps.clear()
        replan_paused = True
        replan_pause_end_time = time.time()
        return

    new_paths_map = {agent.id: agent.get_final_path() for agent in new_agents}
    
    if not new_paths_map and moving_agents:
        print("경로를 찾지 못했습니다. (임시 장애물 또는 다른 로봇에 의해 막혔을 수 있습니다)")
        return
    
    final_paths = []
    final_pending_steps = {}
    agent_id_to_agent_map = {a.id: a for a in agents}
    for agent_id, agent in agent_id_to_agent_map.items():
        if agent_id in waiting_robots:
            final_paths.append([])
        elif agent_id in new_paths_map:
            path = new_paths_map[agent_id]
            final_paths.append(path)
            if path:
                unit_steps = expand_to_unit_steps(path)
                final_pending_steps[agent_id] = deque(unit_steps[1:])
            sim.robot_info[agent_id]['path'] = path
            sim.robot_info[agent_id]['goal'] = agent.goal
    
    paths.clear(); paths.extend(final_paths)
    pending_steps.clear(); pending_steps.update(final_pending_steps)

    print("경로 계산 완료.")
    replan_paused = True
    replan_pause_end_time = time.time()


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
    color = (220, 220, 220)
    for pos in positions_dict.values():
        r, c = pos
        x, y = c * cell_size, r * cell_size
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
        cv2.addWeighted(overlay, 0.5, vis_img, 0.5, 0, vis_img)
           

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

def on_robot_arrival(robot_id, pos):
    global agents, sim, robot_next_phase, replan_pending, grid_array
    if not random_mode_enabled: return
    
    if pending_steps.get(robot_id):
        return

    pos = tuple(map(int, pos))
    ag = next((a for a in agents if a.id == robot_id), None)
    if not ag or not ag.goal or tuple(ag.goal) != pos: return

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
    else:
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

    print(f"\n[랜덤 모드] 로봇 {robot_id} 새 목표 {new_goal} (phase→ {robot_next_phase[robot_id]})")
    for a in agents:
        if a.id == robot_id:
            a.start = pos
            a.goal = new_goal
            break

    get_start_from_robot()
    replan_pending = True
    
def main():
    global agents, paths, grid_array, selected_robot_id, sim, replan_pending
    global delay_input_buffer, delay_input_mode, random_mode_enabled
    global replan_paused, replan_pause_end_time
    
    auto_mode_enabled = False
    last_auto_step_time = 0
    
    # ▼▼▼▼▼ [추가] 턴 사이의 0.1초 대기를 위한 변수 ▼▼▼▼▼
    was_all_robots_idle = True
    idle_since_time = 0
    TURN_COOLDOWN = 0.3 # 0.1초 대기
    # ▲▲▲▲▲ [추가] 여기까지 ▲▲▲▲▲

    grid_array = load_grid(grid_row, grid_col)
    cv2.namedWindow("CBS Grid")
    cv2.setMouseCallback("CBS Grid", mouse_event)

    sim = Simulator(grid_array.astype(bool), colors=COLORS, home_positions=ROBOT_HOME_POSITIONS)
    sim.register_arrival_callback(on_robot_arrival)
    
    for robot_id, start_pos in ROBOT_HOME_POSITIONS.items():
        if robot_id not in PRESET_IDS:
            continue
        sim.add_robot(robot_id, broker, start_pos=start_pos)
        agent = Agent(id=robot_id, start=start_pos, goal=None, delay=0)
        agents.append(agent)
    print(f"{len(agents)}개의 로봇이 초기 위치에 자동 생성되었습니다.")

    while True:
        # --- [기존 시각화 로직은 그대로] ---
        vis = grid_visual(grid_array.copy())
        draw_home_positions(vis, ROBOT_HOME_POSITIONS)
        draw_paths(vis, paths)
        if waiting_robots: # 대기중인 로봇이 있을 때만 실행
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0  # 숫자 크기
            font_thickness = 2 # 숫자 굵기
            text_color = (0, 0, 255) # 텍스트 색상 (빨간색)

            for robot_id, remaining_delay in waiting_robots.items():
                # 로봇의 홈 위치를 찾음
                if robot_id in ROBOT_HOME_POSITIONS:
                    home_pos = ROBOT_HOME_POSITIONS[robot_id]
                    row, col = home_pos

                    # 텍스트 위치 계산 (셀 중앙에 오도록)
                    text = str(remaining_delay)
                    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
                    text_x = (col * cell_size) + (cell_size - text_size[0]) // 2
                    text_y = (row * cell_size) + (cell_size + text_size[1]) // 2

                    # vis 이미지에 텍스트 그리기
                    cv2.putText(vis, text, (text_x, text_y), font, font_scale, text_color, font_thickness)
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
            delay_input_buffer=delay_input_buffer, cell_size=cell_size,
            waiting_robots=waiting_robots
        )
        combined = cv2.hconcat([vis, agent_info_img])
        cv2.imshow("CBS Grid", combined)
        
        sim.run_once()

        # ▼▼▼▼▼ [수정] 쿨다운 타이머 로직 추가 ▼▼▼▼▼
        # 1. 현재 모든 로봇이 멈춰있는지 확인
        all_robots_idle = True
        for robot in sim.robots.values():
            if robot.moving or robot.rotating:
                all_robots_idle = False
                break
        
        # 2. 로봇들이 '움직이다가' -> '막 멈춘' 순간을 감지
        if all_robots_idle and not was_all_robots_idle:
            idle_since_time = time.time() # 멈춘 시간을 기록

        was_all_robots_idle = all_robots_idle # 다음 프레임을 위해 현재 상태 저장
        
        # 3. 자동 진행 로직에 쿨다운 조건 추가
        if auto_mode_enabled and all_robots_idle and (time.time() - idle_since_time >= TURN_COOLDOWN) and (time.time() - last_auto_step_time >= 0.3):
            execute_next_synchronized_step()
            last_auto_step_time = time.time()
        # ▲▲▲▲▲ [수정] 여기까지 ▲▲▲▲▲

        if replan_paused and time.time() > replan_pause_end_time:
            replan_paused = False
            print("일시정지 해제. 스페이스바 또는 'a' 키를 눌러 진행하세요.")
        
        if replan_pending and all_robots_idle:
            if is_safe_to_replan_immediately := all(not r.moving and not r.rotating for r in sim.robots.values()):
                replan_pending = False
                compute_cbs(is_initial_plan=False) # 대기 완료 후 재탐색

        key = cv2.waitKey(30) & 0xFF
        if key == 255: continue

        key_char = chr(key)
        # ... (delay_input_mode 관련 로직은 그대로) ...
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
        
        # ▼▼▼▼▼ [수정] 스페이스바 키 처리 로직에 쿨다운 조건 추가 ▼▼▼▼▼
        elif key == ord(' '):
            if all_robots_idle and (time.time() - idle_since_time >= TURN_COOLDOWN):
                if auto_mode_enabled:
                    auto_mode_enabled = False
                    print("자동 진행 모드 비활성화.")
                execute_next_synchronized_step()
            else:
                print("아직 로봇이 움직이고 있거나 턴 사이 대기 중입니다.")
        # ▲▲▲▲▲ [수정] 여기까지 ▲▲▲▲▲
        
        elif key == ord('a'):
            auto_mode_enabled = not auto_mode_enabled
            if auto_mode_enabled:
                print("자동 진행 모드 활성화 (0.3초 간격).")
                last_auto_step_time = time.time()
                idle_since_time = 0 # 자동 모드 시작 시 쿨다운 초기화
            else:
                print("자동 진행 모드 비활성화.")
        
        # ... (나머지 키 처리 로직은 그대로) ...
        elif key == ord('c'):
            compute_cbs(is_initial_plan=True) # 수동으로 초기 계획 시작
        elif key == ord('x'):
            selected_robot_id = None
            delay_input_mode = False
            delay_input_buffer = ""
        elif key == ord('r'):
            if not random_mode_enabled:
                print("랜덤 모드 시작. 초기 목표를 설정하고 CBS를 실행합니다.")
                random_mode_enabled = True
                sim.random_mode_enabled = True
                all_table_spots = []
                tables = _tables_with_adjacent_free(grid_array)
                for _, adj_list in tables:
                    all_table_spots.extend(adj_list)
                if len(all_table_spots) < len(agents):
                    print(f"[경고] 목표 후보지({len(all_table_spots)}개)가 로봇 수({len(agents)}개)보다 적어 랜덤 모드를 시작할 수 없습니다.")
                    random_mode_enabled = False
                    sim.random_mode_enabled = False
                    continue
                random.shuffle(all_table_spots)
                for agent in agents:
                    new_goal = all_table_spots.pop()
                    while new_goal == agent.start and all_table_spots:
                         new_goal = all_table_spots.pop()
                    agent.goal = new_goal
                    robot_next_phase[agent.id] = 'to_home'
                    print(f"  -> 로봇 {agent.id} 초기 목표: {agent.goal}")
                compute_cbs(is_initial_plan=True) # 랜덤 모드 첫 실행
            else:
                random_mode_enabled = False
                sim.random_mode_enabled = False
                print("랜덤 모드 비활성화.")
            
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()