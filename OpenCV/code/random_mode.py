import random
from typing import Callable, List, Dict, Tuple, Set
from cbs.pathfinder import PathFinder, Agent

# cbs_tester4.py에서 가져온 헬퍼 함수
def _neighbors4(r, c, H, W):
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        rr, cc = r + dr, c + dc
        if 0 <= rr < H and 0 <= cc < W:
            yield rr, cc

def _tables_with_adjacent_free(grid):
    H, W = grid.shape
    out = []
    for r in range(H):
        for c in range(W):
            if grid[r, c] != 0:  # 장애물 셀이 테이블이라고 가정
                adj = [(rr, cc) for (rr, cc) in _neighbors4(r, c, H, W) if grid[rr, cc] == 0]
                if adj:
                    out.append(((r, c), adj))
    return out

class RandomModeManager:
    """
    cbs_tester4의 랜덤 목적지/대기 로직을 실제 로봇 제어와 연동하기 위한 관리 클래스.
    main.py에서 생성되어 사용됩니다.
    """
    def __init__(self, robot_controller, agents_ref: List[Agent], paths_ref: List, get_grid_func: Callable, get_tag_info_func: Callable, path_to_commands_func: Callable, get_initial_hd_func: Callable, home_positions: dict):
        # 외부 의존성 주입 (main.py의 객체 및 함수)
        self.controller = robot_controller
        self.agents_ref = agents_ref      # main.py의 agents 리스트 참조
        self.paths_ref = paths_ref        # main.py의 paths 리스트 참조
        self.get_grid = get_grid_func
        self.get_tag_info = get_tag_info_func
        self.path_to_commands = path_to_commands_func
        self.get_initial_hd = get_initial_hd_func


        # 랜덤 모드 상태 변수
        self.is_active = False
        self.robot_next_phase = {}
        self.waiting_robots = {} # { robot_id: frames_to_wait }
        self.is_first_run = True
        self.immune_to_delay = set()
        self.is_replan_scheduled = False
        self.ROBOT_HOME_POSITIONS = home_positions
        

        
    def on_robot_complete(self, robot_id: str):
        """개별 로봇이 최종 목적지에 도착했을 때 컨트롤러로부터 호출됩니다."""
        if not self.is_active or self.is_replan_scheduled:
            return

        print(f"🎉 로봇 {robot_id} 최종 목적지 도착! 전체 재계획을 예약합니다.")
        # 다른 로봇이 연달아 도착하더라도 재계획이 중복 실행되지 않도록 플래그 설정
        self.is_replan_scheduled = True
        
        # 컨트롤러에게 현재 진행중인 시퀀스를 중단시켜 다른 로봇들을 멈추게 함
        self.controller.stop_sequence()
        
        # 컨트롤러가 중단되면 check_all_completed가 True가 되고, on_sequence_complete가 호출됨
        # on_sequence_complete를 직접 호출하여 즉시 재계획을 시작하게 할 수도 있음
        self.on_sequence_complete()
        
        
    def on_sequence_complete(self):
        """RobotController로부터 모든 시퀀스가 완료되었다는 신호를 받았을 때 호출됩니다."""
        if not self.is_active:
            return
        self.is_replan_scheduled = False

        # 이전에 update 함수에 있던 재계획 로직을 이곳으로 옮깁니다.
        tag_info = self.get_tag_info()
        for agent in self.agents_ref:
            if agent.id in tag_info and "grid_position" in tag_info[agent.id]:
                agent.start = tag_info[agent.id]["grid_position"]

        print("\n[Random Mode] 시퀀스 완료! 새 목표 할당 및 재계획을 시작합니다.")
        self._assign_new_goals_and_replan()

    def toggle_mode(self):
        """랜덤 모드를 켜고 끕니다."""
        self.is_active = not self.is_active
        if self.is_active:
            # ▼▼▼▼▼ [수정] 이 부분을 수정합니다 ▼▼▼▼▼

            # [기존 코드]
            # print("🤖 [Random Mode] 활성화. 모든 로봇의 작업이 완료되면 자동으로 새 목표를 할당합니다.")
            # # 모드 활성화 시, 현재 로봇 상태를 즉시 시작점으로 갱신
            # tag_info = self.get_tag_info()
            # for agent in self.agents_ref:
            #     if agent.id in tag_info and "grid_position" in tag_info[agent.id]:
            #         agent.start = tag_info[agent.id]["grid_position"]
            # self.update() # 활성화 즉시 첫 계획 시도
            
            # [수정 후 코드]
            print("🤖 [Random Mode] 활성화. 첫 번째 목표를 할당합니다.")
            # 모드 활성화 시, 첫 작업을 시작하기 위해 on_sequence_complete를 직접 호출합니다.
            self.on_sequence_complete()
            # ▲▲▲▲▲ [수정] 여기까지 ▲▲▲▲▲
        else:
            print("🛑 [Random Mode] 비활성화.")
            self.waiting_robots.clear()
            self.immune_to_delay.clear()

    def update(self):
        """main.py의 메인 루프에서 매 프레임 호출되어야 하는 함수."""
        # ▼▼▼▼▼ [수정] 이제 이 함수는 '딜레이' 관리만 담당하도록 대폭 단순화됩니다 ▼▼▼▼▼
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
        
        # 딜레이 대기가 막 끝났을 때만 재계획을 호출합니다.
        if replan_needed_after_wait:
            tag_info = self.get_tag_info()
            for agent in self.agents_ref:
                if agent.id in tag_info and "grid_position" in tag_info[agent.id]:
                    agent.start = tag_info[agent.id]["grid_position"]
            
            print("\n[Random Mode] 대기 완료, 기존 목표로 경로 계산을 시작합니다.")
            self._compute_and_send_paths()

    def _assign_new_goals_and_replan(self):
        """도착한 로봇에게 새 목표를 할당하고, CBS 경로 계산 및 실제 명령 전송을 수행합니다."""
        agents = self.agents_ref

        for agent in agents:
            # 대기중인 로봇은 목표를 새로 할당하지 않음
            if agent.id not in self.waiting_robots:
                self._generate_next_goal_for(agent)

        self._compute_and_send_paths()

    def _generate_next_goal_for(self, agent: Agent):
        """특정 에이전트에 대한 다음 목표 지점을 생성합니다."""
        grid = self.get_grid()
        agents = self.agents_ref
        
        phase = self.robot_next_phase.get(agent.id, 'to_table')
        goals_others = {a.goal for a in agents if a.id != agent.id and a.goal}
        
        # 현재 다른 로봇들이 점유한 셀 파악
        occupied_now = set()
        tag_info = self.get_tag_info()
        for rid, data in tag_info.items():
            if data.get("status") == "On" and "grid_position" in data:
                occupied_now.add(data["grid_position"])

        new_goal = None
        if phase == 'to_table':
            tables = _tables_with_adjacent_free(grid)
            if not tables:
                print(f"[경고] 사용 가능한 테이블(장애물)이 없습니다.")
                return
                
            available_tables = [tbl for (tbl, adj) in tables if tbl not in goals_others]
            if not available_tables:
                available_tables = [tbl for (tbl, adj) in tables]
            
            if available_tables:
                table_pos = random.choice(available_tables)
                adj_list = next(adj for tbl, adj in tables if tbl == table_pos)
                # 다른 로봇의 목표 지점이나 현재 위치가 아닌 곳을 후보로 선택
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

        # 홈에서 출발하는 로봇에게 랜덤 딜레이 부여
        home_positions = set(self.ROBOT_HOME_POSITIONS.values())
        for agent in agents:
            if agent.start in home_positions and agent.goal and agent.goal not in home_positions and agent.id not in self.immune_to_delay:
                # 첫 실행 시에는 딜레이 없음
                delay = random.randint(0, 3) if not self.is_first_run else 0
                if delay > 0:
                    # 약 0.3초 * delay 만큼 대기 (30fps 기준)
                    self.waiting_robots[agent.id] = delay * 10 
                    print(f"  -> 로봇 {agent.id} (홈 출발): {self.waiting_robots[agent.id]} 프레임 대기시간 부여.")

        if self.is_first_run:
            self.is_first_run = False
        self.immune_to_delay.clear()

        # 실제 움직일 로봇과 대기할 로봇 분리
        moving_agents = [a for a in agents if a.id not in self.waiting_robots and a.start and a.goal and a.start != a.goal]
        waiters = [a for a in agents if a.id in self.waiting_robots]
        
        if not moving_agents:
            print("[Random Mode] 움직일 로봇이 없습니다. 대기 로봇의 턴을 기다립니다.")
            return

        # 대기 로봇을 임시 장애물로 처리한 그리드 생성
        grid_with_obstacles = grid.copy()
        for w in waiters:
            if w.start:
                r, c = w.start
                grid_with_obstacles[r, c] = 1

        pathfinder = PathFinder(grid_with_obstacles)
        solved_agents = pathfinder.compute_paths(moving_agents)

        # 시각화를 위해 main.py의 paths 리스트 업데이트
        self.paths_ref.clear()
        for solved_agent in solved_agents:
            final_path = solved_agent.get_final_path()
            if final_path:
                self.paths_ref.append(final_path)

        # 실제 명령 생성 및 전송
        cmd_map: Dict[str, List[str]] = {}
        step_cell_plan: Dict[int, Dict[str, Dict]] = {}

        for agent in solved_agents:
            raw_path = agent.get_final_path()
            if not raw_path or len(raw_path) < 2: continue

            hd0 = self.get_initial_hd(agent.id)
            command_set = [c["command"] for c in self.path_to_commands(raw_path, hd0)]
            
            if command_set:
                cmd_map[str(agent.id)] = command_set

                for i in range(len(raw_path) - 1):
                    step_cell_plan.setdefault(i, {})
                    step_cell_plan[i][str(agent.id)] = {"src": tuple(raw_path[i]), "dst": tuple(raw_path[i + 1])}

        if cmd_map:
            print("[Random Mode] 계산된 경로를 로봇에게 전송합니다:", cmd_map)
            self.controller.start_sequence(cmd_map, step_cell_plan=step_cell_plan)
        else:
            print("[Random Mode] 유효한 경로가 생성되지 않았습니다.")