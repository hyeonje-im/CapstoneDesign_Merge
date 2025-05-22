from cbs.cbs_manager import CBSManager
from cbs.agent import Agent

import numpy as np
import random

class PathFinder:
    def __init__(self, grid_array: np.ndarray):
        self.grid = grid_array
        self.map_array = self.grid.astype(bool)
        self.rows, self.cols = self.grid.shape
        self.valid_cells = [
            (r, c) for r in range(self.rows) for c in range(self.cols) if self.grid[r, c] == 0
        ]
        self.manager = CBSManager(solver_type="CBS", disjoint=True, visualize_result=False)


    def compute_paths(self, agents: list[Agent]) -> list[Agent]:
        """
        agents: List of Agent objects with start, goal, and delay already set.
        Returns:
            List[Agent] with computed paths.
        """
        self.manager.load_instance(self.map_array, agents)
        self.manager.run()
        return self.manager.get_agents()






















    # def generate_commands(self, agents: list[Agent], initial_dir: str = "north") -> dict[int, list[str]]:
    #     """
    #     Converts agent paths to movement commands.
    #     Returns:
    #         dict[agent_id → List[str]] (e.g. ["D20", "L", "D10"])
    #     """
    #     result = {}
    #     for agent in agents:
    #         commands = path_to_commands(agent.get_final_path(), initial_dir)
    #         result[agent.id] = self._compress(commands)
    #     return result

    # def _random_goal(self, avoid: tuple[int, int]) -> tuple[int, int]:
    #     options = [cell for cell in self.valid_cells if cell != avoid]
    #     return random.choice(options) if options else avoid

    # def _compress(self, commands: list[str]) -> list[str]:
    #     compressed = []
    #     prev = None
    #     count = 0
    #     for cmd in commands + [None]:
    #         if cmd == prev:
    #             count += 1
    #         else:
    #             if prev == "forward":
    #                 compressed.append(f"D{count * 10}")
    #             elif prev == "left":
    #                 compressed.extend(["L"] * count)
    #             elif prev == "right":
    #                 compressed.extend(["R"] * count)
    #             count = 1
    #             prev = cmd
    #     return compressed


# def run_cbs_manager(grid_array, tag_info):
#     rows, cols = grid_array.shape

#     start_points = []
#     goal_points = []
#     agent_ids = []

#     valid_cells = []
#     for r in range(rows):
#         for c in range(cols):
#             if grid_array[r, c] == 0:
#                 valid_cells.append((r, c))

#     for idx, (tag_id, data) in enumerate(tag_info.items()):
#         x_cm, y_cm = data["coordinates"]

#         tag_grid_x = int(x_cm * grid_width / board_width_cm)
#         tag_grid_y = int(y_cm * grid_height / board_height_cm)
#         start_col = tag_grid_x // cell_size
#         start_row = tag_grid_y // cell_size

#         start_row = max(0, min(rows - 1, start_row))
#         start_col = max(0, min(cols - 1, start_col))

#         if valid_cells:
#             dest_row, dest_col = random.choice(valid_cells)
#         else:
#             dest_row, dest_col = start_row, start_col

#         start_points.append((start_row, start_col))
#         goal_points.append((dest_row, dest_col))
#         agent_ids.append(tag_id)  # 여기서 tag_id를 agent id로 사용

#     # Step 2: CBSManager를 이용해 CBS 실행
#     map_array = grid_array.astype(bool)  # 0: 이동 가능, 1: 장애물
#     manager = CBSManager(solver_type="ICBS", disjoint=True, visualize_result=False)
#     agents = []
#     for tag_id, start, goal in zip(agent_ids, start_points, goal_points):
#         agent = Agent(id=tag_id, start=start, goal=goal, delay=2)  # ← 예시로 delay 2초
#         agents.append(agent)
#     manager.load_instance(map_array, start_points, goal_points, agent_ids)
#     manager.run()  # -> 내부에서 자동으로 path_relay에 저장

#     # Step 3: 결과 가져오기
#     agents = manager.get_agents()

#     print("\n=== Extracted Agents ===")
#     for agent in agents:
#         print(agent)

#     return agents
