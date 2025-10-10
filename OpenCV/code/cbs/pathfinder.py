from cbs.cbs_manager import CBSManager

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


    def compute_paths(self, agents: list["Agent"]) -> list["Agent"]:

        """
        agents: List of Agent objects with start, goal, and delay already set.
        Returns:
            List[Agent] with computed paths.
        """
        self.manager.load_instance(self.map_array, agents)
        self.manager.run()
        return self.manager.get_agents()


class Agent:
    def __init__(self, id, start, goal, delay=0):
        self.id = id
        self.start = start
        self.goal = goal
        self.delay = delay
        self.path = []
        self._final_path = None
        self.direction = None

    def set_path(self, path):
        self.path = path
        self._final_path = None

    def get_final_path(self):
        if self._final_path is None:
            if self.delay > 0:
                self._final_path = [self.start] * self.delay + self.path
            else:
                self._final_path = self.path
        return self._final_path

    def __repr__(self):
        return f"Agent(id={self.id}, start={self.start}, goal={self.goal}, delay={self.delay}, path_len={len(self.path)})"
