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
        if self._final_path is None:  # 아직 계산 안 했으면
            if self.delay > 0:
                self._final_path = [self.start] * self.delay + self.path
            else:
                self._final_path = self.path
        return self._final_path

    def __repr__(self):
        return f"Agent(id={self.id}, start={self.start}, goal={self.goal}, delay={self.delay}, path_len={len(self.path)})"
