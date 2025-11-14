# ========= GridWidget (Kivy 완성본) =========
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, Ellipse, Triangle
from kivy.clock import Clock
from OpenCV.code.ui_bridge import FrameBus


class GridWidget(Widget):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.rows = 6
        self.cols = 6

        self.bind(pos=self.redraw, size=self.redraw)

        # 지속적으로 백엔드 상태 읽기
        Clock.schedule_interval(self.update_from_backend, 1/30)

    # ====================================================
    # =============== Canvas 업데이트 루틴 ================
    # ====================================================
    def redraw(self, *args):
        self.canvas.clear()

        cell_w = self.width / self.cols
        cell_h = self.height / self.rows

        with self.canvas:
            # 배경
            Color(0.96, 0.97, 0.99)
            Rectangle(pos=self.pos, size=self.size)

            # 그리드선
            Color(0.80, 0.84, 0.90)
            for i in range(self.rows + 1):
                y = self.y + i * cell_h
                Line(points=[self.x, y, self.x + self.width, y], width=1)

            for j in range(self.cols + 1):
                x = self.x + j * cell_w
                Line(points=[x, self.y, x, self.y + self.height], width=1)

    # ====================================================
    # ============== 백엔드 상태 읽고 표시 ================
    # ====================================================
    def update_from_backend(self, dt):
        grid = FrameBus.get_grid_state()
        agents = FrameBus.get_agent_states()
        paths = FrameBus.get_paths()
        homes = FrameBus.get_home_positions()
        goals = FrameBus.get_goal_positions()
        delays = FrameBus.get_delays()
        headings = FrameBus.get_headings()

        if grid is None:
            return

        self.draw_obstacles(grid)
        self.draw_home_positions(homes)
        self.draw_goal_positions(goals)
        self.draw_paths(paths)
        self.draw_agents(agents, headings)
        self.draw_agent_delays(agents, delays)

    # ====================================================
    # =================== 장애물 ==========================
    # ====================================================
    def draw_obstacles(self, grid):
        cell_w = self.width / self.cols
        cell_h = self.height / self.rows

        with self.canvas:
            Color(0.6, 0.6, 0.6, 1)  # 회색
            for r in range(self.rows):
                for c in range(self.cols):
                    if grid[r][c] == 1:
                        Rectangle(
                            pos=(self.x + c * cell_w, self.y + (self.rows - 1 - r) * cell_h),
                            size=(cell_w, cell_h)
                        )

    # ====================================================
    # ================= 출발 위치(Home) ===================
    # ====================================================
    def draw_home_positions(self, homes):
        if not homes:
            return
        cell_w = self.width / self.cols
        cell_h = self.height / self.rows

        with self.canvas:
            Color(0.7, 0.85, 1, 0.35)  # 연한 하늘색
            for rid, (r, c) in homes.items():
                Rectangle(
                    pos=(self.x + c * cell_w, self.y + (self.rows - 1 - r) * cell_h),
                    size=(cell_w, cell_h)
                )

    # ====================================================
    # ================== 도착 위치(Goal) ===================
    # ====================================================
    def draw_goal_positions(self, goals):
        if not goals:
            return
        cell_w = self.width / self.cols
        cell_h = self.height / self.rows

        with self.canvas:
            Color(0.1, 0.3, 1, 0.45)  # 진한 블루
            for rid, (r, c) in goals.items():
                Rectangle(
                    pos=(self.x + c * cell_w, self.y + (self.rows - 1 - r) * cell_h),
                    size=(cell_w, cell_h)
                )

    # ====================================================
    # ==================== 경로 ============================
    # ====================================================
    def draw_paths(self, paths):
        if not paths:
            return
        
        cell_w = self.width / self.cols
        cell_h = self.height / self.rows

        for rid, path in paths.items():
            with self.canvas:
                Color(0, 1, 0, 0.35)
                for (r, c) in path:
                    Rectangle(
                        pos=(self.x + c * cell_w, self.y + (self.rows - 1 - r) * cell_h),
                        size=(cell_w, cell_h)
                    )

    # ====================================================
    # ================== 로봇 위치 =========================
    # ====================================================
    def draw_agents(self, agents, headings):
        if not agents:
            return

        cell_w = self.width / self.cols
        cell_h = self.height / self.rows
        size = min(cell_w, cell_h) * 0.5

        for rid, (r, c) in agents.items():
            cx = self.x + c * cell_w + cell_w / 2
            cy = self.y + (self.rows - 1 - r) * cell_h + cell_h / 2

            h = headings.get(rid, 0)

            with self.canvas:
                Color(1, 0.2, 0.2)
                Ellipse(pos=(cx - size/2, cy - size/2), size=(size, size))

    # ====================================================
    # =================== 딜레이 표시 =======================
    # ====================================================
    def draw_agent_delays(self, agents, delays):
        if not delays:
            return
        
        cell_w = self.width / self.cols
        cell_h = self.height / self.rows

        for rid, d in delays.items():
            (r, c) = agents.get(rid, (None, None))
            if r is None: continue

            cx = self.x + c * cell_w + cell_w * 0.7
            cy = self.y + (self.rows - 1 - r) * cell_h + cell_h * 0.7

            with self.canvas:
                Color(0, 0, 0)
                Rectangle(pos=(cx-8, cy-8), size=(16,16))
