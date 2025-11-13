from kivy.uix.widget import Widget
from kivy.graphics import Color, Line, Rectangle
from kivy.clock import Clock
from OpenCV.code.config import grid_row, grid_col, cell_size, COLORS
from OpenCV.code.ui_bridge import post, FrameBus

import numpy as np

class GridCanvasWidget(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.grid = np.zeros((grid_row, grid_col), dtype=int)
        self.paths = []
        self.agents = {}
        self.selected_robot_id = None

        # 그리드 렌더링
        Clock.schedule_interval(self.update_canvas, 1/30) # 30 FPS

    def on_touch_down(self, touch):
        if not self.collide_point(*touch.pos):
            return False

        # 좌표 계산
        col = int((touch.x - self.x) // cell_size)
        row = int((touch.y - self.y) // cell_size)

        if 0 <= row < grid_row and 0 <= col < grid_col:
            if self.selected_robot_id:
                post("set_goal", rid=self.selected_robot_id, row=row, col=col)
                print(f"[UI] Grid 클릭: 로봇 {self.selected_robot_id} → 목표 ({row},{col})")
            else:
                print("⚠️ 로봇 선택 필요")
        return True

    def draw_grid(self):
        """격자 선 및 기본 배경"""
        with self.canvas:
            Color(1, 1, 1)
            Rectangle(pos=self.pos, size=self.size)

            Color(0.8, 0.8, 0.8)
            for i in range(grid_row + 1):
                y = self.y + i * cell_size
                Line(points=[self.x, y, self.x + grid_col * cell_size, y], width=1)
            for j in range(grid_col + 1):
                x = self.x + j * cell_size
                Line(points=[x, self.y, x, self.y + grid_row * cell_size], width=1)

    def draw_agents(self):
        """현재 agent 위치를 원과 텍스트로 표시"""
        for aid, (r, c) in self.agents.items():
            color = COLORS[aid % len(COLORS)]
            Color(*[v/255 for v in color])
            x = self.x + c * cell_size + cell_size/2
            y = self.y + r * cell_size + cell_size/2
            Rectangle(pos=(x - 5, y - 5), size=(10, 10))

    def draw_paths(self):
        """CBS 경로를 색깔로 칠하기"""
        for idx, path in enumerate(self.paths):
            color = COLORS[idx % len(COLORS)]
            Color(*[v/255 for v in color], 0.3)
            for (r, c) in path:
                x = self.x + c * cell_size
                y = self.y + r * cell_size
                Rectangle(pos=(x, y), size=(cell_size, cell_size))

    def update_canvas(self, dt):
        self.agents = FrameBus.get_agent_states()
        self.paths = FrameBus.get_paths()
        self.grid = FrameBus.get_grid_state()
        self.canvas.clear()
        self.draw_grid()
        self.draw_paths()
        self.draw_agents()
