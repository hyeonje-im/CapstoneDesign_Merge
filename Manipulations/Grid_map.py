from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button
from kivy.clock import Clock
from kivy.graphics import Color, RoundedRectangle
from kivy.uix.floatlayout import FloatLayout

from OpenCV.config import grid_row, grid_col
from OpenCV.grid import load_grid, save_grid

class GridMap(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 1. JSON 격자 불러오기
        self.grid_map = load_grid(grid_row, grid_col).tolist()
        self.rows = grid_row
        self.cols = grid_col

        # 2. 격자 박스 배경
        self.grid_container = FloatLayout(
            size_hint=(0.79, 0.64),
            pos_hint={'x': 0, 'y': 260 / 838}
        )
        with self.grid_container.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            self.grid_bg = RoundedRectangle(pos=self.grid_container.pos, size=self.grid_container.size, radius=[5])
        self.grid_container.bind(pos=lambda *a: setattr(self.grid_bg, 'pos', self.grid_container.pos),
                                 size=lambda *a: setattr(self.grid_bg, 'size', self.grid_container.size))

        # 3. 실제 그리드 UI 구성
        self.grid_layout = GridLayout(rows=self.rows, cols=self.cols, spacing=3, size_hint=(1, 1), pos_hint={'x': 0, 'y': 0})
        self.buttons = [[None for _ in range(self.cols)] for _ in range(self.rows)]

        for i in range(self.rows):
            for j in range(self.cols):
                color = [0, 0, 0, 1] if self.grid_map[i][j] == 1 else [1, 1, 1, 1]
                btn = Button(background_normal='', background_color=color)
                btn.bind(on_press=self.on_cell_press(i, j))
                self.buttons[i][j] = btn
                self.grid_layout.add_widget(btn)

        self.grid_container.add_widget(self.grid_layout)
        self.add_widget(self.grid_container)

        self.grid_layout.bind(size=self.update_button_sizes)
        Clock.schedule_once(lambda dt: self.update_button_sizes(), 0)

    def on_cell_press(self, i, j):
        def callback(instance):
            print(f"[INFO] CLicked Block: ({i}, {j}) → 현재 상태: {self.grid_map[i][j]}")
        return callback

    def update_button_sizes(self, *args):
        spacing = self.grid_layout.spacing
        spacing_x = spacing[0] if isinstance(spacing, (list, tuple)) else spacing
        spacing_y = spacing[1] if isinstance(spacing, (list, tuple)) else spacing

        total_spacing_x = (self.cols - 1) * spacing_x
        total_spacing_y = (self.rows - 1) * spacing_y

        available_width = self.grid_layout.width - total_spacing_x
        available_height = self.grid_layout.height - total_spacing_y

        cell_size = min(available_width / self.cols, available_height / self.rows)

        for i in range(self.rows):
            for j in range(self.cols):
                btn = self.buttons[i][j]
                btn.size_hint = (None, None)
                btn.size = (cell_size, cell_size)

        padding_x = (self.grid_layout.width - (cell_size * self.cols + total_spacing_x)) / 2
        padding_y = (self.grid_layout.height - (cell_size * self.rows + total_spacing_y)) / 2

        self.grid_layout.padding = [padding_x, padding_y, padding_x, padding_y]

    def save_grid_state(self):
        import numpy as np
        save_grid(np.array(self.grid_map), self.rows, self.cols)
