from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button
from kivy.clock import Clock
from kivy.graphics import Color, RoundedRectangle
from kivy.uix.floatlayout import FloatLayout

class GridMap(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 1. 그리드를 감싸는 박스
        self.grid_container = FloatLayout(
            size_hint=(0.79, 0.685),
            pos_hint={'x': 0, 'y': 0.265}
        )

        # 2. 배경색 + 라운딩 처리
        with self.grid_container.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            self.grid_bg = RoundedRectangle(pos=self.grid_container.pos, size=self.grid_container.size, radius=[5])
        self.grid_container.bind(pos=lambda *a: setattr(self.grid_bg, 'pos', self.grid_container.pos),
                                 size=lambda *a: setattr(self.grid_bg, 'size', self.grid_container.size))

        # 3. 실제 그리드 레이아웃 (12x12)
        self.grid_layout = GridLayout(rows=12, cols=12, spacing=3, size_hint=(1, 1), pos_hint={'x': 0, 'y': 0})

        self.grid_map = [[0 for _ in range(12)] for _ in range(12)]
        self.buttons = [[None for _ in range(12)] for _ in range(12)]

        for i in range(12):
            for j in range(12):
                btn = Button(background_normal='', background_color=[1, 1, 1, 1])
                btn.bind(on_press=self.on_cell_press(i, j))
                self.buttons[i][j] = btn
                self.grid_layout.add_widget(btn)

        self.grid_container.add_widget(self.grid_layout)
        self.add_widget(self.grid_container)

        self.grid_layout.bind(size=self.update_button_sizes)
        Clock.schedule_once(lambda dt: self.update_button_sizes(), 0)

    def on_cell_press(self, i, j):
        def callback(instance):
            self.grid_map[i][j] = 1 if self.grid_map[i][j] == 0 else 0
            self.buttons[i][j].background_color = [0, 0, 0, 1] if self.grid_map[i][j] == 1 else [1, 1, 1, 1]
            print(f"Selected Block: ({i}, {j}) → 상태: {self.grid_map[i][j]}")
        return callback

    def update_button_sizes(self, *args):
        spacing = self.grid_layout.spacing
        spacing_x = spacing[0] if isinstance(spacing, (list, tuple)) else spacing
        spacing_y = spacing[1] if isinstance(spacing, (list, tuple)) else spacing

        total_spacing_x = (self.grid_layout.cols - 1) * spacing_x
        total_spacing_y = (self.grid_layout.rows - 1) * spacing_y

        available_width = self.grid_layout.width - total_spacing_x
        available_height = self.grid_layout.height - total_spacing_y

        cell_size = min(available_width / 12, available_height / 12)

        for i in range(12):
            for j in range(12):
                btn = self.buttons[i][j]
                btn.size_hint = (None, None)
                btn.size = (cell_size, cell_size)

        padding_x = (self.grid_layout.width - (cell_size * 12 + total_spacing_x)) / 2
        padding_y = (self.grid_layout.height - (cell_size * 12 + total_spacing_y)) / 2

        self.grid_layout.padding = [padding_x, padding_y, padding_x, padding_y]
