from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.graphics import Color, RoundedRectangle
from kivy.uix.widget import Widget

class SystemstatusWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', spacing=0, padding=0, **kwargs)

        with self.canvas.before:
            Color(0.823, 0.855, 1.0, 1)  # D2DAFF 배경
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[10])
        self.bind(pos=self.update_bg, size=self.update_bg)

       
        # ── 상단 바 ──
        self.header = BoxLayout(size_hint_y=None, height=30)
        with self.header.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[7, 7, 0, 0])
        self.header.bind(pos=self.update_header, size=self.update_header)
        self.header.add_widget(Label(text="System Status", bold=True, color=(1, 1, 1, 1)))

        # ── 아래 빈 흰 공간 ──
        self.grid_area = Widget()
        with self.grid_area.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)
            self.grid_bg = RoundedRectangle(pos=self.grid_area.pos, size=self.grid_area.size, radius=[0, 0, 7, 7])
        self.grid_area.bind(pos=self.update_grid_bg, size=self.update_grid_bg)

        self.add_widget(self.header)
        self.add_widget(self.grid_area)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size

    def update_grid_bg(self, *args):
        self.grid_bg.pos = self.grid_area.pos
        self.grid_bg.size = self.grid_area.size