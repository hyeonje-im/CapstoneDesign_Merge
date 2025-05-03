from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.graphics import Color, RoundedRectangle

class VideoCaptureWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', spacing=0, padding=0, **kwargs)

        with self.canvas.before:
            Color(0.933, 0.945, 1.0, 1)  # EEF1FF 배경
            self.bg = RoundedRectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── 상단 바 ──
        self.header = BoxLayout(size_hint_y=None, height=40)
        self.header.canvas.before.clear()
        with self.header.canvas.before:
            Color(0.694, 0.698, 1.0, 1)  # B1B2FF
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[7, 7, 0, 0])
        self.header.bind(pos=self.update_header, size=self.update_header)
        self.header.add_widget(Label(text="Video capture", bold=True, color=(0, 0, 0, 1)))

        # ── 영상 영역 ──
        self.video_area = Widget()
        with self.video_area.canvas.before:
            Color(1, 1, 1, 1)  # FFFFFF
            self.video_bg = RoundedRectangle(pos=self.video_area.pos, size=self.video_area.size, radius = [0,0,7,7])
        self.video_area.bind(pos=self.update_video_bg, size=self.update_video_bg)

        self.add_widget(self.header)
        self.add_widget(self.video_area)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size

    def update_video_bg(self, *args):
        self.video_bg.pos = self.video_area.pos
        self.video_bg.size = self.video_area.size
