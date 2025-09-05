from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.uix.image import Image
from kivy.graphics import Color, RoundedRectangle
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.floatlayout import FloatLayout 

import cv2


class VideoCaptureWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', spacing=0, padding=0, **kwargs)

        with self.canvas.before:
            Color(0.933, 0.945, 1.0, 1)  # EEF1FF 배경
            self.bg = RoundedRectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── 상단 바 ──
        self.header = BoxLayout(size_hint_y=None, height=30)
        self.header.canvas.before.clear()
        with self.header.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)  # B1B2FF
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[7, 7, 0, 0])
        self.header.bind(pos=self.update_header, size=self.update_header)
        self.header.add_widget(Label(text="Video capture", bold=True, color=(1, 1, 1, 1)))

        # ── 영상 영역 ──
        self.video_area = FloatLayout()
        with self.video_area.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)  # Dark background
            self.video_bg = RoundedRectangle(pos=self.video_area.pos, size=self.video_area.size, radius=[0,0,7,7])
        self.video_area.bind(pos=self.update_video_bg, size=self.update_video_bg)

        self.image_widget = Image(
            size_hint=(None, None),
            allow_stretch=False,
            keep_ratio=True,
            pos_hint={'center_x': 0.5, 'center_y': 0.5}
        )
        self.video_area.add_widget(self.image_widget)

        self.add_widget(self.header)
        self.add_widget(self.video_area)

   
    def update_image_position(self, frame_width, frame_height):
        parent_w, parent_h = self.video_area.size
        if parent_w == 0 or parent_h == 0:
            return

        aspect_ratio = frame_width / frame_height
        if parent_w / aspect_ratio <= parent_h:
            new_w = parent_w
            new_h = parent_w / aspect_ratio
        else:
            new_h = parent_h
            new_w = parent_h * aspect_ratio

        self.image_widget.size = (new_w, new_h)
        self.image_widget.pos = (
            (parent_w - new_w) / 2,
            (parent_h - new_h) / 2
        )

    def np_to_texture(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        buf = frame_rgb.flatten()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='rgb')
        texture.blit_buffer(buf, colorfmt='rgb', bufferfmt='ubyte')
        texture.flip_vertical()
        return texture

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size

    def update_video_bg(self, *args):
        self.video_bg.pos = self.video_area.pos
        self.video_bg.size = self.video_area.size
