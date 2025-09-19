import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# ▼ 추가 import
import cv2
from kivy.clock import Clock
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from OpenCV.code.ui_bridge import FrameBus

from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle
from kivy.uix.widget import Widget
from Utilities.UI_utilities import  KLine, make_darkcell, make_brightcell
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label


class VideoFeed(Image):
    """FrameBus에서 BGR 프레임을 읽어 Texture로 그리는 Kivy 위젯."""
    def __init__(self, fps=30, **kwargs):
        super().__init__(allow_stretch=True, keep_ratio=True, **kwargs)
        self._interval = 1.0 / max(1, fps)
        Clock.schedule_interval(self._tick, self._interval)

    def _tick(self, dt):
        frame_bgr = FrameBus.get_video()
        if frame_bgr is None:
            return
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        if (self.texture is None) or (self.texture.width != w) or (self.texture.height != h):
            self.texture = Texture.create(size=(w, h), colorfmt='rgb')
            self.texture.flip_vertical()
        self.texture.blit_buffer(rgb.tobytes(), colorfmt='rgb', bufferfmt='ubyte')


class WarpedFeed(Image):
    """FrameBus에서 왜곡 보정된 BGR 프레임을 읽어 Texture로 그리는 Kivy 위젯."""
    def __init__(self, fps=30, **kwargs):
        super().__init__(allow_stretch=True, keep_ratio=True, **kwargs)
        self._interval = 1.0 / max(1, fps)
        Clock.schedule_interval(self._tick, self._interval)

    def _tick(self, dt):
        frame_bgr = FrameBus.get_warped()
        if frame_bgr is None:
            return
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        if (self.texture is None) or (self.texture.width != w) or (self.texture.height != h):
            self.texture = Texture.create(size=(w, h), colorfmt='rgb')
            self.texture.flip_vertical()
        self.texture.blit_buffer(rgb.tobytes(), colorfmt='rgb', bufferfmt='ubyte')


class RightWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.6, **kwargs)

        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E / 255, 0x33 / 255, 0x49 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ⬆️ 상단 3행 2열 테이블 (고정 높이로 잡아 아래 공간을 영상이 차지)
        anchor_layout = AnchorLayout(anchor_y='top', size_hint_y=None, height=30*3)
        table_layout = BoxLayout(orientation='vertical', size_hint=(1, 1))

        row_data = [("영상 상태", ""), ("프레임 수", ""), ("해상도", "")]
        for left_text, right_text in row_data:
            row = BoxLayout(orientation='horizontal', size_hint_y=None, height=30)
            row.add_widget(make_darkcell(left_text, size_hint_x=0.25))
            row.add_widget(make_brightcell(right_text, size_hint_x=0.75))
            table_layout.add_widget(row)

        anchor_layout.add_widget(table_layout)
        self.add_widget(anchor_layout)

        # ⬇️ 남은 공간 전부 비디오가 차지
        self.video_panel = VideoFeed(fps=30, size_hint=(1, 1))
        self.add_widget(self.video_panel)

    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
