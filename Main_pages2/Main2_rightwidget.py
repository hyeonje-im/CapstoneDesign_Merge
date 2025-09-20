import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# ▼ 추가 import
import cv2
from kivy.clock import Clock
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from OpenCV.code.ui_bridge import FrameBus
from kivy.uix.button import Button
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle
from kivy.uix.widget import Widget
from Utilities.UI_utilities import  KLine, KButton
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

        # 상단 영상 전환 버튼
        button_row = BoxLayout(orientation='horizontal', size_hint_y=None, height=30)
        btn1 = KButton(text = "video feed", size_hint = (None,1), width = 100)
        btn2 = KButton(text = "warped feed", size_hint = (None,1), width = 100)
        btn1.bind(on_release = self.show_video_feed)
        btn2.bind(on_release = self.show_warped_feed)
        
        button_row.add_widget(btn1)
        button_row.add_widget(btn2)
        self.add_widget(button_row)


        #영상 및 워프보드 영상 레이어 생성
        self.video_layer = RelativeLayout(size_hint = (1,1))

        #원본 영상
        self.video_feed = VideoFeed(fps=30, size_hint = (1,1))
    
        #워프보드 영상
        self.warped_feed = Widget(size_hint = (1,1))
        with self.warped_feed.canvas.before:
            Color(0.2, 0.2, 0.2, 1)  # 회색 배경
            self.placeholder_bg = Rectangle(pos=self.warped_feed.pos, size=self.warped_feed.size)
        self.warped_feed.bind(pos=self.update_placeholder, size=self.update_placeholder)

        # 두 위젯 겹쳐서 놓기
        self.video_layer.add_widget(self.video_feed)
        self.video_layer.add_widget(self.warped_feed)

        #초기에 원본 영상 보이도록 설정
        self.video_feed.opacity = 1
        self.warped_feed.opacity = 0

        self.add_widget(self.video_layer)

    def show_video_feed(self, instance):
        self.video_feed.opacity = 1
        self.warped_feed.opacity = 0

    def show_warped_feed(self, instance):
        self.video_feed.opacity = 0
        self.warped_feed.opacity = 1

    def update_placeholder(self, *args):
        self.placeholder_bg.pos = self.warped_feed.pos
        self.placeholder_bg.size = self.warped_feed.size


    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
