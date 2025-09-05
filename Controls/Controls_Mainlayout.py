import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, Rectangle
from kivy.uix.boxlayout import BoxLayout
from Utilities.UI_utilities import KLine


class ControlsMain(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 전체 레이아웃
        self.layout = FloatLayout()
        self.add_widget(self.layout)

        # 전체 배경 
        with self.canvas.before:
            Color(46/255, 51/255, 73/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 상단 바 
        self.top_bar_height = 40
        self.top_bar = FloatLayout(size_hint=(1, None), height=self.top_bar_height, pos_hint={'top': 1})

        # 상단 바 배경색 
        with self.top_bar.canvas.before:
            Color(37/255, 40/255, 59/255, 1)
            self.top_bar_bg = Rectangle(pos=self.top_bar.pos, size=self.top_bar.size)
        with self.top_bar.canvas.after:
            Color(0, 0, 0, 1)
            self.top_bar_border = KLine(self.top_bar)

        # 위치 업데이트
        def update_top_bar_rect(*args):
            self.top_bar_bg.pos = self.top_bar.pos
            self.top_bar_bg.size = self.top_bar.size
            self.top_bar_border.rectangle = (self.top_bar.x, self.top_bar.y, self.top_bar.width, self.top_bar.height)
        self.top_bar.bind(pos=update_top_bar_rect, size=update_top_bar_rect)

        # 제목 
        title = Label(
            text="Controller",
            font_size=16,
            color=(1, 1, 1, 1),
            size_hint=(None, None),
            size=(100, 30),
            pos_hint={'center_x': 0.5, 'center_y': 0.5}
        )
        self.top_bar.add_widget(title)

        # X 버튼
        self.controller_close_btn = Button(
            text="X",
            size_hint=(None, None),
            size=(40, 40),
            pos_hint={'right': 1, 'center_y': 0.5},
            background_color=(0, 0, 0, 0),
            color=(1, 1, 1, 1)
        )
        self.top_bar.add_widget(close_btn)

        # 최종 배치
        self.layout.add_widget(self.top_bar)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
