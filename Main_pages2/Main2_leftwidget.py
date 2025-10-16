import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.graphics import Color, Rectangle, Line

from Utilities.UI_utilities import KLine, make_darkcell, make_brightcell


class LeftWidget(BoxLayout):  # BoxLayout(orientation='vertical')으로도 가능
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.15, **kwargs)

        # ===== 배경/테두리 =====
        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E / 255, 0x33 / 255, 0x49 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ===== 상단 영역 (로봇 구동 정보) =====
        self.anchor = AnchorLayout(anchor_y='top', size_hint_y=None)
        self.inner_layout = BoxLayout(orientation='vertical', size_hint=(1, None))
        self.inner_layout.bind(minimum_height=self.inner_layout.setter('height'))

        # 타이틀
        self.inner_layout.add_widget(make_darkcell("로봇 구동 정보"))

        for i in range(1, 5):
            robot_box = BoxLayout(orientation='vertical', size_hint_y=None, spacing=2)
            robot_box.bind(minimum_height=robot_box.setter('height'))

            robot_box.add_widget(make_darkcell(f"D{i}"))
            for label_text in ["목표 위치", "Delay"]:
                row = GridLayout(cols=2, size_hint_y=None, height=30)
                row.add_widget(make_darkcell(label_text))
                row.add_widget(make_brightcell(""))
                robot_box.add_widget(row)

            self.inner_layout.add_widget(robot_box)

        self.anchor.add_widget(self.inner_layout)
        self.inner_layout.bind(height=lambda instance, val: setattr(self.anchor, 'height', val))
        # ===== 하단 영역 (터미널 창) =====
        self.terminal_box = make_darkcell("Terminal Output")  # 기본 텍스트
        self.terminal_box.size_hint_y = 1  # 남은 여백 전부 차지

        # ===== 전체 배치 =====
        self.add_widget(self.anchor)
        self.add_widget(self.terminal_box)

    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
