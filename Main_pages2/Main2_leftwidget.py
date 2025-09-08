import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle, Line

from Utilities.UI_utilities import  KLine, make_darkcell, make_brightcell


class LeftWidget(BoxLayout):  # BoxLayout(orientation='vertical')으로도 가능
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.15, **kwargs)

        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # 내부 레이아웃
        self.inner_layout = BoxLayout(orientation='vertical', size_hint=(1, 1), spacing=5, padding=(0, 0, 0, 0))

        # 타이틀
        self.inner_layout.add_widget(make_darkcell("로봇 구동 정보"))

        for i in range(1, 5):
            robot_box = BoxLayout(orientation='vertical', size_hint_y=None, height=160)
            robot_box.add_widget(make_darkcell(f"ID:{i}"))
            for label_text in ["구동 속도", "구동 방향", "목표 위치"]:
                row = GridLayout(cols=2, size_hint_y=None, height=30)
                row.add_widget(make_darkcell(label_text))
                row.add_widget(make_brightcell(""))
                robot_box.add_widget(row)

            self.inner_layout.add_widget(robot_box)

        # 빈 공간 추가
        self.inner_layout.add_widget(Widget(size_hint_y=1))

        self.add_widget(self.inner_layout)

    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
