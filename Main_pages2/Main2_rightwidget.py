import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle
from kivy.uix.widget import Widget
from Utilities.UI_utilities import  KLine, make_darkcell, make_brightcell
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label

class RightWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.6, **kwargs)

        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E / 255, 0x33 / 255, 0x49 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ⬆️ 상단 3행 2열 테이블 만들기
        anchor_layout = AnchorLayout(anchor_y='top')  # 고정 높이 예시
        table_layout = BoxLayout(orientation='vertical', size_hint=(1, None))
        table_layout.bind(minimum_height=table_layout.setter('height'))

        # 예시 데이터 (행별 텍스트)
        row_data = [
            ("영상 상태", ""),
            ("프레임 수", ""),
            ("해상도", "")
        ]

        for left_text, right_text in row_data:
            row = BoxLayout(orientation='horizontal', size_hint_y=None, height=30)
            row.add_widget(make_darkcell(left_text, size_hint_x=0.25))
            row.add_widget(make_brightcell(right_text, size_hint_x=0.75))
            table_layout.add_widget(row)

        anchor_layout.add_widget(table_layout)
        self.add_widget(anchor_layout)

    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
