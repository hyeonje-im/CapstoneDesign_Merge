import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle, Line
from kivy.uix.widget import Widget
from Utilities.UI_utilities import  KLine, make_darkcell, make_brightcell
from kivy.uix.anchorlayout import AnchorLayout

class LeftWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.2, **kwargs)

        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ▶ 상단 절반(총 로봇 대수)
        upper_half = AnchorLayout(size_hint_y=0.5, anchor_y='top')
        content = BoxLayout(orientation='vertical', size_hint=(1, None))
        content.bind(minimum_height=content.setter('height'))

        content.add_widget(make_brightcell("총 로봇 대수"))
        content.add_widget(make_brightcell("4"))

        upper_half.add_widget(content)
        

        # ▶ 하단 절반(장애물 설치 정보)
        lower_half = AnchorLayout(size_hint_y=0.5, anchor_y='top')
        content = BoxLayout(orientation='vertical', size_hint=(1, None))
        content.bind(minimum_height=content.setter('height'))
        lower_half.add_widget(make_brightcell("장애물 설치 정보"))

        info_grid = GridLayout(cols=2)
        info_grid.add_widget(make_darkcell("사용자 설정 장애물"))
        info_grid.add_widget(make_brightcell("9 PX"))
        info_grid.add_widget(make_darkcell("자동 인식 장애물"))
        info_grid.add_widget(make_brightcell("9 PX"))
        
        lower_half.add_widget(info_grid)

        # ▶ 중앙 가로선
        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.middle_line = Line(points=[], width=1)
        self.bind(pos=self.update_middle_line, size=self.update_middle_line)

        # ▶ 최종 배치
        self.add_widget(upper_half)
        
        self.add_widget(lower_half)
        


    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def update_middle_line(self, *args):
        x = self.x
        y = self.y + self.height / 2
        self.middle_line.points = [x, y, x + self.width, y]