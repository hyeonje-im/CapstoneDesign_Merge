import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.screenmanager import Screen
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle

from Main_pages2.Main2_topbar import TopBar
from Main_pages2.Scenario_center import ScenarioCenterWidget
from Main_pages2.Main2_rightwidget import RightWidget
from OpenCV.code.ui_bridge import post, FrameBus


class ScenarioLayout(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        
        #전체 배경색
        with self.canvas.before:
            Color(0xFC/255, 0xFC/255, 0xFC/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        
        # 수직 레이아웃
        root_layout = BoxLayout(orientation='vertical', padding=5, spacing=5)

        #상단 TopBar
        top_bar = TopBar()
        root_layout.add_widget(top_bar)

       
        # 하단 레이아웃 
        bottom_row = BoxLayout(orientation='horizontal', spacing=5)


        center_widget = ScenarioCenterWidget()
        bottom_row.add_widget(center_widget)

        right_widget = RightWidget()
        bottom_row.add_widget(right_widget)

        root_layout.add_widget(bottom_row)
        self.add_widget(root_layout)

        FrameBus.set_mode("scenario")
        post("scenario_mode_enter")


    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size