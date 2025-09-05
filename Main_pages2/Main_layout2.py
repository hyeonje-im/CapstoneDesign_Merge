import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.screenmanager import Screen
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from Main_pages2.Main2_topbar import TopBar
from Main_pages2.Main2_leftwidget import LeftWidget
from Main_pages2.Main2_centerwidget import CenterWidget
from Main_pages2.Main2_rightwidget import RightWidget
from Utilities.UI_utilities import KLabel, KLine

class MainLayout2(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 1. 전체 배경색 (25283B)
        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 2. 전체 세로 레이아웃
        root_layout = BoxLayout(orientation='vertical', padding=5, spacing=5)
        
        top_bar = TopBar()
        root_layout.add_widget(top_bar)

        # 하단 3분할
        bottom_row = BoxLayout(orientation='horizontal', spacing=5)

        # 왼쪽 위젯
        left_widget = LeftWidget()
        bottom_row.add_widget(left_widget)

        # 중앙 위젯
        center_widget = CenterWidget()
        bottom_row.add_widget(center_widget)

        # 오른쪽 위젯
        right_widget = RightWidget()
        bottom_row.add_widget(right_widget)


        root_layout.add_widget(bottom_row)

        self.add_widget(root_layout)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_rect(self, widget):
        def updater(*args):
            widget.bg.pos = widget.pos
            widget.bg.size = widget.size
        return updater

    def update_line_rect(self, widget):
        def updater(*args):
            widget.border.rectangle = (widget.x, widget.y, widget.width, widget.height)
            widget.bg.pos = widget.pos
            widget.bg.size = widget.size
        return updater
