import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import Color, Rectangle, RoundedRectangle
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.screenmanager import Screen
from Pages.Video_capture import VideoCaptureWidget 
from Pages.Warped_perspective import WarpedperspectiveWidget  
from Pages.Grid_visualization import GridVisualizationWidget 
from Components.Tags_info import TagsInfoWidget
from Components.System_status import SystemstatusWidget
from Advanced_controls.Advanced_mainlayout import AdvancedcontrolWidget


class ColoredScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        with self.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)  # EEF1FF
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── VideoCaptureWidget 배치 ──
        vc = VideoCaptureWidget()
        vc.size_hint = (755 / 1210, 540 / 838)
        vc.pos_hint = {"x": 10 / 1210, "y": 283 / 838}
        self.add_widget(vc)

        wp = WarpedperspectiveWidget()
        wp.size_hint = (424 / 1210, 540 / 838)
        wp.pos_hint = {"x": 775 / 1210, "y": 283 / 838}
        self.add_widget(wp)

        # ── 하단 패널 ──
        self.bottom_panel = BoxLayout(orientation='horizontal', spacing=10, padding=10)
        self.bottom_panel.size_hint = (1190 / 1210, 250 / 838)
        self.bottom_panel.pos_hint = {"x": 10 / 1210, "y": 15 / 838}

        with self.bottom_panel.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            self.bottom_rect = RoundedRectangle(pos=self.bottom_panel.pos, size=self.bottom_panel.size, radius=[7])

        self.bottom_panel.bind(pos=self.update_bottom_rect, size=self.update_bottom_rect)

        # ── 각 위젯 추가 ──
        grid_visual = GridVisualizationWidget()
        grid_visual.size_hint = (0.25, 1)

        tags_info = TagsInfoWidget()
        tags_info.size_hint = (0.15, 1)

        system_status = SystemstatusWidget()
        system_status.size_hint = (0.35, 1)

        # ── AdvancedcontrolWidget 생성 시 콜백 전달 ──
        def go_to_advanced_mainlayout(instance):
            self.manager.current = 'Advanced_mainlayout'  # ✅ ScreenManager current 변경

        advanced_control = AdvancedcontrolWidget(on_go=go_to_advanced_mainlayout)
        advanced_control.size_hint = (0.3, 1)

        # ── bottom_panel에 위젯 추가 ──
        self.bottom_panel.add_widget(grid_visual)
        self.bottom_panel.add_widget(tags_info)
        self.bottom_panel.add_widget(system_status)
        self.bottom_panel.add_widget(advanced_control)

        self.add_widget(self.bottom_panel)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_bottom_rect(self, *args):
        self.bottom_rect.pos = self.children[0].pos
        self.bottom_rect.size = self.children[0].size
