import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import Color, Rectangle, RoundedRectangle
from kivy.uix.boxlayout import BoxLayout

from Video_capture import VideoCaptureWidget 
from Warped_perspective import WarpedperspectiveWidget  
from Grid_visualization import GridVisualizationWidget 
from Components.Tags_info import TagsInfoWidget  # ← 임포트 추가
from Components.System_status import SystemstatusWidget
from Components.Advanced_controls import AdvancedcontrolWidget


class ColoredScreen(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        with self.canvas.before:
            Color(238/255, 241/255, 255/255, 1)  # EEF1FF
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── VideoCaptureWidget 배치 ──
        vc = VideoCaptureWidget()
        vc.size_hint = (500 / 1210, 500 / 838)         # 약 (0.330, 0.358)
        vc.pos_hint = {"x": 83 / 1210, "y": 283 / 838}  # 약 (0.233, 0.099)
        self.add_widget(vc)

        wp = WarpedperspectiveWidget()
        wp.size_hint = (500 / 1210, 500 / 838)         # 약 (0.330, 0.358)
        wp.pos_hint = {"x": 625 / 1210, "y": 283 / 838}# 약 (0.233, 0.099)
        self.add_widget(wp)

        # ── 하단 흰색 박스 ──
        self.bottom_panel = BoxLayout(orientation='horizontal', spacing=10, padding=10)
        self.bottom_panel.size_hint = ( 1180 / 1210, 250 / 838)
        self.bottom_panel.pos_hint = {"x": 15 / 1210 , "y": 15 / 838}  # 약 (0.012, 0.018)
        
        with self.bottom_panel.canvas.before:
            Color(210 / 255, 218 / 255, 255 / 255 , 1)  # 흰색 FFFFFF
            self.bottom_rect = RoundedRectangle(pos=self.bottom_panel.pos, size=self.bottom_panel.size, radius=[7])
        
        self.bottom_panel.bind(pos=self.update_bottom_rect, size=self.update_bottom_rect)
        

        # Grid Visualization 위젯 추가
        grid_visual = GridVisualizationWidget()
        grid_visual.size_hint = (0.25, 1)  
        
        
        # Tags Info 위젯 추가 (grid_visual 오른쪽)
        tags_info = TagsInfoWidget()
        tags_info.size_hint = (0.15, 1)
        
        # System Status 위젯 추가 (tags info 오른쪽)
        system_status = SystemstatusWidget()
        system_status.size_hint = (0.35, 1)
        
        # Advanced Control 위젯 추가 (system status 오른쪽)
        advanced_control = AdvancedcontrolWidget()
        advanced_control.size_hint = (0.3, 1)


        self.bottom_panel.add_widget(grid_visual)
        self.bottom_panel.add_widget(tags_info)
        self.bottom_panel.add_widget(system_status) 
        self.bottom_panel.add_widget(advanced_control)  # Advanced Control 위젯 추가


        self.add_widget(self.bottom_panel)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_bottom_rect(self, *args):
        self.bottom_rect.pos = self.children[0].pos
        self.bottom_rect.size = self.children[0].size

class MyApp(App):
    def build(self):
        return ColoredScreen()

if __name__ == "__main__":
    MyApp().run()
