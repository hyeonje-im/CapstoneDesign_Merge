import sys
import os

# 상위 폴더를 Python 모듈 경로에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.core.window import Window

from Pages.Video_capture import VideoCaptureWidget
from Pages.Warped_perspective import WarpedPerspectiveWidget
from Pages.Grid_visualization import GridVisualizationWidget
from Components.Robot_status import RobotStatusWidget  
from Components.Tags_info import TagsInfoWidget
from Components.Advanced_controls import AdvancedcontrolWidget
from Components.Tags_cords import TagsCordsWidget





# 창 크기 및 배경 색 설정
Window.size = (1210, 820)
Window.clearcolor = (0xEE / 255, 0xF1 / 255, 0xFF / 255, 1)  # EEF1FF 배경


class MainLayout(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # VideoCaptureWidget
        video_widget = VideoCaptureWidget()
        video_widget.size_hint = (None, None)
        video_widget.pos = (250, 400)
        self.add_widget(video_widget)

        # WarpedPerspectiveWidget
        warped_widget = WarpedPerspectiveWidget()
        warped_widget.size_hint = (None, None)
        warped_widget.pos = (870, 400)
        self.add_widget(warped_widget)

        # GridVisualizationWidget
        grid_widget = GridVisualizationWidget()
        grid_widget.size_hint = (None, None)
        grid_widget.pos = (10, 10)
        self.add_widget(grid_widget)

        # TagsInfoWidget
        tags_widget = TagsInfoWidget()
        tags_widget.size_hint = (None, None)
        tags_widget.pos = (460, 10)
        self.add_widget(tags_widget)

        # RobotStatusWidget 
        robot_widget = RobotStatusWidget()
        robot_widget.size_hint = (None, None)
        robot_widget.pos = (660, 10)
        self.add_widget(robot_widget)
        
        # AdvancedControlsWidget
        advanced_widget = AdvancedcontrolWidget()
        advanced_widget.size_hint = (None, None)
        advanced_widget.pos = (1110, 10)
        self.add_widget(advanced_widget)

        # TagsCordsWidget
        tags_cords_widget = TagsCordsWidget()
        tags_cords_widget.size_hint = (None, None)
        tags_cords_widget.pos = (10, 400)
        self.add_widget(tags_cords_widget)
        


class MainApp(App):
    def build(self):
        return MainLayout()


if __name__ == '__main__':
    MainApp().run()
