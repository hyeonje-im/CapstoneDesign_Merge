import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle, Line
from kivy.uix.widget import Widget
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import Clock

from Utilities.UI_utilities import KLine, make_darkcell, make_brightcell
from kivy.uix.anchorlayout import AnchorLayout

# ★ OpenCV ↔ Kivy 브리지
from OpenCV.code.ui_bridge import FrameBus
import numpy as np

class GridTextureView(Image):
    """FrameBus.get_grid() (BGR ndarray)를 읽어 전체 영역에 그리드 텍스처 표시"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.allow_stretch = True
        self.keep_ratio = True
        Clock.schedule_interval(self._update, 1/20)  # 50ms마다 갱신 (20fps)

    def _update(self, dt):
        frame = FrameBus.get_grid()  # np.ndarray(BGR) or None
        if frame is None:
            return
        # BGR -> RGB
        rgb = frame[:, :, ::-1].copy()
        h, w = rgb.shape[:2]

        # 텍스처 생성/갱신
        if not self.texture or self.texture.width != w or self.texture.height != h:
            self.texture = Texture.create(size=(w, h))
            self.texture.flip_vertical()  # OpenCV↔Kivy 좌표계 보정
        self.texture.blit_buffer(rgb.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
        self.canvas.ask_update()


class CenterWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.25, **kwargs)

        # 배경/테두리
        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ── 상단 50%: "총 로봇 대수" + 그리드뷰(남은 공간 전체)
        upper_half = BoxLayout(orientation='vertical', size_hint_y=0.5, spacing=4, padding=[0,4,0,4])

        # 상단 정보 셀들
        upper_half.add_widget(make_brightcell("총 로봇 대수"))
        # 총 로봇 수 텍스트 셀 (필요시 바인딩으로 갱신 가능)
        self.robot_count_cell = make_brightcell("4")
        upper_half.add_widget(self.robot_count_cell)

        # ★ 남은 공간 전체에 그리드뷰 배치
        self.grid_view = GridTextureView(size_hint_y=1)
        upper_half.add_widget(self.grid_view)

        # ── 하단 50%: 장애물 설치 정보
        lower_half = AnchorLayout(size_hint_y=0.5, anchor_y='top')
        content = BoxLayout(orientation='vertical', size_hint=(1, None))
        content.bind(minimum_height=content.setter('height'))
        lower_half.add_widget(make_brightcell("장애물 설치 정보"))

        info_grid = GridLayout(cols=2, size_hint_y=None)
        info_grid.bind(minimum_height=info_grid.setter('height'))
        info_grid.add_widget(make_darkcell("사용자 설정 장애물"))
        info_grid.add_widget(make_brightcell("9 PX"))
        info_grid.add_widget(make_darkcell("자동 인식 장애물"))
        info_grid.add_widget(make_brightcell("9 PX"))

        lower_half.add_widget(info_grid)

        # 중앙 가로선
        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.middle_line = Line(points=[], width=1)
        self.bind(pos=self.update_middle_line, size=self.update_middle_line)

        # 최종 배치
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
