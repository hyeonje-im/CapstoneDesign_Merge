import sys
import os
import numpy as np
import cv2

from kivy.uix.image import Image
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.graphics import Color, Rectangle
from kivy.graphics.texture import Texture
from kivy.clock import Clock

from OpenCV.code.ui_bridge import FrameBus
from Utilities.UI_utilities import KLine, make_darkcell, make_brightcell


# =====================================
# 주문 패널 (FrameBus에서 실시간 이미지 받기)
# =====================================
class RestaurantPanel(Image):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_interval(self.update_texture, 1 / 30)  # 30FPS

    def update_texture(self, dt):
        frame = FrameBus.get_orders()
        if frame is None:
            return  # 주문 패널이 없으면 아무것도 하지 않음

        # OpenCV → Kivy Texture 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = frame.shape
        tex = Texture.create(size=(w, h), colorfmt='rgb')
        tex.blit_buffer(frame.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
        tex.flip_vertical()
        self.texture = tex


# =====================================
# LeftWidget (왼쪽 전체 레이아웃)
# =====================================
class LeftWidget(BoxLayout):
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

        # 로봇별 상태 표시
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

        # ===== 하단 영역 (터미널 또는 주문 패널) =====
        self.terminal_box = make_darkcell("Terminal Output")  # 기본 터미널 창
        self.terminal_box.size_hint_y = 1  # 남은 공간 전부 차지

        self.orders_panel = RestaurantPanel(size_hint_y=1)

        # 초기엔 터미널 화면 표시
        self.add_widget(self.anchor)
        self.add_widget(self.terminal_box)

        # 주기적으로 주문 패널 표시 여부 갱신
        Clock.schedule_interval(self.update_orders_panel, 1 / 30)

        # 현재 표시 중인 위젯 추적용
        self.current_bottom_widget = self.terminal_box

    # ===== 주문 패널 표시 제어 =====
    def update_orders_panel(self, dt):
        frame = FrameBus.get_orders()

        # 주문 패널 이미지가 있을 때
        if frame is not None and self.current_bottom_widget is not self.orders_panel:
            # 터미널 제거 → 주문 패널 추가
            if self.current_bottom_widget in self.children:
                self.remove_widget(self.current_bottom_widget)
            self.add_widget(self.orders_panel)
            self.current_bottom_widget = self.orders_panel
            # print("[LeftWidget] 주문 패널 표시")

        # 주문 패널 이미지가 없을 때
        elif frame is None and self.current_bottom_widget is not self.terminal_box:
            # 주문 패널 제거 → 터미널 추가
            if self.current_bottom_widget in self.children:
                self.remove_widget(self.current_bottom_widget)
            self.add_widget(self.terminal_box)
            self.current_bottom_widget = self.terminal_box
            # print("[LeftWidget] 터미널 창 복귀")

    # ===== 배경/테두리 업데이트 =====
    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
