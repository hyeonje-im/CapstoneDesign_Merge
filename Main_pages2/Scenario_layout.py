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

        # ===============================
        # 1️⃣ 전체 배경
        # ===============================
        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ===============================
        # 2️⃣ 수직 레이아웃
        # ===============================
        root_layout = BoxLayout(orientation='vertical', padding=5, spacing=5)

        # 2-1) 상단 TopBar (메인과 동일)
        top_bar = TopBar()
        root_layout.add_widget(top_bar)

        # ===============================
        # 3️⃣ 하단 레이아웃 (좌 → 중앙 → 우)
        # ===============================
        bottom_row = BoxLayout(orientation='horizontal', spacing=5)

        # --- 왼쪽은 시나리오 모드에서는 사용 X ---
        # 필요하다면 나중에 Scenario_leftwidget 추가 가능

        # 3-1) 중앙 : ScenarioCenterWidget (MAPF + Grid)
        center_widget = ScenarioCenterWidget()
        bottom_row.add_widget(center_widget)

        # 3-2) 오른쪽 : 원본영상 + Warp 영상 (Main2_rightwidget 재사용)
        right_widget = RightWidget()
        bottom_row.add_widget(right_widget)

        root_layout.add_widget(bottom_row)
        self.add_widget(root_layout)

        # ===============================
        # 페이지 진입 시 시나리오 모드 설정
        # ===============================
        FrameBus.set_mode("scenario")
        post("scenario_mode_enter")

    # ===============================
    # 배경 업데이트
    # ===============================
    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
