import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle, RoundedRectangle
from kivy.uix.button import Button
from Controls.first_introduction import introductions
from Pages.Video_capture import VideoCaptureWidget
from Pages.Grid_visualization import GridVisualizationWidget

class AdvancedMainLayout(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 전체 배경
        with self.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 상단 바
        self.header = BoxLayout(size_hint_y=None, height=30, pos_hint={'top': 1})
        with self.header.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[0])
        self.header.bind(pos=self.update_header_bg, size=self.update_header_bg)

        label = Label(text='[b]Advanced Controls[/b]', markup=True, color=(1, 1, 1, 1))
        self.header.add_widget(label)
        layout = FloatLayout()
       
       
       
        # 중앙 메인 컨텐츠 영역 (왼쪽 영상 + 오른쪽 격자)
        main_area = BoxLayout(orientation='horizontal',
                      size_hint=(0.85, 0.67),
                      pos_hint={'x': 0, 'y': 0.3},
                      spacing=10)

        # 왼쪽: video_capture.py의 위젯
        
        video_widget = VideoCaptureWidget(size_hint_x=0.62)

        # 오른쪽: grid_visualization.py의 위젯
        
        grid_widget = GridVisualizationWidget(size_hint_x=0.38)

        main_area.add_widget(video_widget)
        main_area.add_widget(grid_widget)
       

        
        # 오른쪽 박스 (AnchorLayout)
        right_box = AnchorLayout(anchor_x='center', anchor_y='top',
                                 size_hint=(0.15, 0.95), pos_hint={'right': 1, 'y': 0})
        with right_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            right_box_bg = RoundedRectangle(pos=right_box.pos, size=right_box.size, radius=[5, 0, 0, 0])
        right_box.bind(pos=lambda *a: setattr(right_box_bg, 'pos', right_box.pos),
                       size=lambda *a: setattr(right_box_bg, 'size', right_box.size))

        # ScrollView와 관련된 코드 제거, right_inner를 바로 right_box에 추가
        right_inner = BoxLayout(orientation='vertical', padding=10, spacing=10, size_hint=(1,0.5))

        for item in introductions():
            right_inner.add_widget(item)
        main_button = Button(
            text='[b]Get Started[/b]', markup=True,
            size_hint_y=None, height=50,
            background_normal='', background_color=(0, 0, 0, 0),  # 투명 배경으로 설정
            color=(1, 1, 1, 1)
        )

        main_button.bind(on_press = self.Low_level_control)
        
        # 라운딩 효과 추가
        with main_button.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            main_button.bg_rect = RoundedRectangle(pos=main_button.pos, size=main_button.size, radius=[5])

        # 버튼 위치·크기 변경 시 배경도 따라가게 바인딩
        main_button.bind(pos=lambda *a: setattr(main_button.bg_rect, 'pos', main_button.pos),
                 size=lambda *a: setattr(main_button.bg_rect, 'size', main_button.size))

        right_inner.add_widget(main_button)

        right_box.add_widget(right_inner)

        # 작은 버튼 (EXIT) → 최상위 FloatLayout로 이동
        small_button = Button(
            text='[b]EXIT[/b]', markup = True,
            size_hint=(0.07, 0.04),
            pos_hint={'right': 0.98, 'y': 0.02},
            background_normal='', background_color=(115 / 255, 103 / 255, 239 / 255, 1),
            color=(1, 1, 1, 1)
        )
        with small_button.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            small_button_bg = RoundedRectangle(pos=small_button.pos, size=small_button.size, radius=[5])
        small_button.bind(pos=lambda *a: setattr(small_button_bg, 'pos', small_button.pos),
                          size=lambda *a: setattr(small_button_bg, 'size', small_button.size))
        small_button.bind(on_press=self.Main_layout)
        
        
        # 하단 박스
        bottom_box = FloatLayout(size_hint = (1190 / 1210, 250 / 838), pos_hint={'x': 0, 'y': 0})
        with bottom_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            bottom_box_bg = RoundedRectangle(pos=bottom_box.pos, size=bottom_box.size, radius=[0])
        bottom_box.bind(pos=lambda *a: setattr(bottom_box_bg, 'pos', bottom_box.pos),
                        size=lambda *a: setattr(bottom_box_bg, 'size', bottom_box.size))

        # 레이아웃에 추가
        layout.add_widget(main_area)
        layout.add_widget(bottom_box)
        layout.add_widget(right_box)
        layout.add_widget(small_button)
        layout.add_widget(self.header)
        
        self.add_widget(layout)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header_bg(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size

    def Low_level_control(self, instance):
        self.manager.current = 'Low_level_control'

    def Main_layout(self, instance):
        self.manager.current = 'Main_layout'