from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, Rectangle, RoundedRectangle
from Manipulations.Grid_map import GridMap
from Manipulations.Select_goalpositions import SelectGoalPositions
from Manipulations.Select_userobstacles import SelectUserObstacles

class MainScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


        # 상단 바
        self.header = BoxLayout(size_hint_y=None, height=40, pos_hint={'top': 1})
        with self.header.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[0])
        self.header.bind(pos=self.update_header_bg, size=self.update_header_bg)

        label = Label(text='[b]Low Level Cotrols[/b]', markup=True, color=(1, 1, 1, 1))
        self.header.add_widget(label)

        # 전체 레이아웃
        layout = FloatLayout()

        # 1. 배경색
        with self.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 2. 오른쪽 박스
        right_box = AnchorLayout(anchor_x='center', anchor_y='top',
                                 size_hint=(0.2, 0.95), pos_hint={'right': 1, 'y': 0})
        with right_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            right_bg = RoundedRectangle(pos=right_box.pos, size=right_box.size, radius=[5, 0, 0, 0])
        right_box.bind(pos=lambda *a: setattr(right_bg, 'pos', right_box.pos),
                       size=lambda *a: setattr(right_bg, 'size', right_box.size))

        # # 오른쪽 박스 내부 장애물 위치 레이아웃
        
        # User_obs_layout = BoxLayout(orientation='vertical', size_hint=(1, 0.3), spacing = 10, padding=10)
        # with User_obs_layout.canvas.before:
        #     Color(37 / 255, 40 / 255, 59 / 255, 1)
        #     right_bg = RoundedRectangle(pos=right_box.pos, size=right_box.size, radius=[5])
        # right_box.bind(pos=lambda *a: setattr(right_bg, 'pos', right_box.pos),
        #                size=lambda *a: setattr(right_bg, 'size', right_box.size))

        
        # right_box.add_widget(User_obs_layout)

        # 3. 하단 박스
        bottom_box = FloatLayout(size_hint = (0.8, 250 / 838), pos_hint={'x': 0, 'y': 0})
        with bottom_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            bottom_bg = RoundedRectangle(pos=bottom_box.pos, size=bottom_box.size, radius=[0])
        bottom_box.bind(pos=lambda *a: setattr(bottom_bg, 'pos', bottom_box.pos),
                        size=lambda *a: setattr(bottom_bg, 'size', bottom_box.size))

        goal_positions = SelectGoalPositions(
            size_hint=(0.65, 1),  # 하단 박스 크기 기준 너비 65%, 높이 100%
            pos_hint={'right': 1, 'y': 0.05 },
            height = bottom_box.height - 20,
              # 오른쪽 정렬
        )

        user_obstacles = SelectUserObstacles(
            size_hint=(0.35, 1),  # 하단 박스 크기 기준 너비 35%, 높이 100%
            pos_hint={'left': 1, 'y': 0.05},
            height = bottom_box.height - 20,
              # 왼쪽 정렬
        )

        bottom_box.add_widget(user_obstacles)
        bottom_box.add_widget(goal_positions)
        
        
        # 4. 그리드 맵 추가
        grid_map = GridMap()
        

        layout.add_widget(self.header)
        layout.add_widget(right_box)
        layout.add_widget(bottom_box)
        layout.add_widget(grid_map)
        self.add_widget(layout)
        

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
    
    def update_header_bg(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size
