from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label
from kivy.uix.scrollview import ScrollView
from kivy.graphics import Color, Rectangle, RoundedRectangle
from kivy.uix.button import Button

class AdvancedMainLayout(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 전체 배경
        with self.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 상단 바
        self.header = BoxLayout(size_hint_y=None, height=40, pos_hint={'top': 1})
        with self.header.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[0])
        self.header.bind(pos=self.update_header_bg, size=self.update_header_bg)

        label = Label(text='[b]Advanced Controls[/b]', markup=True, color=(1, 1, 1, 1))
        self.header.add_widget(label)

        # 전체 레이아웃
        layout = FloatLayout()

        # 오른쪽 박스 (AnchorLayout)
        right_box = AnchorLayout(anchor_x='center', anchor_y='top',
                                 size_hint=(0.2, 0.95), pos_hint={'right': 1, 'y': 0})
        with right_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            right_box_bg = RoundedRectangle(pos=right_box.pos, size=right_box.size, radius=[5, 0, 0, 0])
        right_box.bind(pos=lambda *a: setattr(right_box_bg, 'pos', right_box.pos),
                       size=lambda *a: setattr(right_box_bg, 'size', right_box.size))

        # ScrollView 안에 BoxLayout
        scrollview = ScrollView(size_hint=(1, 1))
        right_inner = BoxLayout(orientation='vertical', padding=10, spacing=10, size_hint_y=None)
        right_inner.bind(minimum_height=right_inner.setter('height'))

        # 텍스트 항목 추가
        items = [
            ("Obstacle rearrangement", "Relocate the grid obstacles and manually define the accessible area."),
            ("Robot's goal positions setting", "Relocate the grid obstacles and manually define the accessible area."),
            ("Low-level robot control", "Try manually controlling the robot's start and stop times."),
            ("Obstacle rearrangement", "Relocate the grid obstacles and manually define the accessible area.")
        ]

        for title, desc in items:
            item_box = BoxLayout(orientation='vertical', size_hint_y=None, height=70)
            title_label = Label(text=f"[b]{title}[/b]", markup=True, color=(1, 1, 1, 1), halign='left', valign='middle')
            desc_label = Label(text=desc, color=(1, 1, 1, 0.7), halign='left', valign='top')

            title_label.bind(size=lambda inst, *a: setattr(inst, 'text_size', inst.size))
            desc_label.bind(size=lambda inst, *a: setattr(inst, 'text_size', inst.size))

            item_box.add_widget(title_label)
            item_box.add_widget(desc_label)
            right_inner.add_widget(item_box)
        
        
        main_button = Button(
            text='[b]Get Started[/b]', markup=True,
            size_hint_y=None, height=50,
            background_normal='', background_color=(0, 0, 0, 0),  # 투명 배경으로 설정
            color=(1, 1, 1, 1)
        )

        # 라운딩 효과 추가
        with main_button.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            main_button.bg_rect = RoundedRectangle(pos=main_button.pos, size=main_button.size, radius=[5])

        # 버튼 위치·크기 변경 시 배경도 따라가게 바인딩
        main_button.bind(pos=lambda *a: setattr(main_button.bg_rect, 'pos', main_button.pos),
                 size=lambda *a: setattr(main_button.bg_rect, 'size', main_button.size))

        right_inner.add_widget(main_button)

        scrollview.add_widget(right_inner)
        right_box.add_widget(scrollview)

        # 작은 버튼 (EXIT) → 최상위 FloatLayout로 이동
        small_button = Button(
            text='EXIT', size_hint=(0.08, 0.05),
            pos_hint={'right': 0.98, 'y': 0.02},
            background_normal='', background_color=(115 / 255, 103 / 255, 239 / 255, 1),
            color=(1, 1, 1, 1)
        )

        # 하단 박스
        bottom_box = FloatLayout(size_hint=(0.8, 0.25), pos_hint={'x': 0, 'y': 0})
        with bottom_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            bottom_box_bg = RoundedRectangle(pos=bottom_box.pos, size=bottom_box.size, radius=[0])
        bottom_box.bind(pos=lambda *a: setattr(bottom_box_bg, 'pos', bottom_box.pos),
                        size=lambda *a: setattr(bottom_box_bg, 'size', bottom_box.size))

        # 레이아웃에 추가
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
