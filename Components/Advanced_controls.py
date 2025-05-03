from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, RoundedRectangle

class AdvancedcontrolWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', spacing=0, padding=0, **kwargs)

        # 전체 배경
        with self.canvas.before:
            Color(0.823, 0.855, 1.0, 1)  # D2DAFF
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[10])
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── 상단 바 ──
        self.header = BoxLayout(size_hint_y=None, height=40)
        with self.header.canvas.before:
            Color(0.667, 0.769, 1.0, 1)  # AAC4FF
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[7, 7, 0, 0])
        self.header.bind(pos=self.update_header, size=self.update_header)
        self.header.add_widget(Label(text="Advanced Controls", bold=True, color=(0, 0, 0, 1)))

        # ── 아래 흰 공간 → FloatLayout로 교체 ──
        self.grid_area = FloatLayout()
        with self.grid_area.canvas.before:
            Color(1, 1, 1, 1)
            self.grid_bg = RoundedRectangle(pos=self.grid_area.pos, size=self.grid_area.size, radius=[0, 0, 7, 7])
        self.grid_area.bind(pos=self.update_grid_bg, size=self.update_grid_bg)

        # ── 중앙 파란 박스 ──
        blue_box = BoxLayout(orientation='horizontal', padding=(10,10), spacing=10, size_hint=(0.95, 0.35), pos_hint={'center_x':0.5, 'center_y':0.5})
        with blue_box.canvas.before:
            Color(0.67, 0.77, 1.0, 1)  # AAC4FF
            blue_box_bg = RoundedRectangle(pos=blue_box.pos, size=blue_box.size, radius=[10])
        blue_box.bind(pos=lambda *a: setattr(blue_box_bg, 'pos', blue_box.pos),
                      size=lambda *a: setattr(blue_box_bg, 'size', blue_box.size))

        # ── 파란 박스 안 컨텐츠 ──
        content_box = BoxLayout(orientation='vertical', size_hint_x=0.8, spacing=5)

        label1 = Label(text='[b]Select control components[/b]', markup=True, color=(0,0,0,1))
        label2 = Label(text="You can manually place obstacles \nand control the robot's goal position.", color=(0,0,0,0.5))

        content_box.add_widget(label1)
        content_box.add_widget(label2)

        def update_font_size(*args):
            label1.font_size = blue_box.height * 0.25
            label2.font_size = blue_box.height * 0.18

        blue_box.bind(size=update_font_size)

        arrow_btn = Button(
            background_normal='../assets/button1.png',
            background_down='../assets/button1.png',  # 눌렀을 때도 같은 이미지 사용
            size_hint_x=0.2,
            background_color=(0, 0, 0, 0),  # 배경색 제거 (투명)
            border=(0,0,0,0)  # 경계선 제거
        )


        blue_box.add_widget(content_box)
        blue_box.add_widget(arrow_btn)

        self.grid_area.add_widget(blue_box)

        self.add_widget(self.header)
        self.add_widget(self.grid_area)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size

    def update_grid_bg(self, *args):
        self.grid_bg.pos = self.grid_area.pos
        self.grid_bg.size = self.grid_area.size
