from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, RoundedRectangle

class AdvancedcontrolWidget(BoxLayout):  # ✅ BoxLayout 상속
    def __init__(self, on_go=None, **kwargs):  # ✅ 콤마 추가
        super().__init__(orientation='vertical', spacing=0, padding=0, **kwargs)

        # ── 전체 배경
        with self.canvas.before:
            Color(0.823, 0.855, 1.0, 1)  # D2DAFF
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[10])
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── 상단 바
        self.header = BoxLayout(size_hint_y=None, height=40)
        with self.header.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size, radius=[7, 7, 0, 0])
        self.header.bind(pos=self.update_header, size=self.update_header)
        self.header.add_widget(Label(text="Advanced Controls", bold=True, color=(1, 1, 1, 1)))

        # ── 아래 공간
        self.grid_area = FloatLayout()
        with self.grid_area.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)
            self.grid_bg = RoundedRectangle(pos=self.grid_area.pos, size=self.grid_area.size, radius=[0, 0, 7, 7])
        self.grid_area.bind(pos=self.update_grid_bg, size=self.update_grid_bg)

        # ── 중앙 박스
        blue_box = BoxLayout(orientation='horizontal', padding=(10,10), spacing=10,
                             size_hint=(0.95, 0.35), pos_hint={'center_x':0.5, 'center_y':0.5})
        with blue_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)  # AAC4FF
            blue_box_bg = RoundedRectangle(pos=blue_box.pos, size=blue_box.size, radius=[10])
        blue_box.bind(pos=lambda *a: setattr(blue_box_bg, 'pos', blue_box.pos),
                      size=lambda *a: setattr(blue_box_bg, 'size', blue_box.size))

        # ── content_box
        content_box = BoxLayout(orientation='vertical', size_hint_x=0.6, spacing=5)
        label1 = Label(text='[b]Select control components[/b]', markup=True, color=(1,1,1,1))
        label2 = Label(text="You can manually place obstacles \nand control the robot's goal position.", color=(1,1,1,0.5))
        content_box.add_widget(label1)
        content_box.add_widget(label2)

        def update_font_size(*args):
            label1.font_size = blue_box.height * 0.25
            label2.font_size = blue_box.height * 0.18
        blue_box.bind(size=update_font_size)

        # ── temp_box
        temp_box = FloatLayout(size_hint=(0.2, 1))
        with temp_box.canvas.before:
            Color(177/255, 178/255, 1, 1)  # B1B2FF
            temp_box_bg = RoundedRectangle(pos=temp_box.pos, size=temp_box.size, radius=[7])
        temp_box.bind(pos=lambda *a: setattr(temp_box_bg, 'pos', temp_box.pos),
                      size=lambda *a: setattr(temp_box_bg, 'size', temp_box.size))

        # ── GO 버튼
        go_btn = Button(text='GO!', bold=True,
                        size_hint=(0.7, 0.7),
                        pos_hint={'center_x':0.5, 'center_y':0.5},
                        background_normal='',
                        background_color=(177/255, 178/255, 1, 1),
                        color=(1,1,1,1))

        def update_go_font_size(*args):
            go_btn.font_size = go_btn.height * 0.5
        go_btn.bind(size=update_go_font_size)

        # ✅ GO 버튼 클릭 시 → 전달받은 콜백 실행
        if on_go:
            go_btn.bind(on_release=on_go)

        temp_box.add_widget(go_btn)

        # ── blue_box에 추가
        blue_box.add_widget(content_box)
        blue_box.add_widget(temp_box)

        # ── grid_area에 blue_box 추가
        self.grid_area.add_widget(blue_box)

        # ── self에 추가
        self.add_widget(self.header)
        self.add_widget(self.grid_area)

    # ── 위치/크기 업데이트
    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size

    def update_grid_bg(self, *args):
        self.grid_bg.pos = self.grid_area.pos
        self.grid_bg.size = self.grid_area.size
