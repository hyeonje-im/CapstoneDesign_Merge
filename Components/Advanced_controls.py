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

                # ── blue_box들을 수직으로 정렬할 컨테이너
        box_container = BoxLayout(orientation='vertical',
                                  spacing=10,
                                  size_hint=(0.95, 0.95),
                                  pos_hint={'center_x': 0.5, 'center_y': 0.5})

        # ✅ [1] 첫 번째 blue_box 
        blue_box1 = BoxLayout(orientation='horizontal', padding=10, spacing=10,
                              size_hint=(1, 1/3))
        with blue_box1.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)  
            blue_box1_bg = RoundedRectangle(pos=blue_box1.pos, size=blue_box1.size, radius=[10])
        blue_box1.bind(pos=lambda *a: setattr(blue_box1_bg, 'pos', blue_box1.pos),
                       size=lambda *a: setattr(blue_box1_bg, 'size', blue_box1.size))

        content_box = BoxLayout(orientation='vertical', size_hint_x=0.6, spacing=5)
        label1 = Label(text='[b]Select control components[/b]', markup=True, color=(1,1,1,1))
        label2 = Label(text="You can manually place obstacles \nand control the robot's goal position.", color=(1,1,1,0.5))
        content_box.add_widget(label1)
        content_box.add_widget(label2)

        def update_font_size(*args):
            label1.font_size = blue_box1.height * 0.25
            label2.font_size = blue_box1.height * 0.18
        blue_box1.bind(size=update_font_size)

        temp_box1 = FloatLayout(size_hint=(0.2, 1))
        with temp_box1.canvas.before:
            Color(177/255, 178/255, 1, 1)
            temp_box1_bg = RoundedRectangle(pos=temp_box1.pos, size=temp_box1.size, radius=[7])
        temp_box1.bind(pos=lambda *a: setattr(temp_box1_bg, 'pos', temp_box1.pos),
                       size=lambda *a: setattr(temp_box1_bg, 'size', temp_box1.size))

        go_btn = Button(text='GO!', bold=True,
                        size_hint=(0.7, 0.7),
                        pos_hint={'center_x':0.5, 'center_y':0.5},
                        background_normal='',
                        background_color=(177/255, 178/255, 1, 1),
                        color=(1,1,1,1))
        go_btn.bind(size=lambda *_: setattr(go_btn, 'font_size', go_btn.height * 0.5))
        if on_go:
            go_btn.bind(on_release=on_go)

        temp_box1.add_widget(go_btn)
        blue_box1.add_widget(content_box)
        blue_box1.add_widget(temp_box1)

        # ✅ [2] 두 번째 blue_box (내용 없음)
        blue_box2 = BoxLayout(orientation='horizontal', padding=10, spacing=10,
                              size_hint=(1, 1/3))
        with blue_box2.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            blue_box2_bg = RoundedRectangle(pos=blue_box2.pos, size=blue_box2.size, radius=[10])
        blue_box2.bind(pos=lambda *a: setattr(blue_box2_bg, 'pos', blue_box2.pos),
                       size=lambda *a: setattr(blue_box2_bg, 'size', blue_box2.size))

        # ✅ [3] 세 번째 blue_box (내용 없음)
        blue_box3 = BoxLayout(orientation='horizontal', padding=10, spacing=10,
                              size_hint=(1, 1/3))
        with blue_box3.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            blue_box3_bg = RoundedRectangle(pos=blue_box3.pos, size=blue_box3.size, radius=[10])
        blue_box3.bind(pos=lambda *a: setattr(blue_box3_bg, 'pos', blue_box3.pos),
                       size=lambda *a: setattr(blue_box3_bg, 'size', blue_box3.size))

        # ── 추가
        box_container.add_widget(blue_box1)
        box_container.add_widget(blue_box2)
        box_container.add_widget(blue_box3)

        self.grid_area.add_widget(box_container)


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
