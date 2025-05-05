from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle


class AdvancedMainLayout(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # 전체 배경
        with self.canvas.before:
            Color(238/255, 241/255, 255/255, 1)  # EEF1FF
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 상단 바
        self.header = BoxLayout(size_hint_y=None, height=40, pos_hint = {'top': 1})
        with self.header.canvas.before:
            Color(177/255, 178/255, 1, 1)  # B1B2FF
            self.header_bg = Rectangle(pos=self.header.pos, size=self.header.size, radius=[7, 7, 0, 0])
        self.header.bind(pos=self.update_header_bg, size=self.update_header_bg)

        # 상단 바 텍스트
        label = Label(text='[b]Advanced Controls[/b]', markup=True, color=(0, 0, 0, 1))
        self.header.add_widget(label)

        # 레이아웃 추가
        layout = FloatLayout()
        layout.add_widget(self.header)

        self.add_widget(layout)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_header_bg(self, *args):
        self.header_bg.pos = self.header.pos
        self.header_bg.size = self.header.size
