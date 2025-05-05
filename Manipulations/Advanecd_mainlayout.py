from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import Color, Rectangle

class AdvancedMainLayout(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        layout = FloatLayout()
        with layout.canvas.before:
            Color(1, 1, 1, 1)  # 완전 흰색
            self.bg = Rectangle(pos=layout.pos, size=layout.size)
        layout.bind(pos=self.update_bg, size=self.update_bg)
        
        self.add_widget(layout)

    def update_bg(self, *args):
        self.bg.pos = self.children[0].pos
        self.bg.size = self.children[0].size
