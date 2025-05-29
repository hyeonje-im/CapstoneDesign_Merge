import os

from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.lang import Builder


kv_path = os.path.join(os.path.dirname(__file__), "select_userobstacles.kv")
Builder.load_file(kv_path)


class SelectUserObstacles(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        with self.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

class PlainBox(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        with self.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)

            Color(0, 0, 0, 1)
            self.border = Line(rectangle=(self.x, self.y, self.width, self.height), width=1)

        self.bind(
            pos=lambda inst, val: (
                setattr(inst.bg, 'pos', val),
                setattr(inst.border, 'rectangle', (inst.x, inst.y, inst.width, inst.height))
            ),
            size=lambda inst, val: (
                setattr(inst.bg, 'size', val),
                setattr(inst.border, 'rectangle', (inst.x, inst.y, inst.width, inst.height))
            )
        )
