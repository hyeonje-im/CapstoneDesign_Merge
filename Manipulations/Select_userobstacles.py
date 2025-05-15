from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, RoundedRectangle, Line


class SelectUserObstacles(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (0.35, 1)
        self.pos_hint = {'left': 1}
        self.padding = [10, 10, 10, 10]
        self.orientation = 'vertical'
        

        with self.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size,)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 상단 박스
        top_bar = self._create_plain_box(size_hint = (1,0.2))
        self.add_widget(top_bar)

        # 중간 2박스 영역 (수평 박스)
        middle_bar = BoxLayout(orientation='horizontal', size_hint=(1, 0.8))
        for _ in range(2):
            middle_bar.add_widget(self._create_plain_box())
        self.add_widget(middle_bar)

        
    def _create_plain_box(self, **kwargs):
        box = Widget(**kwargs)
        with box.canvas.before:
            Color(37/255, 40/255, 59/255, 1)  # 내부 배경색
            box.bg = Rectangle(pos=box.pos, size=box.size)

            Color(0, 0, 0, 1)  # 검정 테두리
            box.border = Line(rectangle=(box.x, box.y, box.width, box.height), width=1)

        box.bind(
            pos=lambda inst, val: (
                setattr(inst.bg, 'pos', val),
                setattr(inst.border, 'rectangle', (inst.x, inst.y, inst.width, inst.height))
            ),
            size=lambda inst, val: (
                setattr(inst.bg, 'size', val),
                setattr(inst.border, 'rectangle', (inst.x, inst.y, inst.width, inst.height))
            )
        )
        return box

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
