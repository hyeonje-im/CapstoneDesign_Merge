from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.button import Button
from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.boxlayout import BoxLayout


# 한글 폰트 적용된 Kivy Label
def KLabel(text, **kwargs):
    font_path = "assets/fonts/Pretendard-Regular.otf"
    return Label(text=text, font_name=font_path, **kwargs)

# 테두리 선 그리기 함수
def KLine(widget, **kwargs):
    return Line(rectangle=(widget.x, widget.y, widget.width, widget.height), width=1, **kwargs)

# 어두운 셀 생성 함수
def make_darkcell(text, **kwargs):
    box = BoxLayout(
    size_hint_y  = None,
    height = 30, **kwargs)
    
    #배경 색상
    with box.canvas.before:
        Color(0x25/255, 0x28/255, 0x3B/255, 1)  # 25283B
        box.bg = Rectangle(pos=box.pos, size=box.size)

    #테두리 선
    with box.canvas.after:
        Color(0, 0, 0, 1)
        box.border = Line(rectangle = (box.x, box.y, box.width, box.height), width=1)
    
    def update_graphics(*args):
        box.bg.pos = box.pos
        box.bg.size = box.size
        box.border.rectangle = (box.x, box.y, box.width, box.height)

    box.bind(pos=update_graphics, size=update_graphics)
    
    #텍스트 추가
    box.add_widget(KLabel(text=text, font_size=13, color=(1,1,1,1)))
    return box

# 밝은 셀 생성 함수
def make_brightcell(text, **kwargs):
    box = BoxLayout(
    size_hint_y  = None,
    height = 30, **kwargs)
    
    #배경 색상
    with box.canvas.before:
        Color(0x2E/255, 0x33/255, 0x49/255, 1) # 2E3349
        box.bg = Rectangle(pos=box.pos, size=box.size)

    #테두리 선
    with box.canvas.after:
        Color(0, 0, 0, 1)
        box.border = Line(rectangle = (box.x, box.y, box.width, box.height), width=1)
    
    def update_graphics(*args):
        box.bg.pos = box.pos
        box.bg.size = box.size
        box.border.rectangle = (box.x, box.y, box.width, box.height)

    box.bind(pos=update_graphics, size=update_graphics)
    
    #텍스트 추가
    box.add_widget(KLabel(text=text, font_size=13, color=(1,1,1,1)))
    return box

class KButton(ButtonBehavior, BoxLayout):
    def __init__(self, text, **kwargs):
        super().__init__(**kwargs)
        self.size_hint_y = None
        self.height = 30
        self.text = text

        # 색상 정의
        self.normal_color = (0x2E/255, 0x33/255, 0x49/255, 1)  # 밝은 셀
        self.down_color   = (0x25/255, 0x28/255, 0x3B/255, 1)  # 어두운 셀

        # 배경 + 테두리
        with self.canvas.before:
            Color(*self.normal_color)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = Line(rectangle=(self.x, self.y, self.width, self.height), width=1)

        self.bind(pos=self._update_graphics, size=self._update_graphics)

        # 내부 텍스트
        self.label = KLabel(text=text, font_size=13, color=(1, 1, 1, 1))
        self.add_widget(self.label)

    def _update_graphics(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def on_press(self):
        with self.canvas.before:
            Color(*self.down_color)
            self.bg = Rectangle(pos=self.pos, size=self.size)

    def on_release(self):
        with self.canvas.before:
            Color(*self.normal_color)
            self.bg = Rectangle(pos=self.pos, size=self.size)




class KRoundSquareButton(ButtonBehavior, BoxLayout):
    def __init__(self, text="", size=40, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (None, None)
        self.size = (size, size)   # 정사각형 고정
        self.text = text

        # 색상 정의 (기존 KButton과 동일)
        self.normal_color = (0x2E/255, 0x33/255, 0x49/255, 1)  # 밝은 셀
        self.down_color   = (0x25/255, 0x28/255, 0x3B/255, 1)  # 어두운 셀

        # 배경 + 테두리 (라운드 처리)
        with self.canvas.before:
            Color(*self.normal_color)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[5, 5, 5, 5])
        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = Line(rounded_rectangle=(self.x, self.y, self.width, self.height, 5), width=1)

        self.bind(pos=self._update_graphics, size=self._update_graphics)

        # 내부 텍스트
        from kivy.uix.label import Label
        from kivy.utils import get_color_from_hex
        font_path = "assets/fonts/Pretendard-Regular.otf"
        self.label = Label(text=text, font_size=13, color=(1, 1, 1, 1), font_name=font_path)
        self.add_widget(self.label)

    def _update_graphics(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rounded_rectangle = (self.x, self.y, self.width, self.height, 5)

    def _sync_size(self, *args):
        # 가로/세로가 항상 동일하도록 강제
        side = min(self.width, self.height)
        self.size = (side, side)

    def on_press(self):
        with self.canvas.before:
            Color(*self.down_color)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[5, 5, 5, 5])

    def on_release(self):
        with self.canvas.before:
            Color(*self.normal_color)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[5, 5, 5, 5])
