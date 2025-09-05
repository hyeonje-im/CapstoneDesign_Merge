
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.graphics import Color, Rectangle, Line

def KLabel(text, **kwargs):
    font_path = "assets/fonts/Pretendard-Regular.otf"
    return Label(text=text, font_name=font_path, **kwargs)

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
        box.border = Line(rectagle = (box.x, box.y, box.width, box.height), width=1)
    
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
        box.border = Line(rectagle = (box.x, box.y, box.width, box.height), width=1)
    
    def update_graphics(*args):
        box.bg.pos = box.pos
        box.bg.size = box.size
        box.border.rectangle = (box.x, box.y, box.width, box.height)

    box.bind(pos=update_graphics, size=update_graphics)
    
    #텍스트 추가
    box.add_widget(KLabel(text=text, font_size=13, color=(1,1,1,1)))
    return box

