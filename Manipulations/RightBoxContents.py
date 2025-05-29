import os

from kivy.uix.boxlayout import BoxLayout
from kivy.properties import NumericProperty
from kivy.lang import Builder

kv_path = os.path.join(os.path.dirname(__file__), "rightboxcontents.kv")
Builder.load_file(kv_path)
class RightBoxContents(BoxLayout):
    font_size = NumericProperty(14)  # 폰트 크기 사용자 설정 가능
