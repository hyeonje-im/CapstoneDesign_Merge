from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.popup import Popup
from kivy.graphics import Color, Rectangle
from Utilities.UI_utilities import KLabel, KLine
from Controls.Controls_Mainlayout import ControlsMain

from kivy.uix.modalview import ModalView

class KButton(ButtonBehavior, BoxLayout):
    def __init__(self, text, **kwargs):
        super().__init__(orientation='vertical', size_hint=(0.15, 1), **kwargs)
        self.label = KLabel(text=text, font_size=20, color=(1, 1, 1, 1), halign='center', valign='middle')
        self.label.bind(size=self.update_label_text_size)
        self.add_widget(self.label)

        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)

        self.bind(pos=self.update_border, size=self.update_border)

    def update_label_text_size(self, instance, size):
        instance.text_size = size

    def update_border(self, *args):
        self.border.rectangle = (self.x, self.y, self.width, self.height)


class TopBar(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='horizontal', size_hint_y=0.1, padding=5, spacing=5, **kwargs)

        with self.canvas.before:
            Color(0x2E / 255, 0x33 / 255, 0x49 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # === 버튼 1: 메인 ===
        self.main_button = KButton(text="메인")
        self.main_button.bind(on_press=self.on_main_press)
        self.add_widget(self.main_button)

        # === 버튼 2: 컨트롤러 ===
        self.controller_button = KButton(text="컨트롤러")
        self.controller_button.bind(on_press=self.open_controller_popup)
        self.add_widget(self.controller_button)

        # === 버튼 3: 추가기능 ===
        self.extra_button = KButton(text="추가기능")
        self.extra_button.bind(on_press=self.on_extra_press)
        self.add_widget(self.extra_button)

        # === 우측 빈 공간 채우기 ===
        self.add_widget(Widget())

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def open_controller_popup(self, instance):
        popup = ModalView(            
            size_hint=(1,1),
            auto_dismiss=True,
            background = '',
            background_color = (0,0,0,0),
            
        )
        popup.add_widget(ControlsMain(size_hint=(1,1),
        pos_hint={'center_x': 0.5, 'center_y': 0.5}))
        popup.open()

    def on_main_press(self, instance):
        print("메인 버튼 클릭됨")
        # 여기에 메인 관련 기능 추가 예정

    def on_extra_press(self, instance):
        print("추가기능 버튼 클릭됨")
        # 여기에 추가기능 관련 기능 추가 예정
