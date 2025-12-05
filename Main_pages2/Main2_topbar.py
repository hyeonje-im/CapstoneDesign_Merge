from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.behaviors import ButtonBehavior
from kivy.uix.popup import Popup
from kivy.graphics import Color, Rectangle
from Utilities.UI_utilities import KLabel, KLine

from kivy.uix.modalview import ModalView
from kivy.app import App
from kivy.clock import Clock

class KButton(ButtonBehavior, BoxLayout):
    def __init__(self, text, bg_color=(0xCC / 255, 0xD6 / 255, 0xE5 / 255, 1), border_color=(0xAB / 255, 0xAB / 255, 0xAB / 255, 1), **kwargs):
        super().__init__(orientation='vertical', size_hint=(0.15, 1), **kwargs)

        self.label = KLabel(
            text=text, font_size=20,
            color=(0,0,0,1),
            halign='center', valign='middle'
        )
        self.label.bind(size=self.update_label_text_size)
        self.add_widget(self.label)

        # 배경 테두리
        with self.canvas.before:
            # 배경색
            Color(*bg_color)
            self.bg_rect = Rectangle(pos=self.pos, size=self.size)

            #테두리선 색상
            Color(*border_color)
            self.border = KLine(self)

        
        self.bind(pos=self.update_graphics, size=self.update_graphics)

    def update_label_text_size(self, instance, size):
        instance.text_size = size

    def update_graphics(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def set_text(self, text):
        self.label.text = text



class TopBar(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='horizontal', size_hint_y=0.1, padding=5, spacing=5, **kwargs)

        with self.canvas.before:
            Color(0xF5 / 255, 0xF7 / 255, 0xFC / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 왼쪽 메인 버튼
        self.main_button = KButton(text="메인")
        self.main_button.bind(on_press=self.go_main_direct)
        self.add_widget(self.main_button)

        # 단일 제어 모드
        self.controller_button = KButton(text="단일 제어 모드")
        self.add_widget(self.controller_button)

        # 시나리오 모드
        self.extra_button = KButton(text="시나리오 모드")
        self.add_widget(self.extra_button)

        # 우측 여백
        self.add_widget(Widget())
        Clock.schedule_once(self._late_bind, 0)

  
    def _late_bind(self, *_):
        app = App.get_running_app()
        if not app or not hasattr(app, "root"):
            return
        sm = app.root
        # 버튼 상태 갱신
        sm.bind(current=lambda *args: self.refresh_buttons())
        # 최초 1회 상태 갱신
        self.refresh_buttons()

    def refresh_buttons(self):
        app = App.get_running_app()
        if not app or not hasattr(app, "root"):
            return
        current = app.root.current

        # 중복 바인딩 방지
        self.controller_button.unbind(on_press=self.switch_to_main)
        self.extra_button.unbind(on_press=self.switch_to_scenario)

        # 단일 제어 모드 버튼
        self.controller_button.set_text("단일 제어 모드")
        self.controller_button.disabled = False
        self.controller_button.opacity = 1
        self.controller_button.bind(on_press=self.switch_to_main)

        # 시나리오 모드 버튼
        self.extra_button.set_text("시나리오 모드")
        self.extra_button.disabled = False
        self.extra_button.opacity = 1
        self.extra_button.bind(on_press=self.switch_to_scenario)

    def switch_to_main(self, *_):
        app = App.get_running_app()
        if app and hasattr(app, "root"):
            app.root.current = "main"

    def switch_to_scenario(self, *_):
        app = App.get_running_app()
        if app and hasattr(app, "root"):
            app.root.current = "scenario"

    def go_main_direct(self, *_):
        self.switch_to_main()

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size