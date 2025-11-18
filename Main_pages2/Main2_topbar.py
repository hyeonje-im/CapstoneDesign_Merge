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

    # 편의: 텍스트 바꾸기
    def set_text(self, text):
        self.label.text = text


class TopBar(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='horizontal', size_hint_y=0.1, padding=5, spacing=5, **kwargs)

        with self.canvas.before:
            Color(0x2E / 255, 0x33 / 255, 0x49 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # === 왼쪽: 메인으로 ===
        self.main_button = KButton(text="메인")
        self.main_button.bind(on_press=self.go_main_direct)
        self.add_widget(self.main_button)

        # === 가운데 A: 단일 제어 모드(= 메인으로 가기) ===
        self.controller_button = KButton(text="단일 제어 모드")
        self.add_widget(self.controller_button)

        # === 가운데 B: 시나리오 모드(= 시나리오로 가기) ===
        self.extra_button = KButton(text="시나리오 모드")
        self.add_widget(self.extra_button)

        # === 우측 여백 ===
        self.add_widget(Widget())

        # ScreenManager.current 변화 감지해서 버튼 토글
        Clock.schedule_once(self._late_bind, 0)

    # --- 내부 유틸 ---
    def _late_bind(self, *_):
        app = App.get_running_app()
        if not app or not hasattr(app, "root"):
            return
        sm = app.root
        # 화면 바뀔 때마다 버튼 상태 갱신
        sm.bind(current=lambda *args: self.refresh_buttons())
        # 최초 1회 상태 갱신
        self.refresh_buttons()

    def refresh_buttons(self):
        """현재 화면명에 따라 버튼 텍스트/동작/표시를 토글"""
        app = App.get_running_app()
        if not app or not hasattr(app, "root"):
            return
        current = app.root.current

        # 중복 바인딩 방지: 기존 바인딩 해제
        self.controller_button.unbind(on_press=self.go_main_direct)
        self.controller_button.unbind(on_press=self.switch_to_main)
        self.extra_button.unbind(on_press=self.switch_to_scenario)

        if current == "scenario":
            # 시나리오 화면이면: '단일 제어 모드' 버튼만 보이게
            self.controller_button.set_text("단일 제어 모드")
            self.controller_button.opacity = 1
            self.controller_button.disabled = False
            self.controller_button.bind(on_press=self.switch_to_main)

            self.extra_button.opacity = 0
            self.extra_button.disabled = True
        else:
            # 메인 화면이면: '시나리오 모드' 버튼만 보이게
            self.extra_button.set_text("시나리오 모드")
            self.extra_button.opacity = 1
            self.extra_button.disabled = False
            self.extra_button.bind(on_press=self.switch_to_scenario)

            self.controller_button.opacity = 0
            self.controller_button.disabled = True

    def switch_to_main(self, *_):
        app = App.get_running_app()
        if app and hasattr(app, "root"):
            app.root.current = "main"

    def switch_to_scenario(self, *_):
        app = App.get_running_app()
        if app and hasattr(app, "root"):
            app.root.current = "scenario"

    def go_main_direct(self, *_):
        """왼쪽 '메인' 버튼: 언제 눌러도 메인으로"""
        self.switch_to_main()

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
