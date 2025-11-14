import sys
import os
import threading
from kivy.app import App
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager, FadeTransition

# ==========================
# 1️⃣ 경로 설정
# ==========================
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# ==========================
# 2️⃣ 백엔드 로드 (OpenCV)
# ==========================
os.environ["SHOW_CV_WINDOWS"] = "1"   # 개발용만 켜두기
import OpenCV.code.main_merge as backend

# ==========================
# 3️⃣ Kivy UI 화면 임포트
# ==========================
from Main_pages2.Main_layout2 import MainLayout2
from Main_pages2.Scenario_layout import ScenarioLayout  # 새로 추가 예정


# ==========================
# 4️⃣ 화면 관리자
# ==========================
class MyScreenManager(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(transition=FadeTransition(duration=0.3), **kwargs)
        self.add_widget(MainLayout2(name='main'))
        self.add_widget(ScenarioLayout(name='scenario'))


# ==========================
# 5️⃣ Kivy App
# ==========================
class MyApp(App):
    def build(self):
        # 백엔드 스레드 기동
        t = threading.Thread(target=backend.main, daemon=True)
        t.start()

        # 키 입력 전달
        def _on_key_down(window, keycode, scancode, codepoint, modifiers):
            name = None
            if isinstance(keycode, (list, tuple)):
                if len(keycode) >= 2 and isinstance(keycode[1], str):
                    name = keycode[1]
                elif isinstance(keycode[0], str):
                    name = keycode[0]
            elif isinstance(keycode, str):
                name = keycode

            if name:
                n = name.lower()
                if len(n) == 1:
                    backend.push_keycode(ord(n))
                else:
                    special_map = {'escape': ord('q')}
                    if n in special_map:
                        backend.push_keycode(special_map[n])
            return False

        Window.bind(on_key_down=_on_key_down)
        return MyScreenManager()


# ==========================
# 6️⃣ 실행
# ==========================
if __name__ == "__main__":
    MyApp().run()
