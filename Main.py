import sys
import os
import threading

# 1) OpenCV 창 비활성화를 "import 전에" 설정
os.environ["SHOW_CV_WINDOWS"] = "1"  # 0: 비활성화, 1: 활성화

# 현재 파일 기준 상대 임포트가 필요하면 유지
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 2) 이제 백엔드 import (환경변수 적용된 상태로 로드됨)
import OpenCV.code.main as backend

from kivy.app import App
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager
from Main_pages2.Main_layout2 import MainLayout2
from Utilities.UI_utilities import KLabel, KLine

class MyScreenManager(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_widget(MainLayout2(name='Main_layout2'))

class MyApp(App):
    def build(self):
        # 3) 백엔드 스레드 기동
        t = threading.Thread(target=backend.main, daemon=True)
        t.start()

        # 4) 키 입력 → 백엔드로 전달
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
                # 소문자로 정규화해서 backend의 ord('n') 등과 일치시키기
                n = name.lower()
                if len(n) == 1:
                    backend.push_keycode(ord(n))
                else:
                    special_map = {
                        'escape': ord('q'),  # 필요시 확장
                    }
                    if n in special_map:
                        backend.push_keycode(special_map[n])
            return False

        Window.bind(on_key_down=_on_key_down)
        return MyScreenManager()

if __name__ == "__main__":
    MyApp().run()
