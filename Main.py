import sys
import os
import threading

# (선택) 현재 파일 기준 상대 임포트가 필요하면 유지
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import OpenCV.code.main as backend

from kivy.app import App
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager
from Main_pages2.Main_layout2 import MainLayout2
from Utilities.UI_utilities import KLabel, KLine

class MyScreenManager(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_widget(MainLayout2(name='Main_layout2'))  # 1번 화면

class MyApp(App):
    def build(self):
        # 1) OpenCV 백엔드 스레드 기동 (창 숨김)
        os.environ["SHOW_CV_WINDOWS"] = "0"

        t = threading.Thread(target=backend.main, daemon=True)
        t.start()

        # 2) Kivy 키 입력을 백엔드로 전달 (기존 if key == ... 분기 재사용)
        def _on_key_down(window, keycode, scancode, codepoint, modifiers):
            # keycode는 (key, key_str) 또는 (scan, name) 형태일 수 있음
            # 보통 (key, key_str) = (97, 'a') 형태. 안전하게 처리:
            name = None
            if isinstance(keycode, (list, tuple)):
                # (key, name) / (scan, name) 중 name을 우선 사용
                if len(keycode) >= 2 and isinstance(keycode[1], str):
                    name = keycode[1]
                elif isinstance(keycode[0], str):
                    name = keycode[0]
            elif isinstance(keycode, str):
                name = keycode

            if name:
                # 한 글자 키(a..z, 0..9 등)는 그대로 전달
                if len(name) == 1:
                    backend.push_keycode(ord(name))
                else:
                    # 자주 쓰는 특수키 매핑 (원하면 확장)
                    special_map = {
                        'escape': ord('q'),   # ESC를 q로 매핑(종료)
                        # 'enter': ord('\r'), # 필요시 추가
                    }
                    if name in special_map:
                        backend.push_keycode(special_map[name])
            return False  # 다른 위젯에도 이벤트 전달

        Window.bind(on_key_down=_on_key_down)

        # 3) UI 반환
        return MyScreenManager()

if __name__ == "__main__":
    MyApp().run()
