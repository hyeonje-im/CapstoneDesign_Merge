import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


from kivy.app import App
from kivy.uix.screenmanager import ScreenManager
from Pages.Main_layout import ColoredScreen  # ColoredScreen 파일명 맞게 임포트
from Manipulations.Advanecd_mainlayout import AdvancedMainLayout  # AdvancedMainLayout 파일명 맞게 임포트


class MyScreenManager(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.add_widget(ColoredScreen(name='colored_screen'))
        self.add_widget(AdvancedMainLayout(name='Advanced_mainlayout'))

class MyApp(App):
    def build(self):
        return MyScreenManager()

if __name__ == "__main__":
    MyApp().run()
