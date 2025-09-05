import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


from kivy.app import App
from kivy.uix.screenmanager import ScreenManager 
from Main_pages2.Main_layout2 import MainLayout2
from Utilities.UI_utilities import KLabel, KLine

class MyScreenManager(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_widget(MainLayout2(name = 'Main_layout2')) # 1번 화면
class MyApp(App):
    def build(self):
        return MyScreenManager()



if __name__ == "__main__":
    MyApp().run()
