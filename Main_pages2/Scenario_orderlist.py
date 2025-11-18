# Scenario_order_list.py
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.gridlayout import GridLayout
from Utilities.UI_utilities import KLabel


class ScenarioOrderList(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", spacing=5, padding=5, **kwargs)

        # 제목 (KLabel 적용)
        self.title = KLabel(
            text="시나리오 주문 리스트",
            size_hint_y=None,
            height=25,
            font_size=14,
            color=(1, 1, 1, 1),
            halign="center",
            valign="middle"
        )
        self.add_widget(self.title)

        # 스크롤 영역
        self.scroll = ScrollView(size_hint=(1, 1))
        self.list_layout = GridLayout(cols=1, size_hint_y=None, spacing=5, padding=[0,5,0,5])
        self.list_layout.bind(minimum_height=self.list_layout.setter("height"))

        self.scroll.add_widget(self.list_layout)
        self.add_widget(self.scroll)

    # 리스트 항목 추가 (KLabel)
    def add_item(self, text):
        item = KLabel(
            text=text,
            size_hint_y=None,
            height=30,
            font_size=13,
            color=(1, 1, 1, 1),
            halign="left",
            valign="middle",
        )
        self.list_layout.add_widget(item)

    # 리스트 초기화
    def clear_items(self):
        self.list_layout.clear_widgets()
