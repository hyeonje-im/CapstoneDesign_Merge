from kivy.uix.boxlayout import BoxLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from kivy.metrics import dp


class ScenarioOrderList(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", spacing=10, padding=10, **kwargs)

        # ============================
        # 배경
        # ============================
        with self.canvas.before:
            Color(0.96, 0.97, 0.99, 1)  # 연한 배경
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self._update_bg, size=self._update_bg)

        # ============================
        # 제목(옵션: 필요 없으면 삭제)
        # ============================
        # 여기는 나중에 제목을 넣을 수 있음
        # 현재는 아무 위젯도 넣지 않음

        # ============================
        # 스크롤 영역
        # ============================
        self.scroll = ScrollView(size_hint=(1, 1))

        # 리스트 영역 (추후 Row가 들어올 공간)
        self.list_layout = GridLayout(
            cols=1,
            spacing=10,
            size_hint_y=None
        )
        self.list_layout.bind(minimum_height=self.list_layout.setter("height"))

        self.scroll.add_widget(self.list_layout)
        self.add_widget(self.scroll)

    # ============================
    # 배경 업데이트
    # ============================
    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    # ============================
    # 백엔드 연동 (지금은 비어 있음)
    # ============================
    def update_from_framebus(self, dt):
        """
        나중에 FrameBus 데이터 넣어 리스트 갱신하는 코드를 여기 작성하면 됨.
        지금은 완전 빈 상태로 둠.
        """
        pass

    # ============================
    # Row 추가 (샘플 함수)
    # ============================
    def add_row(self, widget):
        """
        나중에 원하는 Row 위젯을 만들어 이 함수로 넣으면 됨.
        """
        self.list_layout.add_widget(widget)
