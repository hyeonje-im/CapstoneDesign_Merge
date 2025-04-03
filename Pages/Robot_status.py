from kivy.uix.widget import Widget
from kivy.graphics import Color, RoundedRectangle


class RobotStatusWidget(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.size_hint = (None, None)
        self.size = (450, 350) # 위젯 크기
        self.pos = (660, 10)

        # 바깥 배경: 연보라색
        with self.canvas.before:
            Color(0xD2 / 255, 0xDA / 255, 0xFF / 255, 1)  # #D2DAFF
            self.bg_rect = RoundedRectangle(
                pos=self.pos,
                size=self.size,
                radius=[(0, 0), (0, 0), (0, 0), (10, 10)]
            )

        self.bind(pos=self._update_rect, size=self._update_rect)

        # 내부 박스 추가
        self._add_inner_box()

    def _update_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size
        if hasattr(self, "inner_box"):
            self._update_inner_box()

    def _add_inner_box(self):
        self.inner_box = Widget(size_hint=(None, None))
        self.inner_box.size = (430, 330)  # 부모보다 가로세로 10 작게

        with self.inner_box.canvas.before:
            Color(1, 1, 1, 1)  # 흰색
            self.inner_rect = RoundedRectangle(
                pos=self.inner_box.pos,
                size=self.inner_box.size,
                radius=[(10, 10), (10, 10), (10, 10), (10, 10)]
            )

        self.inner_box.bind(pos=self._update_inner_box, size=self._update_inner_box)
        self.add_widget(self.inner_box)

        # 타이틀 바 추가
        self._add_title_bar()

        self._update_inner_box()

    def _update_inner_box(self, *args):
        # 중앙 정렬
        px, py = self.pos
        pw, ph = self.size
        iw, ih = self.inner_box.size

        cx = px + (pw - iw) / 2
        cy = py + (ph - ih) / 2

        self.inner_box.pos = (cx, cy)
        self.inner_rect.pos = self.inner_box.pos
        self.inner_rect.size = self.inner_box.size

        if hasattr(self, "title_bar"):
            self._update_title_bar()

    def _add_title_bar(self):
        self.title_bar = Widget(size_hint=(1, None), height=40)

        with self.title_bar.canvas.before:
            Color(0xAA / 255, 0xC4 / 255, 0xFF / 255, 1)  # #AAC4FF
            self.title_rect = RoundedRectangle(
                pos=self.title_bar.pos,
                size=self.title_bar.size,
                radius=[(10, 10), (10, 10), (0, 0), (0, 0)]
            )

        self.title_bar.bind(pos=self._update_title_bar, size=self._update_title_bar)
        self.inner_box.add_widget(self.title_bar)

    def _update_title_bar(self, *args):
        ix, iy = self.inner_box.pos
        iw, ih = self.inner_box.size
        bh = self.title_bar.height

        self.title_bar.pos = (ix, iy + ih - bh)
        self.title_rect.pos = self.title_bar.pos
        self.title_rect.size = (iw, bh)
