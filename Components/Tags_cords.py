from kivy.uix.widget import Widget
from kivy.graphics import Color, RoundedRectangle


class TagsCordsWidget(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.size_hint = (None, None)
        self.size = (200, 600)  # 위젯 크기
        self.pos = (10, 10)

        # 바깥 보라색 배경
        with self.canvas.before:
            Color(0xD2 / 255, 0xDA / 255, 0xFF / 255, 1)  # #D2DAFF
            self.bg_rect = RoundedRectangle(
                pos=self.pos,
                size=self.size,
                radius=[(10, 10), (10, 10), (0, 0), (0, 0)]
            )

        self.bind(pos=self._update_rect, size=self._update_rect)

        # 내부 위젯 추가
        self._add_inner_white_box()

    def _update_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size
        if hasattr(self, "inner_box"):
            self._update_inner_box()

    def _add_inner_white_box(self):
        self.inner_box = Widget(size_hint=(None, None))
        self.inner_box.size = (180, 580)

        with self.inner_box.canvas.before:
            Color(1, 1, 1, 1)  # 흰색 배경
            self.inner_rect = RoundedRectangle(
                pos=self.inner_box.pos,
                size=self.inner_box.size,
                radius=[(10, 10), (10, 10), (10, 10), (10, 10)]
            )

        self.inner_box.bind(pos=self._update_inner_box, size=self._update_inner_box)
        self.add_widget(self.inner_box)

        # 내부 타이틀 바 추가
        self._add_title_bar()

        self._update_inner_box()

    def _update_inner_box(self, *args):
        # 부모 기준 중앙 위치 계산
        parent_x, parent_y = self.pos
        parent_w, parent_h = self.size
        inner_w, inner_h = self.inner_box.size

        center_x = parent_x + (parent_w - inner_w) / 2
        center_y = parent_y + (parent_h - inner_h) / 2

        self.inner_box.pos = (center_x, center_y)
        self.inner_rect.pos = self.inner_box.pos
        self.inner_rect.size = self.inner_box.size

        # 타이틀 바 위치도 업데이트
        if hasattr(self, "title_bar"):
            self._update_title_bar()

    def _add_title_bar(self):
        self.title_bar = Widget(size_hint=(1, None), height=40)

        with self.title_bar.canvas.before:
            Color(0xAA / 255, 0xC4 / 255, 0xFF / 255, 1)  # #AAC4FF
            self.title_rect = RoundedRectangle(
                pos=self.title_bar.pos,
                size=self.title_bar.size,
                radius=[(10, 10), (10, 10), (0, 0), (0, 0)]  # 상단 둥글게
            )

        self.title_bar.bind(pos=self._update_title_bar, size=self._update_title_bar)
        self.inner_box.add_widget(self.title_bar)

    def _update_title_bar(self, *args):
        # inner_box의 위치 기준으로 계산
        inner_x, inner_y = self.inner_box.pos
        inner_w, inner_h = self.inner_box.size
        bar_height = self.title_bar.height

        self.title_bar.pos = (inner_x, inner_y + inner_h - bar_height)
        self.title_rect.pos = self.title_bar.pos
        self.title_rect.size = (inner_w, bar_height)
