from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.graphics import Color, RoundedRectangle, Rectangle
from kivy.core.window import Window


class VideoCaptureWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.orientation = 'vertical'
        self.size_hint = (None, None)
        self.size = (600, 600)

        # 상단 바 (48px)
        top_bar = Widget(size_hint=(1, None), height=48)
        with top_bar.canvas.before:
            Color(0xB1 / 255, 0xB2 / 255, 0xFF / 255, 1)  # #B1B2FF
            self.top_rect = RoundedRectangle(pos=top_bar.pos, size=top_bar.size,
                                      radius=[(10, 10), (10, 10), (0, 0), (0, 0)]
                                      )
        top_bar.bind(pos=self._update_top_rect, size=self._update_top_rect)

        # 아래 영상 영역 (374 - 48 = 326px)
        video_area = Widget(size_hint=(1, 1))
        with video_area.canvas.before:
            Color(1, 1, 1, 1)  # 진한 연파랑, 테스트용
            self.video_rect = Rectangle(pos=video_area.pos, size=video_area.size)
        video_area.bind(pos=self._update_video_rect, size=self._update_video_rect)

        # 레이아웃에 추가
        self.add_widget(top_bar)
        self.add_widget(video_area)

    def _update_top_rect(self, instance, *args):
        self.top_rect.pos = instance.pos
        self.top_rect.size = instance.size

    def _update_video_rect(self, instance, *args):
        self.video_rect.pos = instance.pos
        self.video_rect.size = instance.size
