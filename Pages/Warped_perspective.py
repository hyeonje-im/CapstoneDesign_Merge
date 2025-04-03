from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.graphics import Color, RoundedRectangle, Rectangle


class WarpedPerspectiveWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.orientation = 'vertical'
        self.size_hint = (None, None)
        self.size = (600, 600) # 위젯 크기

        # 상단 바 (48px 높이)
        top_bar = Widget(size_hint=(1, None), height=48)
        with top_bar.canvas.before:
            Color(0xB1 / 255, 0xB2 / 255, 0xFF / 255, 1)  # #B1B2FF
            self.top_rect = RoundedRectangle(
                pos=top_bar.pos,
                size=top_bar.size,
                radius=[(10, 10), (10, 10), (0, 0), (0, 0)]  # 상단 모서리만 둥글게
            )
        top_bar.bind(pos=self._update_top_rect, size=self._update_top_rect)

        # 영상 영역 (흰색)
        video_area = Widget(size_hint=(1, 1))
        with video_area.canvas.before:
            Color(1, 1, 1, 1)  # 흰색
            self.video_rect = Rectangle(pos=video_area.pos, size=video_area.size)
        video_area.bind(pos=self._update_video_rect, size=self._update_video_rect)

        self.add_widget(top_bar)
        self.add_widget(video_area)

    def _update_top_rect(self, instance, *args):
        self.top_rect.pos = instance.pos
        self.top_rect.size = instance.size

    def _update_video_rect(self, instance, *args):
        self.video_rect.pos = instance.pos
        self.video_rect.size = instance.size
