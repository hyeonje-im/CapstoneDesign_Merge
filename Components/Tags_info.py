from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, RoundedRectangle
from kivy.uix.floatlayout import FloatLayout

class TagsInfoWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', spacing=10, padding=5, **kwargs)

        # 전체 배경
        with self.canvas.before:
            Color(37 / 255, 40 / 255, 59 / 255, 1)  
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[10])
        self.bind(pos=self.update_bg, size=self.update_bg)

        # ── view_all_btn → 라운드된 흰색 박스 안에 라벨 ──
        view_all_container = FloatLayout(size_hint_y=None, height=30)
        with view_all_container.canvas.before:
            Color(115 / 255, 103 / 255, 239 / 255, 1)
            view_all_container.bg = RoundedRectangle(pos=view_all_container.pos, size=view_all_container.size, radius=[7])
        view_all_container.bind(pos=lambda *a: setattr(view_all_container.bg, 'pos', view_all_container.pos),
                                size=lambda *a: setattr(view_all_container.bg, 'size', view_all_container.size))

        view_all_label = Label(
            text='View all Tags',
            color=(1, 1, 1, 1),
            bold=True,
            font_size=15,
            pos_hint={'center_x': 0.5, 'center_y': 0.5}  # ★ 중앙 정렬!
        )

        view_all_container.add_widget(view_all_label)


        # 태그 이미지 박스 ──
        aac4ff_box = BoxLayout(orientation='vertical', padding=5, spacing=5)
        with aac4ff_box.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1) 
            aac4ff_box.bg = RoundedRectangle(pos=aac4ff_box.pos, size=aac4ff_box.size, radius=[7])
        aac4ff_box.bind(pos=lambda *a: setattr(aac4ff_box.bg, 'pos', aac4ff_box.pos),
                        size=lambda *a: setattr(aac4ff_box.bg, 'size', aac4ff_box.size))

        # ── AAC4FF 박스 안의 라벨 ──
        label = Label(
            text='Tag image',
            size_hint_y=None, height=15,
            color=(1, 1, 1, 1),
            bold=True
        )

        # ── center_widget: 흰 박스 (안에 버튼 2개) ──
        center_widget = FloatLayout()
        with center_widget.canvas.before:
            Color(1, 1, 1, 1)
            center_widget.bg = RoundedRectangle(pos=center_widget.pos, size=center_widget.size, radius=[7])
        center_widget.bind(pos=lambda *a: setattr(center_widget.bg, 'pos', center_widget.pos),
                           size=lambda *a: setattr(center_widget.bg, 'size', center_widget.size))

        left_btn = Button(
            text='<', bold=True,
            size_hint=(None, None), size=(30, 30),
            pos_hint={'x': 0, 'center_y': 0.5},
            background_normal='',
            background_color=(1, 1, 1, 1),
            color=(0, 0, 0, 1)
        )
        right_btn = Button(
            text='>', bold=True,
            size_hint=(None, None), size=(30, 30),
            pos_hint={'right': 1, 'center_y': 0.5},
            background_normal='',
            background_color=(1, 1, 1, 1),
            color=(0, 0, 0, 1)
        )
        center_widget.add_widget(left_btn)
        center_widget.add_widget(right_btn)

        # AAC4FF 박스에 라벨과 center_widget 추가
        aac4ff_box.add_widget(label)
        aac4ff_box.add_widget(center_widget)

        # ── tag_num_btn → 박스 안에 라벨 ──
        tag_num_container = FloatLayout(size_hint_y=None, height=30)
        with tag_num_container.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            tag_num_container.bg = RoundedRectangle(pos=tag_num_container.pos, size=tag_num_container.size, radius=[7])
        tag_num_container.bind(pos=lambda *a: setattr(tag_num_container.bg, 'pos', tag_num_container.pos),
                            size=lambda *a: setattr(tag_num_container.bg, 'size', tag_num_container.size))

        tag_num_label = Label(
            text='Tag number',
            color=(1, 1, 1, 1),
            bold=True,
            font_size=15,
            pos_hint={'center_x': 0.5, 'center_y': 0.5} #중앙 정렬렬
        )

        tag_num_container.add_widget(tag_num_label)

        
        # ── 최종 조립 ──
        self.add_widget(view_all_container)
        self.add_widget(aac4ff_box)
        self.add_widget(tag_num_container)

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
