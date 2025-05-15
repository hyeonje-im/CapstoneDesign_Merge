from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.graphics import Color, RoundedRectangle


class SelectGoalPositions(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (0.65, 1)  # 하단 박스 크기 기준 너비 65%, 높이 100%
        self.pos_hint = {'right': 1}  # 오른쪽 정렬
        self.padding = [0, 10, 10, 10]  # 왼쪽, 위, 오른쪽, 아래 (오른쪽에서 10 띄움)
        self.orientation = 'horizontal'
        self.spacing = 10

        with self.canvas.before:
            Color(37/255, 40/255, 59/255, 1)  # #25283B
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[5])
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 로봇 4개 정보 박스 추가
        for i in range(1, 5):
            self.add_widget(self._create_robot_info(i))

    def _create_robot_info(self, idx):
        # 색상은 임의로 지정, 실제 값은 데이터에 따라 조정 가능
        colors = [(0.7, 1, 0.7, 1), (1, 1, 0.6, 1), (1, 0.7, 0.7, 1), (1, 0.85, 0.6, 1)]
        bg_color = colors[idx - 1]

        box = BoxLayout(orientation='vertical', padding=5, spacing=5)
        box.add_widget(Label(text=f'[b]Robot{idx}[/b]', markup=True, color=(1, 1, 1, 1)))
        box.add_widget(Label(text='Unselected', color=(1, 1, 1, 1)))
        box.add_widget(Label(text='Specified\n[?,?]', color=(1, 1, 1, 1)))  # 좌표는 나중에 바인딩 가능

        # 배경 박스 스타일링
        with box.canvas.before:
            Color(*bg_color)
            box.bg = RoundedRectangle(pos=box.pos, size=box.size, radius=[3])
        box.bind(pos=lambda inst, val: setattr(box.bg, 'pos', val),
                 size=lambda inst, val: setattr(box.bg, 'size', val))

        return box

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
