from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, RoundedRectangle, Line


class RobotGoalBox(BoxLayout):
    def __init__(self, robot_name="Robot1", base_color=(1, 1, 1, 0.4), active_color=(1, 1, 1, 1), **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        self.padding = 5
        self.spacing = 5

        self.base_color = base_color
        self.active_color = active_color
        self.selected = False

        # 상단 타이틀
        title = Label(
            text=f"[b]{robot_name}[/b]",
            markup=True,
            size_hint=(1, 0.2),
            halign='center',
            valign='middle',
            color=(1, 1, 1, 1)
        )
        title.bind(size=lambda inst, val: setattr(inst, 'text_size', val))
        self.add_widget(title)

        # ✅ 버튼을 감싸는 wrapper
        color_wrapper = BoxLayout(size_hint=(0.5, 1))
        self.color_button = Button(
            background_normal='',
            background_color=self.base_color,
            size_hint=(1, 1)
        )
        self.color_button.bind(on_press=self.toggle_button)
        color_wrapper.add_widget(self.color_button)

        # 오른쪽 상태 텍스트
        right_box = BoxLayout(orientation='vertical', size_hint=(0.5, 1), padding=3)
        self.status_label = Label(text="Unselected", color=(1, 1, 1, 1), halign='left', valign='middle')
        self.coord_label = Label(text="Specified\n[?, ?]", color=(1, 1, 1, 1), halign='left', valign='middle')
        for lab in [self.status_label, self.coord_label]:
            lab.bind(size=lambda inst, val: setattr(inst, 'text_size', val))
            right_box.add_widget(lab)

        # 수평 박스에 둘 다 넣음
        horizontal = BoxLayout(orientation='horizontal', size_hint=(1, 0.6))
        horizontal.add_widget(color_wrapper)
        horizontal.add_widget(right_box)

        # horizontal 전체만 한 번 추가
        self.add_widget(horizontal)

    def toggle_button(self, instance):
        self.selected = not self.selected
        self.color_button.background_color = self.active_color if self.selected else self.base_color
        self.status_label.text = "Selected" if self.selected else "Unselected"

class SelectGoalPositions(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (0.65, 1)
        self.pos_hint = {'right': 1}
        self.padding = [10, 10, 10, 10]
        self.orientation = 'vertical'

        with self.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # 상단 바
        top_bar = BoxLayout(size_hint=(1, 0.15), orientation='horizontal')
        top_bar.bind(pos=self.update_inner_box, size=self.update_inner_box)
        with top_bar.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            top_bar.bg = Rectangle(pos=top_bar.pos, size=top_bar.size)
            Color(0, 0, 0, 1)
            top_bar.border = Line(rectangle=(top_bar.x, top_bar.y, top_bar.width, top_bar.height), width=1)

        label = Label(
            text='[b]Select goal positions[/b]',
            markup=True,
            color=(1, 1, 1, 1),
            halign='center',
            valign='middle',
            size_hint=(1, 1)
        )
        label.bind(size=lambda inst, val: setattr(inst, 'text_size', val))
        top_bar.add_widget(label)
        self.add_widget(top_bar)

        # 중간 4박스
        middle_bar = BoxLayout(orientation='horizontal', size_hint=(1, 0.75), spacing = 15,
                               padding = [10, 10])
        with middle_bar.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            middle_bar.bg = RoundedRectangle(pos=middle_bar.pos, size=middle_bar.size)
            Color(0, 0, 0, 1)
            middle_bar.border = Line(rectangle=(middle_bar.x, middle_bar.y, middle_bar.width, middle_bar.height), width=1)
       
       
        robot_colors = [
            ((178/255, 255/255, 177/255, 0.5), (178/255, 255/255, 177/255, 1)),
            ((254/255, 255/255, 179/255, 0.5), (254/255, 255/255, 179/255, 1)),
            ((255/255, 178/255, 178/255, 0.5), (255/255, 178/255, 178/255, 1)),
            ((255/255, 217/255, 178/255, 0.5), (255/255, 217/255, 178/255, 1)),
        ]
        for i in range(4):
            middle_bar.add_widget(RobotGoalBox(
                robot_name=f"Robot{i+1}",
                base_color=robot_colors[i][0],
                active_color=robot_colors[i][1]
            ))
        self.add_widget(middle_bar)

        # 하단 바
        bottom_bar = self._create_plain_box(size_hint=(1, 0.1))
        self.add_widget(bottom_bar)
        with top_bar.canvas.before:
            Color(46 / 255, 51 / 255, 73 / 255, 1)
            bottom_bar.bg = Rectangle(pos=top_bar.pos, size=top_bar.size)
    
    
    def _create_plain_box(self, **kwargs):
        box = Widget(**kwargs)
        with box.canvas.before:
            Color(37/255, 40/255, 59/255, 1)
            box.bg = Rectangle(pos=box.pos, size=box.size)
            Color(0, 0, 0, 1)
            box.border = Line(rectangle=(box.x, box.y, box.width, box.height), width=1)
        box.bind(
            pos=lambda inst, val: (
                setattr(inst.bg, 'pos', val),
                setattr(inst.border, 'rectangle', (inst.x, inst.y, inst.width, inst.height))
            ),
            size=lambda inst, val: (
                setattr(inst.bg, 'size', val),
                setattr(inst.border, 'rectangle', (inst.x, inst.y, inst.width, inst.height))
            )
        )
        return box

    def update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def update_inner_box(self, instance, value):
        instance.bg.pos = instance.pos
        instance.bg.size = instance.size
        instance.border.rectangle = (instance.x, instance.y, instance.width, instance.height)
