from kivy.uix.screenmanager import Screen
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle, RoundedRectangle, Line
from kivy.uix.button import Button

class AdvancedMainLayout(Screen):
  def __init__(self, **kwargs):
    super().__init__(**kwargs)

    # 전체 배경
    with self.canvas.before:
      Color(238/255, 241/255, 255/255, 1)
      self.bg = Rectangle(pos=self.pos, size=self.size)
    self.bind(pos=self.update_bg, size=self.update_bg)

    # 상단 바
    self.header = BoxLayout(size_hint_y=None, height=40, pos_hint={'top': 1})
    with self.header.canvas.before:
      Color(177/255, 178/255, 1, 1)
      self.header_bg = RoundedRectangle(pos=self.header.pos, size=self.header.size)
    self.header.bind(pos=self.update_header_bg, size=self.update_header_bg)

    # 상단 바 텍스트
    label = Label(text='[b]Advanced Controls[/b]', markup=True, color=(0, 0, 0, 1))
    self.header.add_widget(label)

    # 레이아웃 추가
    layout = FloatLayout()

    # 오른쪽 박스
    right_box = FloatLayout(size_hint=(0.2, 0.93), pos_hint={'right': 1, 'y': 0})
    with right_box.canvas.before:
      Color(210/255, 218/255, 1, 1)
      right_box_bg = RoundedRectangle(pos=right_box.pos, size=right_box.size, radius=[10])
    right_box.bind(pos=lambda *a: setattr(right_box_bg, 'pos', right_box.pos),
                   size=lambda *a: setattr(right_box_bg, 'size', right_box.size))

    # content_container
    content_container = BoxLayout(orientation='vertical', padding=10, spacing=10,
                                  size_hint=(0.9, None), pos_hint={'center_x': 0.5, 'top': 0.97})
    content_container.bind(minimum_height=content_container.setter('height'))

    with content_container.canvas.before:
      Color(1, 1, 1, 1)
      container_bg = RoundedRectangle(pos=content_container.pos, size=content_container.size, radius=[10])
    content_container.bind(pos=lambda *a: setattr(container_bg, 'pos', content_container.pos),
                           size=lambda *a: setattr(container_bg, 'size', content_container.size))

    items = [
      ("Obstacle rearrangement", "Relocate the grid obstacles and\nmanually define the accessible area."),
      ("Robot's goal positions setting", "Relocate the grid obstacles and\nmanually define the accessible area."),
      ("Low-level robot control", "Try manually controlling the robot's\nstart and stop times."),
      ("Obstacle rearrangement", "Relocate the grid obstacles and\nmanually define the accessible area.")
    ]

    for title, desc in items:
      item_box = BoxLayout(orientation='vertical', size_hint_y=None, height=90)

      title_label = Label(text=f"[b]{title}[/b]", markup=True, color=(0, 0, 0, 1), halign='left', valign='middle')
      desc_label = Label(text=desc, color=(0, 0, 0, 0.5), halign='left', valign='top')

      title_label.bind(size=lambda inst, *a: setattr(inst, 'text_size', inst.size))
      desc_label.bind(size=lambda inst, *a: setattr(inst, 'text_size', inst.size))

      def adjust_title_font(instance, value):
        instance.font_size = instance.height * 0.5
      def adjust_desc_font(instance, value):
        instance.font_size = instance.height * 0.4
      title_label.bind(height=adjust_title_font)
      desc_label.bind(height=adjust_desc_font)

      line_holder = [None]

      with item_box.canvas.after:
        Color(0.7, 0.7, 0.7, 0.1)
        line_holder[0] = Line(points=[0, 0, 0, 0], width=1)

      def update_line(instance, *args):
        line_holder[0].points = [instance.x + 5, instance.y, instance.right - 5, instance.y]

      item_box.bind(pos=update_line, size=update_line)
      update_line(item_box)

      item_box.add_widget(title_label)
      item_box.add_widget(desc_label)

      content_container.add_widget(item_box)

    # content_container를 right_box에 추가
    right_box.add_widget(content_container)

    # 버튼 박스를 content_container 밖에 추가
    button_container = FloatLayout(size_hint=(0.9, None), height=50, pos_hint={'center_x': 0.5, 'y': 0.02})

    with button_container.canvas.before:
      container_bg = RoundedRectangle(
        pos=button_container.pos,
        size=button_container.size,
        radius=[7]
      )
    button_container.bind(
      pos=lambda *a: setattr(container_bg, 'pos', button_container.pos),
      size=lambda *a: setattr(container_bg, 'size', button_container.size)
    )

    button = Button(
      text='[b]Get Started[/b]',
      markup=True,
      color=(0, 0, 0, 1),
      background_normal='',
      background_color=(1, 1, 1, 1),
      size_hint=(1, 1),
      pos_hint={'center_x': 0.5, 'center_y': 0.5}
    )

    button_container.add_widget(button)
    right_box.add_widget(button_container)

    # 왼쪽 박스
    left_box = FloatLayout(size_hint=(0.2, 0.93), pos_hint={'x': 0, 'y': 0})
    with left_box.canvas.before:
      Color(210/255, 218/255, 1, 1)
      left_box_bg = RoundedRectangle(pos=left_box.pos, size=left_box.size, radius=[0, 10, 0, 0])
    left_box.bind(pos=lambda *a: setattr(left_box_bg, 'pos', left_box.pos),
                  size=lambda *a: setattr(left_box_bg, 'size', left_box.size))

    # 레이아웃에 추가
    layout.add_widget(left_box)
    layout.add_widget(right_box)
    layout.add_widget(self.header)

    self.add_widget(layout)

  def update_bg(self, *args):
    self.bg.pos = self.pos
    self.bg.size = self.size

  def update_header_bg(self, *args):
    self.header_bg.pos = self.header.pos
    self.header_bg.size = self.header.size
