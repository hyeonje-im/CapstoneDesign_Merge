from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.metrics import dp
from kivy.graphics import Color, RoundedRectangle, Line
from Main_pages2.Main2_grid import GridWidget
from Utilities.UI_utilities import KToggleRoundedButton
# ===============================
# 1) 개별 주문 행 (ID1의 Num, Goal, Status)
# ===============================
class OrderRowWidget(BoxLayout):
    def __init__(self, num="[3,4]", goal="[3,4]", status="도착", **kwargs):
        super().__init__(orientation="horizontal", size_hint_y=None, height=dp(36), spacing=10, **kwargs)

        # Num
        self.add_widget(Label(text=num, size_hint_x=0.3, halign="left", valign="middle"))

        # Goal
        self.add_widget(Label(text=goal, size_hint_x=0.3, halign="left", valign="middle"))

        # Status (초록 버튼처럼)
        st = Button(
            text=status,
            size_hint_x=0.4,
            background_color=(0.75, 1, 0.75, 1),
            background_normal=""
        )
        self.add_widget(st)


class IDColumnWidget(BoxLayout):
    def __init__(self, title="ID1", side="middle", **kwargs):
        super().__init__(orientation="vertical", spacing=10, padding=10, **kwargs)

        # ===== 라운딩 방향 결정 =====
        if side == "left":
            radius = [dp(10), 0, 0, dp(10)]   # 좌상, 우상, 우하, 좌하
        elif side == "right":
            radius = [0, dp(10), dp(10), 0]
        else:
            radius = [0,0,0,0]

        # ==== 컬럼 배경 ====
        with self.canvas.before:
            Color(0.96, 0.97, 0.99, 1)
            self.bg = RoundedRectangle(radius=radius, pos=self.pos, size=self.size)

        self.bind(pos=self.update_bg, size=self.update_bg)

        # ===== Title =====
        title_label = Label(
            text=title,
            size_hint_y=None,
            height=dp(30),
            bold=True,
            color=(0,0,0,1),
            halign="left",
            valign="middle",
        )
        title_label.bind(size=lambda inst, val: setattr(inst, 'text_size', val))
        self.add_widget(title_label)

        # ===== Header =====
        header = GridLayout(cols=3, size_hint_y=None, height=dp(30), spacing=0, padding=0)
        header.add_widget(Label(text="Num", bold=True))
        header.add_widget(Label(text="Goal", bold=True))
        header.add_widget(Label(text="Status", bold=True))
        self.add_widget(header)

        # ===== Scroll List =====
        self.scroll = ScrollView(size_hint=(1, 1))
        self.list_layout = GridLayout(cols=1, size_hint_y=None, spacing=5)
        self.list_layout.bind(minimum_height=self.list_layout.setter("height"))

        self.scroll.add_widget(self.list_layout)
        self.add_widget(self.scroll)

    def update_bg(self, *_):
        self.bg.pos = self.pos
        self.bg.size = self.size



class ScenarioOrderList(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", spacing=10, padding=10, **kwargs)

        tab_bar = BoxLayout(size_hint_y=None, height=dp(40), spacing=10)

        self.tab_order = KToggleRoundedButton(
            text="ORDER LIST",
            group="scenario_tabs",
            
            radius=10,
            font_size=20,
        )
        self.tab_grid = KToggleRoundedButton(
            text="GRID VIEW",
            group="scenario_tabs",
            radius=10,
            font_size=20,
        )
        self.tab_order.bind(on_release=lambda *_: self.show_order_view())
        self.tab_grid.bind(on_release=lambda *_: self.show_grid_view())

        tab_bar.add_widget(self.tab_order)
        tab_bar.add_widget(self.tab_grid)
        self.add_widget(tab_bar)

        # ===========================
        #  메인 화면 컨테이너
        # ===========================
        self.main_area = BoxLayout()
        self.add_widget(self.main_area)

        # 두 화면 구성
        self.order_screen = self._build_order_screen()
        self.grid_screen = self._build_grid_screen()

        # 초기 화면 → ORDER LIST만 보이기
        self.show_order_view()


    def _build_order_screen(self):
        layout = BoxLayout(orientation="vertical", spacing=10)

        # === 배경 ===
        with layout.canvas.before:
            Color(0.96, 0.97, 0.99, 1)
            self.order_bg = RoundedRectangle(pos=layout.pos, size=layout.size)
        layout.bind(pos=lambda *_: self._update_order_bg(layout),
                    size=lambda *_: self._update_order_bg(layout))

        # === 3개 컬럼 묶음 ===
        row = BoxLayout(orientation="horizontal", spacing=0, padding=0)

        
        
        row.add_widget(IDColumnWidget("ID1", side="left"))
        row.add_widget(IDColumnWidget("ID2", side="middle"))
        row.add_widget(IDColumnWidget("ID3", side="right"))

        layout.add_widget(row)
        return layout

    def _update_order_bg(self, layout):
        self.order_bg.pos = layout.pos
        self.order_bg.size = layout.size

    

    # ----------------------------------------------------
    # 화면 구성: 그리드 화면
    # ----------------------------------------------------
    def _build_grid_screen(self):
        layout = BoxLayout()

        with layout.canvas.before:
            Color(1, 1, 1, 1)
            self.bg = RoundedRectangle(radius=[dp(10)], pos=layout.pos, size=layout.size)
        layout.bind(pos=lambda *_: self._update_bg(layout),
                    size=lambda *_: self._update_bg(layout))

        layout.add_widget(Label(text="Grid Map Placeholder", color=(0, 0, 0, 1)))
        return layout


    # ----------------------------------------------------
    # 1) ORDER VIEW 화면 전환
    # ----------------------------------------------------
    def show_order_view(self):
        self.main_area.clear_widgets()
        self.main_area.add_widget(self.order_screen)

    # ----------------------------------------------------
    # 2) GRID VIEW 화면 전환
    # ----------------------------------------------------
    def show_grid_view(self):
        self.main_area.clear_widgets()
        self.main_area.add_widget(self.grid_screen)

    def _update_bg(self, instance):
        self.bg.pos = instance.pos
        self.bg.size = instance.size
