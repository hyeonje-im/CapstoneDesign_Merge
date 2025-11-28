from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.metrics import dp
from kivy.clock import Clock
from kivy.graphics import Color, RoundedRectangle
from OpenCV.code.ui_bridge import FrameBus
from Main_pages2.Main2_grid import GridWidget
from Utilities.UI_utilities import KToggleRoundedButton


# ====================================================
# 1) 개별 행 위젯: Num | Pos | Goal | Status
# ====================================================
class OrderRowWidget(BoxLayout):
    def __init__(self, num="#12", pos="[3,1]", goal="[3,4]", status="MOVING", **kwargs):
        super().__init__(orientation="horizontal", size_hint_y=None, height=dp(36), spacing=10, **kwargs)

        # Num
        self.add_widget(Label(text=num, size_hint_x=0.2,
                              halign="left", valign="middle", color=(0,0,0,1)))

        # Position
        self.add_widget(Label(text=pos, size_hint_x=0.25,
                              halign="left", valign="middle", color=(0,0,0,1)))

        # Goal
        self.add_widget(Label(text=goal, size_hint_x=0.25,
                              halign="left", valign="middle", color=(0,0,0,1)))

        # Status 버튼
        st = Button(
            text=status,
            size_hint_x=0.3,
            background_color=self._status_color(status),
            background_normal="",
            color=(0,0,0,1)
        )
        self.add_widget(st)

    # ----------------------------------------------
    # 상태별 색상
    # ----------------------------------------------
    def _status_color(self, status):
        STATUS_COLOR = {
            "IDLE": (0.85, 0.85, 0.85, 1),
            "MOVING": (0.4, 0.7, 1, 1),
            "ARRIVED": (0.6, 1, 0.6, 1),
            "WAITING": (1, 0.6, 0.3, 1),
            "REPLANNING": (1, 0.85, 0.4, 1),
            "ALIGNING": (0.3, 1, 0.7, 1),
            "RETURNING": (0.6, 0.4, 1, 1),
            "AT_HOME": (0.9, 0.9, 1, 1),
        }
        return STATUS_COLOR.get(status, (0.9, 0.9, 0.9, 1))


# ====================================================
# 2) ID별 컬럼 (ID1 / ID2 / ID3)
# ====================================================
class IDColumnWidget(BoxLayout):
    def __init__(self, title="ID1", side="middle", **kwargs):
        super().__init__(orientation="vertical", spacing=10, padding=10, **kwargs)

        # === 라운딩 설정 ===
        if side == "left":
            radius = [dp(10), 0, 0, dp(10)]
        elif side == "right":
            radius = [0, dp(10), dp(10), 0]
        else:
            radius = [0, 0, 0, 0]

        # === 배경 ===
        with self.canvas.before:
            Color(0.96, 0.97, 0.99, 1)
            self.bg = RoundedRectangle(radius=radius, pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg, size=self.update_bg)

        # --- Title ---
        title_label = Label(
            text=title, size_hint_y=None, height=dp(30),
            bold=True, color=(0,0,0,1),
            halign="left", valign="middle",
        )
        title_label.bind(size=lambda inst, val: setattr(inst, 'text_size', val))
        self.add_widget(title_label)

        # --- Header ---
        header = GridLayout(cols=4, size_hint_y=None, height=dp(30))
        header.add_widget(Label(text="Num", bold=True, color=(0,0,0,1)))
        header.add_widget(Label(text="Pos", bold=True, color=(0,0,0,1)))
        header.add_widget(Label(text="Goal", bold=True, color=(0,0,0,1)))
        header.add_widget(Label(text="Status", bold=True, color=(0,0,0,1)))
        self.add_widget(header)

        # --- 리스트 영역 ---
        self.scroll = ScrollView(size_hint=(1, 1))
        self.list_layout = GridLayout(cols=1, size_hint_y=None, spacing=5)
        self.list_layout.bind(minimum_height=self.list_layout.setter("height"))

        self.scroll.add_widget(self.list_layout)
        self.add_widget(self.scroll)

    def update_bg(self, *_):
        self.bg.pos = self.pos
        self.bg.size = self.size


# ====================================================
# 3) 전체 ORDER LIST 화면
# ====================================================
class ScenarioOrderList(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", spacing=10, padding=10, **kwargs)

        # -----------------------
        # 탭
        # -----------------------
        tab_bar = BoxLayout(size_hint_y=None, height=dp(40), spacing=10)

        self.tab_order = KToggleRoundedButton(
            text="ORDER LIST",
            group="scenario_tabs", radius=10,
            font_size=20
        )
        self.tab_grid = KToggleRoundedButton(
            text="GRID VIEW",
            group="scenario_tabs", radius=10,
            font_size=20
        )

        self.tab_order.bind(on_release=lambda *_: self.show_order_view())
        self.tab_grid.bind(on_release=lambda *_: self.show_grid_view())

        tab_bar.add_widget(self.tab_order)
        tab_bar.add_widget(self.tab_grid)
        self.add_widget(tab_bar)

        # -----------------------
        # 메인 영역
        # -----------------------
        self.main_area = BoxLayout()
        self.add_widget(self.main_area)

        # 화면 구성
        self.order_screen = self._build_order_screen()
        self.grid_screen = self._build_grid_screen()

        # 컬럼 저장용
        self.col_widgets = {}

        # UI 시작 화면
        self.show_order_view()

        # FrameBus 실시간 업데이트
        Clock.schedule_interval(self.update_from_framebus, 0.1)


    # ====================================================
    # ORDER LIST 화면
    # ====================================================
    def _build_order_screen(self):
        layout = BoxLayout(orientation="vertical", spacing=10)

        with layout.canvas.before:
            Color(0.96, 0.97, 0.99, 1)
            self.order_bg = RoundedRectangle(pos=layout.pos, size=layout.size)
        layout.bind(pos=lambda *_: self._update_order_bg(layout),
                    size=lambda *_: self._update_order_bg(layout))

        row = BoxLayout(orientation="horizontal", spacing=0)

        col1 = IDColumnWidget("ID1", side="left")
        col2 = IDColumnWidget("ID2", side="middle")
        col3 = IDColumnWidget("ID3", side="right")

        self.col_widgets = {1: col1, 2: col2, 3: col3}

        row.add_widget(col1)
        row.add_widget(col2)
        row.add_widget(col3)

        layout.add_widget(row)
        return layout

    def _update_order_bg(self, layout):
        self.order_bg.pos = layout.pos
        self.order_bg.size = layout.size

    # ====================================================
    # GRID 화면
    # ====================================================
    def _build_grid_screen(self):
        layout = BoxLayout()

        with layout.canvas.before:
            Color(1,1,1,1)
            self.bg = RoundedRectangle(radius=[dp(10)], pos=layout.pos, size=layout.size)
        layout.bind(pos=lambda *_: self._update_bg(layout),
                    size=lambda *_: self._update_bg(layout))

        layout.add_widget(Label(text="Grid Map Placeholder",
                                color=(0,0,0,1)))
        return layout

    # ====================================================
    # FrameBus → UI 연동
    # ====================================================
    def update_from_framebus(self, dt):
        ui_state = FrameBus.get_robot_ui_state()
        if not ui_state:
            return
        
        # ID1 / ID2 / ID3
        for rid, col in self.col_widgets.items():
            col.list_layout.clear_widgets()

            robot = ui_state.get(rid)
            if not robot:
                continue

            num = str(robot.get("num", "-"))
            pos = str(robot.get("pos", "-"))
            goal = str(robot.get("goal", "-"))
            status = robot.get("status", "IDLE")

            row = OrderRowWidget(num=num, pos=pos, goal=goal, status=status)
            col.list_layout.add_widget(row)

    # -----------------------
    # 화면 전환
    # -----------------------
    def show_order_view(self):
        self.main_area.clear_widgets()
        self.main_area.add_widget(self.order_screen)

    def show_grid_view(self):
        self.main_area.clear_widgets()
        self.main_area.add_widget(self.grid_screen)

    def _update_bg(self, instance):
        self.bg.pos = instance.pos
        self.bg.size = instance.size
