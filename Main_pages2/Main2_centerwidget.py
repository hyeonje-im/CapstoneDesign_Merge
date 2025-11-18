
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock
from kivy.uix.anchorlayout import AnchorLayout

from Utilities.UI_utilities import KLine, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from Main_pages2.Main2_grid import GridWidget   

class GroupBox(BoxLayout):
    def __init__(self, title="", mode="row", **kwargs):
        super().__init__(orientation="vertical", padding=0, spacing=0, **kwargs)

        # 전체 배경
        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)

        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = KLine(self)

        self.bind(pos=self._update_bg, size=self._update_bg)

        # ================= 1) 타이틀 영역 =================
        self.title_area = AnchorLayout(
            size_hint_y=0.2,
            anchor_x="center",
            anchor_y="center",
        )

        title_label = KLabel(
            text=title,
            font_size=15,
            color=(1,1,1,1),
            size_hint=(1,1),
            halign="center",
            valign="middle",
        )
        self.title_area.add_widget(title_label)
        self.add_widget(self.title_area)

        # ===== 구분선 =====
        self.separator = BoxLayout(size_hint_y=None, height=1)
        with self.separator.canvas:
            Color(0, 0, 0, 1)
            self.sep_line = Rectangle(pos=self.separator.pos, size=self.separator.size)
        self.separator.bind(pos=self._update_sep, size=self._update_sep)
        self.add_widget(self.separator)

        # ================= 2) 버튼 영역 =================
        self.button_area = BoxLayout(
            size_hint_y=0.8,
            padding=10,
            spacing=10
        )
        self.add_widget(self.button_area)

        # 버튼 레이아웃 선택
        if mode == "row":
            self.button_layout = BoxLayout(
                orientation="horizontal",
                spacing=10,
                size_hint=(1,None)
            )
        elif mode == "grid":
            self.button_layout = GridLayout(
                cols=2,
                spacing=10,
                padding=0,
                size_hint=(1,None)
            )
        else:
            raise ValueError("mode must be 'row' or 'grid'")

        anchor = AnchorLayout(anchor_x="center", anchor_y="center")
        anchor.add_widget(self.button_layout)

        self.button_area.add_widget(anchor)

    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def _update_sep(self, *args):
        self.sep_line.pos = self.separator.pos
        self.sep_line.size = self.separator.size

# =================== CenterWidget ===================
class SingleControl(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", size_hint_x=0.45, **kwargs)

        self.selected_robot_id = None
        self.current_scenario_mode = "test"

        # ===== 배경 =====
        with self.canvas.before:
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)

        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = KLine(self)

        self.bind(pos=self._update_bg, size=self._update_bg)

        # ===== 상단 (GridView 영역) =====
        upper = BoxLayout(size_hint_y=0.6)
        grid_holder = AnchorLayout(anchor_x='center', anchor_y='center')

        # OpenCV 이미지 대신 Kivy Widget으로 변경
        self.grid_view = GridWidget(size_hint=(0.9, 0.9))
        grid_holder.add_widget(self.grid_view)

        upper.add_widget(grid_holder)

        # ===== 하단 버튼 =====
        lower = BoxLayout(orientation="vertical", spacing=5, size_hint_y=0.4)

        row1 = BoxLayout(orientation="horizontal", spacing=5)
        row2 = BoxLayout(orientation="horizontal", spacing=5)
        
        # 로봇 선택
        robot_group = GroupBox(title="로봇 선택", mode = "row")
        for i in range(1, 5):
            btn = KButton(text=f"D{i}", size_hint = (1,1))
            btn.bind(on_press=lambda inst, rid=i: self.select_robot(rid))
            robot_group.button_layout.add_widget(btn)
        row1.add_widget(robot_group)

        # 정렬
        align_group = GroupBox(title="정렬", mode = "row")
        for text, cmd in [
            ("중앙 정렬", "center_align"),
            ("방향 정렬", "direction_align"),
        ]:
            btn = KButton(text=text, size_hint = (1,1))
            btn.bind(on_press=lambda inst, c=cmd: post(cmd=c))
            align_group.button_layout.add_widget(btn)
        row1.add_widget(align_group)

        # 보드 제어
        board_group = GroupBox(title="보드 제어", mode = "grid")
        for text, cmd in [
            ("보드 고정", "lock_board"),
            ("보드 해제", "unlock_board"),
            ("ROI 재선택", "start_roi_selection"),
            ("시각화 ON/OFF", "toggle_visualization"),
        ]:
            btn = KButton(text=text, size_hint=(1,1))
            btn.bind(on_press=lambda inst, c=cmd: post(cmd=c))
            board_group.button_layout.add_widget(btn)
        row2.add_widget(board_group)

        

        # CBS 제어
        cbs_group = GroupBox(title="CBS 제어", mode = 'grid')
        for text, cmd in [
            ("경로탐색", "compute_cbs"),
            ("정지", "pause"),
            ("재개", "resume"),
            ("즉시정지", "immediate_stop"),
        ]:
            btn = KButton(text=text, size_hint = (1,1))
            btn.bind(on_press=lambda inst, c=cmd: post(cmd=c))
            cbs_group.button_layout.add_widget(btn)
        row2.add_widget(cbs_group)

        # 하단 구성
        lower.add_widget(row1)
        lower.add_widget(row2)

        self.add_widget(upper)
        self.add_widget(lower)

        Clock.schedule_interval(self.update_grid_from_backend, 0.1)
        Clock.schedule_interval(self.sync_scenario_mode, 0.5)
    # ===== 로봇 선택 =====
    def select_robot(self, rid):
        self.selected_robot_id = rid
        post("select_robot", rid=rid)
        print(f"[UI] robot {rid} selected.")

    # ===== Visual/Border 업데이트 =====
    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    # ===== GridWidget 백엔드 연동 =====
    def update_grid_from_backend(self, dt):
        """
        FrameBus → GridWidget 실시간 업데이트
        """
        grid = FrameBus.get_grid_state()
        agents = FrameBus.get_agent_states()
        paths = FrameBus.get_paths()
        homes = FrameBus.get_home_positions()

        self.grid_view.update_backend_state(
            grid_state=grid,
            agent_states=agents,
            paths=paths,
            home_positions=homes,
        )

    # ===== 시나리오 모드 동기화 =====
    def sync_scenario_mode(self, dt):
        mode = FrameBus.get_mode()
        if mode and mode != self.current_scenario_mode:
            self.current_scenario_mode = mode
            print(f"[UI] Mode updated → {mode}")
