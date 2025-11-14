
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock
from kivy.uix.anchorlayout import AnchorLayout

from Utilities.UI_utilities import KLine, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from Main_pages2.Main2_grid import GridWidget   

# =================== GroupBox ===================
class GroupBox(BoxLayout):
    def __init__(self, title="", **kwargs):
        super().__init__(orientation="vertical", padding=5, spacing=5, **kwargs)

        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)

        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = KLine(self)

        self.bind(pos=self._update_box, size=self._update_box)

        label = KLabel(
            text=title, size_hint_y=None, height=22,
            halign="center", valign="middle", font_size=13, color=(1,1,1,1)
        )
        self.add_widget(label)

        self.content = BoxLayout(orientation="vertical", spacing=4)
        self.add_widget(self.content)

    def _update_box(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)


# =================== CenterWidget ===================
class SingleControl(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", size_hint_x=0.4, **kwargs)

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
        self.grid_view = GridWidget(size_hint=(1, 1))
        grid_holder.add_widget(self.grid_view)

        upper.add_widget(grid_holder)

        # ===== 하단 버튼 =====
        lower = GridLayout(cols=2, rows=2, spacing=5, size_hint_y=0.4)

        # 로봇 선택
        robot_group = GroupBox(title="로봇 선택")
        for i in range(1, 5):
            btn = KButton(text=f"D{i}")
            btn.bind(on_press=lambda inst, rid=i: self.select_robot(rid))
            robot_group.content.add_widget(btn)
        lower.add_widget(robot_group)

        # 보드 제어
        board_group = GroupBox(title="보드 제어")
        for text, cmd in [
            ("보드 고정", "lock_board"),
            ("보드 해제", "unlock_board"),
            ("ROI 재선택", "start_roi_selection"),
            ("시각화 ON/OFF", "toggle_visualization"),
        ]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(cmd=c))
            board_group.content.add_widget(btn)
        lower.add_widget(board_group)

        # 정렬
        align_group = GroupBox(title="정렬")
        for text, cmd in [
            ("중앙 정렬", "center_align"),
            ("방향 정렬", "direction_align"),
        ]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(cmd=c))
            align_group.content.add_widget(btn)
        lower.add_widget(align_group)

        # CBS 제어
        cbs_group = GroupBox(title="CBS 제어")
        for text, cmd in [
            ("경로탐색", "compute_cbs"),
            ("정지", "pause"),
            ("재개", "resume"),
            ("즉시정지", "immediate_stop"),
        ]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(cmd=c))
            cbs_group.content.add_widget(btn)
        lower.add_widget(cbs_group)

        # === 최종 배치 ===
        self.add_widget(upper)
        self.add_widget(lower)

        # === 백엔드 → 그리드 실시간 동기화 ===
        Clock.schedule_interval(self.update_grid_from_backend, 0.1)

        # === 모드 동기화 ===
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
