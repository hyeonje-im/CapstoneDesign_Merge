from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock
from kivy.uix.anchorlayout import AnchorLayout

from Utilities.UI_utilities import KLine, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from Main_pages2.Main2_grid import GridWidget   # ← 핵심

# =================== GroupBox ===================
class GroupBox(BoxLayout):
    def __init__(self, title="", **kwargs):
        super().__init__(orientation="vertical", padding=5, spacing=5, **kwargs)
        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self._update_rect, size=self._update_rect)

        title_label = KLabel(
            text=title, size_hint_y=None, height=22,
            halign="center", valign="middle", font_size=13, color=(1,1,1,1)
        )
        self.add_widget(title_label)

        self.content = BoxLayout(orientation="vertical", spacing=3)
        self.add_widget(self.content)

    def _update_rect(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size


# =================== ScenarioCenterWidget ===================
class ScenarioCenterWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.4, **kwargs)

        self.current_scenario_mode = "Idle"
        self.current_solver = "CBS"

        # ===== 배경 =====
        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ===== 상단 GridView =====
        upper_section = BoxLayout(orientation='vertical', size_hint_y=0.4)
        grid_container = AnchorLayout(anchor_x='center', anchor_y='center')

        # ⭐ 핵심: OpenCV 기반 이미지 제거 → GridCanvasWidget 삽입
        self.grid_view = GridWidget(size_hint=(0.9, 0.9))
        self.grid_view.center_widget = self

        grid_container.add_widget(self.grid_view)
        upper_section.add_widget(grid_container)

        # ===== 하단 버튼 그룹 =====
        lower_section = GridLayout(rows=2, cols=2, size_hint_y=0.6, spacing=5)

        # ===== 1. 로봇 선택 =====
        robot_group = GroupBox(title="로봇 선택")
        for i in range(1, 5):
            btn = KButton(text=f"D{i}")
            btn.bind(on_press=lambda inst, rid=i: self.select_robot(rid))
            robot_group.content.add_widget(btn)
        lower_section.add_widget(robot_group)

        # ===== 2. 보드 제어 =====
        board_group = GroupBox(title="보드 제어")
        for name, cmd in [
            ("보드 고정", "lock_board"),
            ("보드 해제", "unlock_board"),
            ("ROI 재선택", "start_roi_selection"),
            ("시각화 토글", "toggle_visualization"),
        ]:
            btn = KButton(text=name)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            board_group.content.add_widget(btn)
        lower_section.add_widget(board_group)

        # ===== 3. 시나리오 제어 =====
        scenario_group = GroupBox(title="시나리오 제어")
        btn_run  = KButton(text="시나리오 실행")
        btn_stop = KButton(text="시나리오 정지")
        btn_reset = KButton(text="초기화")
        self.btn_mode = KButton(text=f"모드: {self.current_scenario_mode}")

        btn_run.bind(on_press=lambda inst: post("scenario_run"))
        btn_stop.bind(on_press=lambda inst: post("scenario_stop"))
        btn_reset.bind(on_press=lambda inst: post("scenario_reset"))
        self.btn_mode.bind(on_press=self.toggle_mode)

        for b in [btn_run, btn_stop, btn_reset, self.btn_mode]:
            scenario_group.content.add_widget(b)
        lower_section.add_widget(scenario_group)

        # ===== 4. MAPF 제어 =====
        mapf_group = GroupBox(title="MAPF 제어")

        btn_search = KButton(text="경로 탐색")
        btn_reset_path = KButton(text="경로 리셋")
        btn_disjoint = KButton(text="Disjoint(p)")

        btn_search.bind(on_press=lambda inst: post("compute_cbs"))
        btn_reset_path.bind(on_press=lambda inst: post("reset_paths"))
        btn_disjoint.bind(on_press=lambda inst: post("toggle_disjoint"))

        # Solver 선택 UI
        btn_solver_prev = KButton(text="<")
        btn_solver_next = KButton(text=">")
        self.lbl_solver = KLabel(text=self.current_solver, font_size=14, color=(1,1,1,1))

        btn_solver_prev.bind(on_press=lambda inst: self.change_solver("prev"))
        btn_solver_next.bind(on_press=lambda inst: self.change_solver("next"))

        solver_box = BoxLayout(orientation="horizontal", spacing=5)
        solver_box.add_widget(btn_solver_prev)
        solver_box.add_widget(self.lbl_solver)
        solver_box.add_widget(btn_solver_next)

        for w in [btn_search, btn_reset_path, solver_box, btn_disjoint]:
            mapf_group.content.add_widget(w)

        lower_section.add_widget(mapf_group)

        # ===== 최종 배치 =====
        self.add_widget(upper_section)
        self.add_widget(lower_section)

        # ⭐ 백엔드 → Kivy Grid 실시간 업데이트
        Clock.schedule_interval(self.update_grid_from_backend, 0.1)

    # ===== BACKEND → GRID 연결 =====
    def update_grid_from_backend(self, dt):
        grid = FrameBus.get_grid_state()
        agents = FrameBus.get_agent_states()
        paths = FrameBus.get_paths()
        homes = FrameBus.get_home_positions()

        self.grid_view.update_backend_state(
            grid_state=grid,
            agent_states=agents,
            paths=paths,
            home_positions=homes
        )

    # ===== GUI 보조 메서드 =====
    def update_bg_and_border(self, *args):
        self.bg.pos, self.bg.size = self.pos, self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def select_robot(self, rid):
        print(f"[Scenario UI] 로봇 {rid} 선택됨")
        post("select_robot", rid=rid)

    def toggle_mode(self, instance):
        new_mode = "Run" if self.current_scenario_mode == "Idle" else "Idle"
        self.current_scenario_mode = new_mode
        self.btn_mode.text = f"모드: {new_mode}"
        post("toggle_scenario_mode")

    def change_solver(self, direction):
        solvers = ["CBS", "ICBS_CB", "ICBS"]
        cur = solvers.index(self.current_solver)
        nxt = (cur + 1) % len(solvers) if direction == "next" else (cur - 1) % len(solvers)

        self.current_solver = solvers[nxt]
        self.lbl_solver.text = self.current_solver
        print(f"[Scenario UI] Solver 변경 → {self.current_solver}")
        post(f"solver_{direction}")
