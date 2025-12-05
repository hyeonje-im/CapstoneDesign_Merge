
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock
from kivy.uix.anchorlayout import AnchorLayout

from Utilities.UI_utilities import KLine, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from Main_pages2.Scenario_orderlist import ScenarioOrderList
from Main_pages2.Main2_grid import GridWidget

# GroupBox
class GroupBox(BoxLayout):
    def __init__(self, title="", **kwargs):
        super().__init__(orientation="vertical", padding=5, spacing=5, **kwargs)

        self.current_scenario_mode = "Idle"
        self.current_solver = "CBS"

        # 배경
        with self.canvas.before:
            Color(0xF5/255, 0xF7/255, 0xFC/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)

        with self.canvas.after:
            Color(0xAB/255, 0xAB/255, 0xAB/255, 1)
            self.border = KLine(self)
        self.bind(pos=self._update_rect, size=self._update_rect)

        # 제목
        label = KLabel(
            text=title,
            size_hint_y=None,
            height=22,
            halign="center",
            valign="middle",
            font_size=15,
            color=(0,0,0,1)
        )
        self.add_widget(label)

        self.separator = BoxLayout(size_hint_y=None, height=1)
        with self.separator.canvas.before:
            Color(0xAB / 255, 0xAB / 255, 0xAB / 255, 1)
            self.sep_line = Rectangle(pos=self.separator.pos, size=self.separator.size)
        self.separator.bind(pos=self._update_sep, size=self._update_sep)
        self.add_widget(self.separator)

        
        self.content = GridLayout(
            cols=2,
            spacing=10,
            padding=10,
            size_hint_y=1
        )
        self.add_widget(self.content)

    def _update_rect(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)


    def _update_sep(self, *args):
        self.sep_line.pos = self.separator.pos
        self.sep_line.size = self.separator.size


class ScenarioCenterWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", size_hint_x=0.5, **kwargs)

        self.current_scenario_mode = "Idle"
        self.current_solver = "CBS"

        # 배경
        with self.canvas.before:
            Color(0xFC/255, 0xFC/255, 0xFC/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        with self.canvas.after:
            Color(0xAB/255, 0xAB/255, 0xAB/255, 1)
            self.border = KLine(self)
        self.bind(pos=self._update_bg, size=self._update_bg)

        # ================================
        # 🔥 1) 상단 탭(전환 버튼) 만들기
        # ================================
        tab_bar = BoxLayout(size_hint_y=None, height=40, spacing=10, padding=10)

        self.btn_grid = KButton(text="GRID VIEW")
        self.btn_order = KButton(text="ORDER LIST")

        self.btn_grid.bind(on_press=lambda *_: self.show_grid())
        self.btn_order.bind(on_press=lambda *_: self.show_order())

        tab_bar.add_widget(self.btn_grid)
        tab_bar.add_widget(self.btn_order)

        # ================================
        # 🔥 2) 실제 화면이 들어갈 main_area
        # ================================
        self.main_area = BoxLayout(size_hint_y=0.75, spacing=10, padding=10)

        # Grid 화면
        grid_holder = AnchorLayout(anchor_x='center', anchor_y='center')

        self.grid_view = GridWidget(
            grid_json_path="OpenCV/grid/0926grid.json",
            size_hint=(0.95, 0.95)
        )

        grid_holder.add_widget(self.grid_view)
        self.grid_screen = grid_holder

        # Order List 화면
        self.order_screen = ScenarioOrderList(size_hint=(1,1))

        # 처음은 GRID 화면
        self.show_grid()

        # ================================
        # 🔥 3) 하단 제어 버튼
        # ================================
        lower = GridLayout(rows=2, cols=2, size_hint_y=0.25, spacing=10)

        # 시나리오 제어
        scenario_group = GroupBox(title="시나리오 제어")

        btn_run     = KButton(text="시나리오 실행")
        btn_stop    = KButton(text="시나리오 정지")
        btn_reset   = KButton(text="초기화")
        self.btn_mode = KButton(text=f"모드: {self.current_scenario_mode}")

        btn_run.bind(on_press=lambda b: post("scenario_run"))
        btn_stop.bind(on_press=lambda b: post("scenario_stop"))
        btn_reset.bind(on_press=lambda b: post("scenario_reset"))
        self.btn_mode.bind(on_press=self.toggle_mode)

        for b in [btn_run, btn_stop, btn_reset, self.btn_mode]:
            scenario_group.content.add_widget(b)

        lower.add_widget(scenario_group)

        # MAPF 제어
        mapf_group = GroupBox(title="MAPF 제어")

        btn_path   = KButton(text="경로 탐색")
        btn_reset  = KButton(text="경로 리셋")
        btn_dis    = KButton(text="Disjoint(p)")

        btn_path.bind(on_press=lambda b: post("compute_cbs"))
        btn_reset.bind(on_press=lambda b: post("reset_paths"))
        btn_dis.bind(on_press=lambda b: post("toggle_disjoint"))

        solver_prev = KButton(text="<")
        solver_next = KButton(text=">")
        self.lbl_solver = KLabel(text=self.current_solver, font_size=14)

        solver_prev.bind(on_press=lambda b: self.change_solver("prev"))
        solver_next.bind(on_press=lambda b: self.change_solver("next"))

        solver_box = BoxLayout(orientation="horizontal", spacing=5)
        solver_box.add_widget(solver_prev)
        solver_box.add_widget(self.lbl_solver)
        solver_box.add_widget(solver_next)

        for w in [btn_path, btn_reset, solver_box, btn_dis]:
            mapf_group.content.add_widget(w)

        lower.add_widget(mapf_group)

        # 전체 구성
        self.add_widget(tab_bar)
        self.add_widget(self.main_area)
        self.add_widget(lower)

        # 백엔드 루프
        Clock.schedule_interval(self.update_from_backend, 0.1)
        Clock.schedule_interval(self.update_grid_from_backend, 0.1)


    # ==========================
    # 🔥 화면 전환 기능
    # ==========================
    def show_grid(self):
        self.main_area.clear_widgets()
        self.main_area.add_widget(self.grid_screen)

        self.btn_grid.color = (0,0,0,1)
        self.btn_order.color = (0.5,0.5,0.5,1)

    def show_order(self):
        self.main_area.clear_widgets()
        self.main_area.add_widget(self.order_screen)

        self.btn_order.color = (0,0,0,1)
        self.btn_grid.color = (0.5,0.5,0.5,1)


    # ==========================
    # GridView 업데이트
    # ==========================
    def update_grid_from_backend(self, dt):
        grid = FrameBus.get_grid_state()
        agents = FrameBus.get_agent_states()
        goals = FrameBus.get_goal_positions()
        homes = FrameBus.get_home_positions()
        paths = FrameBus.get_paths()
        heads = FrameBus.get_headings()

        self.grid_view.update_backend_state(
            grid_state=grid,
            agent_states=agents,
            goal_positions=goals,
            home_positions=homes,
            paths=paths,
            agent_headings=heads
        )

    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
    
    def update_from_backend(self, dt):
        # 시나리오 모드 상태 가져오기
        backend_mode = FrameBus.get_mode()
        if backend_mode and backend_mode != self.current_scenario_mode:
            self.current_scenario_mode = backend_mode
            # 모드 버튼 갱신
            if hasattr(self, "btn_mode"):
                self.btn_mode.text = f"모드: {backend_mode}"

        # Solver 타입 가져오기
        solver = FrameBus.get_solver_type()
        if solver and solver != self.current_solver:
            self.current_solver = solver
            if hasattr(self, "lbl_solver"):
                self.lbl_solver.text = solver

    # -----------------------------
    # 시나리오 모드 토글
    # -----------------------------
    def toggle_mode(self, *args):
        post("toggle_scenario_mode")

    def change_solver(self, direction):
        solvers = ["CBS", "ICBS_CB", "ICBS"]
        idx = solvers.index(self.current_scenario_mode)

        if direction == "next":
            idx = (idx + 1) % len(solvers)
        else:
            idx = (idx - 1) % len(solvers)

        self.current_solver = solvers[idx]
        self.lbl_solver.text = self.current_solver
        post(f"solver_{direction}")

    
    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

   
    def toggle_mode(self, *args):
        post("toggle_scenario_mode")



    



    