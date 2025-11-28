from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock
from kivy.uix.anchorlayout import AnchorLayout

from Utilities.UI_utilities import KLine, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from Main_pages2.Main2_grid import GridWidget
from Main_pages2.Scenario_orderlist import ScenarioOrderList

# ======================= GroupBox (새로운 방식) =======================
class GroupBox(BoxLayout):
    def __init__(self, title="", **kwargs):
        super().__init__(orientation="vertical", padding=5, spacing=5, **kwargs)

        # 배경
        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)

        with self.canvas.after:
            Color(0,0,0,1)
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
            color=(1,1,1,1)
        )
        self.add_widget(label)

        self.separator = BoxLayout(size_hint_y=None, height=1)
        with self.separator.canvas.before:
            Color(0, 0, 0, 1)
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


# ======================= ScenarioCenterWidget (새로운 방식) =======================
class ScenarioCenterWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", size_hint_x=0.5, **kwargs)

        self.current_scenario_mode = "Idle"
        self.current_solver = "CBS"

        # 배경
        with self.canvas.before:
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        with self.canvas.after:
            Color(0,0,0,1)
            self.border = KLine(self)
        self.bind(pos=self._update_bg, size=self._update_bg)

        # ================= 상단 구역 =================
        upper = BoxLayout(orientation="horizontal",
                            size_hint_y=0.75,
                            spacing=10,
                            padding=10)

        # ------ 왼쪽: 주문 리스트 -------
        self.order_list = ScenarioOrderList(size_hint_x=1)
        upper.add_widget(self.order_list)

        

        # ================= 하단 버튼 그룹 (2×2 그리드) =================
        lower = GridLayout(rows=2, cols=2, size_hint_y=0.25, spacing=10)

        # -------- 3. 시나리오 제어 --------
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

        # -------- 4. MAPF 제어 --------
        mapf_group = GroupBox(title="MAPF 제어")

        btn_path   = KButton(text="경로 탐색")
        btn_reset  = KButton(text="경로 리셋")
        btn_dis    = KButton(text="Disjoint(p)")

        btn_path.bind(on_press=lambda b: post("compute_cbs"))
        btn_reset.bind(on_press=lambda b: post("reset_paths"))
        btn_dis.bind(on_press=lambda b: post("toggle_disjoint"))

        # Solver UI
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

        # 하단 전체 추가
        self.add_widget(upper)
        self.add_widget(lower)
        

        


    # ================= Helper Methods =================
    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    

    
    def toggle_mode(self, *args):
        self.current_scenario_mode = "Run" if self.current_scenario_mode == "Idle" else "Idle"
        self.btn_mode.text = f"모드: {self.current_scenario_mode}"
        post("toggle_scenario_mode")

    def change_solver(self, direction):
        solvers = ["CBS", "ICBS_CB", "ICBS"]
        idx = solvers.index(self.current_solver)

        if direction == "next":
            idx = (idx + 1) % len(solvers)
        else:
            idx = (idx - 1) % len(solvers)

        self.current_solver = solvers[idx]
        self.lbl_solver.text = self.current_solver
        post(f"solver_{direction}")
