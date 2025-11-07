from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.uix.anchorlayout import AnchorLayout
from Utilities.UI_utilities import KLine, make_darkcell, make_brightcell, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from OpenCV.code.config import grid_row, grid_col
import numpy as np


# =================== GroupBox (공통 UI 컴포넌트) ===================
class GroupBox(BoxLayout):
    def __init__(self, title="", **kwargs):
        super().__init__(orientation="vertical", padding=5, spacing=5, **kwargs)

        # 배경 + 라운드 테두리
        with self.canvas.before:
            Color(0x25 / 255, 0x28 / 255, 0x3B / 255, 1)
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[5])
        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = Line(rounded_rectangle=(self.x, self.y, self.width, self.height, 5), width=1)

        self.bind(pos=self._update_rect, size=self._update_rect)

        title_label = KLabel(
            text=title, size_hint_y=None, height=20,
            halign="center", valign="middle", font_size=13, color=(1, 1, 1, 1)
        )
        self.add_widget(title_label)

        self.content = BoxLayout(size_hint_y=1, spacing=4)
        self.add_widget(self.content)

    def _update_rect(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rounded_rectangle = (self.x, self.y, self.width, self.height, 5)


# =================== GridTextureView ===================
class GridTextureView(Image):
    """FrameBus.get_grid()로부터 BGR 이미지를 받아 Texture로 표시"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.allow_stretch = True
        self.keep_ratio = True
        self.center_widget = None
        Clock.schedule_interval(self._update, 1 / 20)

    def _update(self, dt):
        frame = FrameBus.get_grid()
        if frame is None:
            return
        rgb = frame[:, :, ::-1].copy()
        h, w = rgb.shape[:2]
        if not self.texture or self.texture.width != w or self.texture.height != h:
            self.texture = Texture.create(size=(w, h))
            self.texture.flip_vertical()
        self.texture.blit_buffer(rgb.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
        self.canvas.ask_update()

    def on_touch_down(self, touch):
        if not self.collide_point(*touch.pos):
            return False
        if not self.texture:
            return False

        img_w, img_h = float(self.texture.width), float(self.texture.height)
        W, H = float(self.width), float(self.height)
        s = min(W / img_w, H / img_h)
        draw_w, draw_h = img_w * s, img_h * s
        off_x, off_y = (W - draw_w) * 0.5, (H - draw_h) * 0.5

        lx, ly = touch.x - self.x - off_x, touch.y - self.y - off_y
        if lx < 0 or ly < 0 or lx > draw_w or ly > draw_h:
            return False

        u, v = lx / draw_w, ly / draw_h
        v_top = 1.0 - v
        col = max(0, min(grid_col - 1, int(u * grid_col)))
        row = max(0, min(grid_row - 1, int(v_top * grid_row)))

        if self.center_widget and self.center_widget.selected_robot_id is not None:
            post("set_goal", rid=self.center_widget.selected_robot_id, row=row, col=col)
            print(f"[UI] Grid 클릭: row={row}, col={col}, robot={self.center_widget.selected_robot_id}")
        else:
            print("⚠️ 로봇 선택이 필요합니다. (D1~D4 버튼 클릭)")
        return True


# =================== CenterWidget ===================
class CenterWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.4, **kwargs)
        self.selected_robot_id = None
        self.current_scenario_mode = "test"   # 초기 모드
        self.current_solver = "CBS"           # solver 기본값

        # ===== 배경/테두리 =====
        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E / 255, 0x33 / 255, 0x49 / 255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ===== 상단 (GridView) =====
        upper_section = BoxLayout(orientation='vertical', size_hint_y=0.4, spacing=5)
        grid_container = AnchorLayout(anchor_x='center', anchor_y='center', size_hint=(1, 1))
        self.grid_view = GridTextureView(size_hint=(0.9, 0.9), pos_hint={'center_x': 0.5, 'center_y': 0.5})
        self.grid_view.center_widget = self
        grid_container.add_widget(self.grid_view)
        upper_section.add_widget(grid_container)

        # ===== 하단 (버튼 그룹들) =====
        lower_section = BoxLayout(orientation='vertical', size_hint_y=0.6, spacing=5)

        # 로봇 선택
        robot_group = GroupBox(title="로봇 선택", size_hint_y=1/6)
        for i in range(1, 5):
            btn = KButton(text=f"D{i}")
            btn.bind(on_press=lambda inst, rid=i: self.select_robot(rid))
            robot_group.content.add_widget(btn)
        lower_section.add_widget(robot_group)

        # 보드 제어
        board_group = GroupBox(title="보드 제어", size_hint_y=1/6)
        for text, cmd in [
            ("보드 고정", "lock_board"),
            ("보드 해제", "unlock_board"),
            ("보드 재선택(ROI)", "start_roi_selection"),
            ("시각화 토글on/off", "toggle_visualization")
        ]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            board_group.content.add_widget(btn)
        lower_section.add_widget(board_group)

        # 정렬
        align_group = GroupBox(title="정렬", size_hint_y=1/6)
        for text, cmd in [
            ("중앙 정렬", "center_align"),
            ("방향 정렬", "direction_align")
        ]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            align_group.content.add_widget(btn)
        lower_section.add_widget(align_group)

        # CBS 제어
        cbs_group = GroupBox(title="CBS 제어", size_hint_y=1/6)
        for text, cmd in [
            ("경로탐색", "compute_cbs"),
            ("정지", "pause"),
            ("재개", "resume"),
            ("즉시정지", "immediate_stop")
        ]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            cbs_group.content.add_widget(btn)
        lower_section.add_widget(cbs_group)

        # Grid 관리
        grid_group = GroupBox(title="Grid 관리", size_hint_y=1/6)
        for text, cmd in [("Grid 저장", "save_grid"), ("Reset All", "reset_all")]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            grid_group.content.add_widget(btn)
        lower_section.add_widget(grid_group)

        # 시나리오 제어
        scenario_group = GroupBox(title="시나리오 제어", size_hint_y=1/6)
        self.btn_scenario_mode = KButton(text=f"Mode: {self.current_scenario_mode.capitalize()}")
        self.btn_scenario_mode.bind(on_press=self.toggle_scenario_mode)
        scenario_group.content.add_widget(self.btn_scenario_mode)

        btn_scenario_run = KButton(text="시나리오 실행/정지")
        btn_scenario_run.bind(on_press=lambda inst: post("toggle_scenario_run"))
        scenario_group.content.add_widget(btn_scenario_run)
        lower_section.add_widget(scenario_group)

        # MAPF 제어
        mapf_group = GroupBox(title="MAPF 제어", size_hint_y=1/6)
        btn_auto = KButton(text="로봇 초기화 및 경로탐색")
        btn_auto.bind(on_press=lambda inst: post("auto_release_align_cbs"))
        mapf_group.content.add_widget(btn_auto)

        solver_box = BoxLayout(orientation="horizontal", spacing=5)
        btn_prev = KButton(text="<")
        btn_next = KButton(text=">")
        self.lbl_solver = KLabel(text=self.current_solver, font_size=14, color=(1, 1, 1, 1))
        btn_prev.bind(on_press=lambda inst: self.change_solver("prev"))
        btn_next.bind(on_press=lambda inst: self.change_solver("next"))
        solver_box.add_widget(btn_prev)
        solver_box.add_widget(self.lbl_solver)
        solver_box.add_widget(btn_next)
        mapf_group.content.add_widget(solver_box)

        btn_disjoint = KButton(text="Disjoint 토글 (p)")
        btn_disjoint.bind(on_press=lambda inst: post("toggle_disjoint"))
        mapf_group.content.add_widget(btn_disjoint)
        lower_section.add_widget(mapf_group)

        # 종료
        quit_group = GroupBox(title="프로그램 종료", size_hint_y=1/6)
        btn_quit = KButton(text="종료")
        btn_quit.bind(on_press=lambda inst: post("quit"))
        quit_group.content.add_widget(btn_quit)
        lower_section.add_widget(quit_group)

        # 최종 배치
        self.add_widget(upper_section)
        self.add_widget(lower_section)

        # --- 주기적으로 백엔드 모드 동기화 ---
        Clock.schedule_interval(self.sync_scenario_mode, 0.5)

    # ------------------------ 유틸 메서드 ------------------------
    def select_robot(self, rid):
        self.selected_robot_id = rid
        post("select_robot", rid=rid)
        print(f"[UI] 로봇 {rid} 선택됨")

    def update_bg_and_border(self, *args):
        self.bg.pos, self.bg.size = self.pos, self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    # --- 시나리오 모드 토글 ---
    def toggle_scenario_mode(self, instance):
        post("toggle_scenario_mode")
        print("[UI] 시나리오 모드 변경 요청 전송")

    # --- 백엔드 모드 동기화 (FrameBus 통해 반영) ---
    def sync_scenario_mode(self, dt):
        new_mode = FrameBus.get_mode()
        if new_mode and new_mode != self.current_scenario_mode:
            self.current_scenario_mode = new_mode
            self.btn_scenario_mode.text = f"Mode: {new_mode.capitalize()}"
            print(f"[UI] 모드 동기화됨 → {new_mode.capitalize()}")

    # --- Solver 변경 (< >) ---
    def change_solver(self, direction):
        solvers = ["CBS", "ICBS_CB", "ICBS"]
        cur_idx = solvers.index(self.current_solver)
        if direction == "next":
            next_idx = (cur_idx + 1) % len(solvers)
            post("solver_next")
        else:
            next_idx = (cur_idx - 1) % len(solvers)
            post("solver_prev")
        self.current_solver = solvers[next_idx]
        self.lbl_solver.text = self.current_solver
        print(f"[UI] Solver 변경 → {self.current_solver}")
