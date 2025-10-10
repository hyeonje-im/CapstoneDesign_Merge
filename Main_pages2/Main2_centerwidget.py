from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.uix.widget import Widget
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.label import Label
from Utilities.UI_utilities import KLine, make_darkcell, make_brightcell, KButton, KLabel
from OpenCV.code.ui_bridge import FrameBus, post
from OpenCV.code.config import grid_row, grid_col

import numpy as np


# =================== GroupBox (공통 UI 컴포넌트) ===================
class GroupBox(BoxLayout):
    def __init__(self, title="", **kwargs):
        super().__init__(orientation="vertical", padding=5, spacing=5,  **kwargs)
        

        # 배경 + 라운드 테두리
        with self.canvas.before:
            Color(0x25/255, 0x28/255, 0x3B/255, 1)  # 배경색 다크셀과 동일
            self.bg = RoundedRectangle(pos=self.pos, size=self.size, radius=[5])
        with self.canvas.after:
            Color(0, 0, 0, 1)  # 검정 테두리
            self.border = Line(rounded_rectangle=(self.x, self.y, self.width, self.height, 5), width=1)

        self.bind(pos=self._update_rect, size=self._update_rect)

        # 타이틀
        title_label = KLabel(
            text=title, size_hint_y=None, height=20,
            halign="center", valign="middle", font_size=13, color=(1, 1, 1, 1)
        )
        self.add_widget(title_label)

        # 실제 콘텐츠 (버튼 행이 들어갈 컨테이너)
        self.content = BoxLayout(size_hint_y=1, spacing=4)
        self.add_widget(self.content)

    def _update_rect(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rounded_rectangle = (self.x, self.y, self.width, self.height, 5)


# =================== GridTextureView ===================
class GridTextureView(Image):
    """FrameBus.get_grid() (BGR ndarray)를 읽어 전체 영역에 그리드 텍스처 표시 + 클릭 이벤트 처리"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.allow_stretch = True
        self.keep_ratio = True
        self.center_widget = None
        Clock.schedule_interval(self._update, 1/20)  # 20fps

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

        # keep_ratio 적용된 그려지는 영역
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
            print("⚠️ 로봇 선택이 필요합니다. (우측 D1~D4 버튼 클릭)")
        return True


# =================== CenterWidget ===================
class CenterWidget(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(orientation='vertical', size_hint_x=0.25, **kwargs)
        self.selected_robot_id = None

        # 배경/테두리
        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.border = KLine(self)
            Color(0x2E/255, 0x33/255, 0x49/255, 1)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_bg_and_border, size=self.update_bg_and_border)

        # ================= 상단 (0.35) =================
        upper_section = BoxLayout(orientation='vertical', size_hint_y=0.45, spacing=5)
        upper_section.add_widget(make_brightcell("총 로봇 대수"))
        self.robot_count_cell = make_brightcell("0")
        upper_section.add_widget(self.robot_count_cell)
        upper_section.add_widget(Widget(size_hint_y=None, height=10))

        # Grid + 버튼 나란히 배치
        grid = BoxLayout(orientation="horizontal", size_hint_y=0.8, spacing=5)

        # Grid는 상단 영역에서 0.8 비율만 차지
        self.grid_view = GridTextureView(size_hint=(0.9, 0.9))
        self.grid_view.center_widget = self
        grid_container = AnchorLayout(anchor_x='center', anchor_y='center', size_hint=(0.8, 1))
        grid_container.add_widget(self.grid_view)

        grid.add_widget(grid_container)
        upper_section.add_widget(grid)

        # 더미 공간 (0.05)
        upper_section.add_widget(Widget(size_hint_y=0.05))

        # ================= 하단 (0.65) =================
        lower_section = BoxLayout(orientation='vertical', size_hint_y=0.5, spacing=5)

        # GroupBoxes

        robot_group = GroupBox(title="로봇 선택", size_hint_y=1/6)
        for i in range(1, 5):
            btn = KButton(text=f"D{i}")
            btn.bind(on_press=lambda inst, rid=i: self.select_robot(rid))
            robot_group.content.add_widget(btn)
        lower_section.add_widget(robot_group)


        board_group = GroupBox(title="보드 제어",size_hint_y=1/6)
        for text, cmd in [("보드 고정", "lock_board"), ("보드 해제", "unlock_board"),
                          ("보드 재선택(ROI)", "start_roi_selection"), ("시각화 토글", "toggle_visualization")]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            board_group.content.add_widget(btn)
        lower_section.add_widget(board_group)

        align_group = GroupBox(title="정렬",size_hint_y=1/6)
        for text, cmd in [("정렬(센터)", "center_align"), ("정렬(방향)", "direction_align")]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            align_group.content.add_widget(btn)
        lower_section.add_widget(align_group)

        cbs_group = GroupBox(title="CBS 제어",size_hint_y=1/6)
        for text, cmd in [("경로탐색", "compute_cbs"), ("정지", "pause"),
                          ("재개", "resume"), ("즉시정지", "immediate_stop")]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            cbs_group.content.add_widget(btn)
        lower_section.add_widget(cbs_group)

        grid_group = GroupBox(title="Grid 관리",size_hint_y=1/6)
        for text, cmd in [("Grid 저장", "save_grid"), ("Reset All", "reset_all")]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            grid_group.content.add_widget(btn)
        lower_section.add_widget(grid_group)

        toggle_group = GroupBox(title="모드 전환",size_hint_y=1/6)
        for text, cmd in [("수동 모드", "manual_toggle"), ("GoalAlign", "goalalign_toggle")]:
            btn = KButton(text=text)
            btn.bind(on_press=lambda inst, c=cmd: post(c))
            toggle_group.content.add_widget(btn)
        lower_section.add_widget(toggle_group)

        quit_group = GroupBox(title="프로그램 종료",size_hint_y=1/6)
        btn_quit = KButton(text="종료")
        btn_quit.bind(on_press=lambda inst: post("quit"))
        quit_group.content.add_widget(btn_quit)
        lower_section.add_widget(quit_group)

        # ================= 최종 배치 =================
        self.add_widget(upper_section)
        self.add_widget(lower_section)

    # =================== 유틸 메서드 ===================
    def select_robot(self, rid):
        self.selected_robot_id = rid
        post("select_robot", rid=rid)
        print(f"[UI] 로봇 {rid} 선택됨")

    def update_bg_and_border(self, *args):
        self.bg.pos, self.bg.size = self.pos, self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def update_grid_size(self, *args):
        parent_w = self.width
        side = parent_w * 0.7
        self.grid_view.size = (side, side)
