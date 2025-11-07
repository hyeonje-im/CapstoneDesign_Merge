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

        