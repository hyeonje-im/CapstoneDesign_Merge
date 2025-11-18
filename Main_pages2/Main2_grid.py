from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, Ellipse, Triangle
from kivy.core.text import Label as CoreLabel
from OpenCV.code.config import grid_row, grid_col
from OpenCV.code.ui_bridge import post
import math

# -------------------------------------
# ID COLOR
# -------------------------------------
_ID_COLORS = [
    (0.90, 0.30, 0.30, 1),
    (0.30, 0.70, 0.30, 1),
    (0.30, 0.40, 0.90, 1),
    (0.90, 0.70, 0.30, 1),
    (0.70, 0.30, 0.90, 1),
    (0.30, 0.80, 0.80, 1),
]

def color_for_id(rid: int):
    return _ID_COLORS[int(rid) % len(_ID_COLORS)]


# ==========================================================
#                        GRID WIDGET
# ==========================================================
class GridWidget(Widget):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.rows = grid_row
        self.cols = grid_col

        # FrameBus → UI 동기화 값
        self.grid_state = None
        self.agent_states = {}
        self.paths = {}
        self.home_positions = {}
        self.goal_positions = {}
        self.delays = {}
        self.headings = {}

        self.bind(pos=self._redraw, size=self._redraw)

    # ==========================================================
    #               백엔드 → UI 값 업데이트
    # ==========================================================
    def update_backend_state(
        self,
        grid_state=None,
        agent_states=None,
        paths=None,
        home_positions=None,
        goal_positions=None,
        delays=None,
        headings=None,
    ):
        if grid_state is not None:
            self.grid_state = grid_state
        if agent_states is not None:
            self.agent_states = dict(agent_states)
        if paths is not None:
            self.paths = dict(paths)
        if home_positions is not None:
            self.home_positions = dict(home_positions)
        if goal_positions is not None:
            self.goal_positions = dict(goal_positions)
        if delays is not None:
            self.delays = dict(delays)
        if headings is not None:
            self.headings = dict(headings)

        self._redraw()

    # ==========================================================
    #                클릭 → 도착지 설정
    # ==========================================================
    def on_touch_down(self, touch):
        if not self.collide_point(*touch.pos):
            return super().on_touch_down(touch)

        side = min(self.width, self.height)
        real_x = self.x + (self.width - side) / 2
        real_y = self.y + (self.height - side) / 2

        cell_w = side / self.cols
        cell_h = side / self.rows

        c = int((touch.x - real_x) / cell_w)
        r = int((touch.y - real_y) / cell_h)

        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return super().on_touch_down(touch)

        # SingleControl에서 선택된 로봇 id 얻기
        root = self.parent.parent
        rid = getattr(root, "selected_robot_id", None)

        if rid:
            rid = int(rid)
            print(f"[UI] set goal → robot {rid} → ({r},{c})")

            # UI 로컬 즉시 반영
            self.goal_positions[rid] = (r, c)
            self._redraw()

            # 백엔드 전달
            post("set_goal", rid=rid, row=r, col=c)

        return super().on_touch_down(touch)

    # ==========================================================
    #                      DRAW
    # ==========================================================
    def _redraw(self, *args):

        self.canvas.clear()

        if self.rows == 0 or self.cols == 0:
            return

        # 정사각형 유지
        side = min(self.width, self.height)
        real_x = self.x + (self.width - side) / 2
        real_y = self.y + (self.height - side) / 2
        cell_w = side / self.cols
        cell_h = side / self.rows

        with self.canvas:

            # =====================================
            # 1. 전체 배경
            # =====================================
            Color(0.96, 0.97, 0.99, 1)
            Rectangle(pos=(real_x, real_y), size=(side, side))

            # =====================================
            # 2. 장애물 + 출발지 + 도착지(원)
            # =====================================
            for r in range(self.rows):
                for c in range(self.cols):

                    x = real_x + c * cell_w
                    y = real_y + r * cell_h

                    # 장애물
                    if self.grid_state is not None:
                        try:
                            if self.grid_state[r][c] == 1:
                                Color(0.1, 0.1, 0.1, 1)
                                Rectangle(pos=(x, y), size=(cell_w, cell_h))
                        except:
                            pass

                    # 출발지(셀 전체 연하게)
                    for rid, (hr, hc) in self.home_positions.items():
                        if hr == r and hc == c:
                            Color(0.75, 0.83, 1.0, 0.5)
                            Rectangle(pos=(x, y), size=(cell_w, cell_h))

                    # 도착지(셀 전체 칠하는 대신 “중앙 원”)
                    for rid, (gr, gc) in self.goal_positions.items():
                        if gr == r and gc == c:
                            cx = x + cell_w / 2
                            cy = y + cell_h / 2
                            radius = min(cell_w, cell_h) * 0.30
                            rcol = color_for_id(int(rid))
                            Color(rcol[0], rcol[1], rcol[2], 0.75)
                            Ellipse(pos=(cx - radius, cy - radius),
                                    size=(radius * 2, radius * 2))

            # =====================================
            # 3. GRID LINE
            # =====================================
            Color(0.80, 0.84, 0.90, 1)
            for i in range(self.rows + 1):
                y = real_y + i * cell_h
                Line(points=[real_x, y, real_x + side, y], width=1)

            for j in range(self.cols + 1):
                x = real_x + j * cell_w
                Line(points=[x, real_y, x, real_y + side], width=1)

            # =====================================
            # 4. PATH 표시
            # =====================================
            for rid, path in self.paths.items():
                Color(*color_for_id(int(rid)), 0.22)
                for (pr, pc) in path:
                    px = real_x + pc * cell_w
                    py = real_y + pr * cell_h
                    Rectangle(pos=(px, py), size=(cell_w, cell_h))

            # =====================================
            # 5. 로봇 + 방향 삼각형
            # =====================================
            for rid, (r, c) in self.agent_states.items():
                cx = real_x + (c + 0.5) * cell_w
                cy = real_y + (r + 0.5) * cell_h
                radius = min(cell_w, cell_h) * 0.26

                # 로봇 원
                Color(*color_for_id(int(rid)))
                Ellipse(pos=(cx - radius, cy - radius), size=(radius * 2, radius * 2))

                # 방향(삼각형)
                heading_deg = self.headings.get(rid, None)
                if heading_deg is not None:
                    theta = math.radians(heading_deg)
                    tip_x = cx + radius * math.cos(theta)
                    tip_y = cy + radius * math.sin(theta)
                    left_x = cx + radius * 0.55 * math.cos(theta + 2.5)
                    left_y = cy + radius * 0.55 * math.sin(theta + 2.5)
                    right_x = cx + radius * 0.55 * math.cos(theta - 2.5)
                    right_y = cy + radius * 0.55 * math.sin(theta - 2.5)

                    Color(0, 0, 0, 0.9)
                    Triangle(points=[tip_x, tip_y, left_x, left_y, right_x, right_y])

            # =====================================
            # 6. S1 / G1 텍스트
            # =====================================
            for rid, (rr, cc) in self.home_positions.items():
                _draw_small_text(f"s{rid}", real_x, real_y, rr, cc, cell_w, cell_h)

            for rid, (rr, cc) in self.goal_positions.items():
                _draw_small_text(f"g{rid}", real_x, real_y, rr, cc, cell_w, cell_h)


# ----------------------------------------------------------
# 작은 텍스트
# ----------------------------------------------------------
def _draw_small_text(text, real_x, real_y, r, c, cell_w, cell_h):
    x = real_x + c * cell_w + 3
    y = real_y + r * cell_h + 3
    lbl = CoreLabel(text=text, font_size=cell_h * 0.25, color=(0, 0, 0, 1))
    lbl.refresh()
    texture = lbl.texture
    Color(0, 0, 0, 1)
    Rectangle(texture=texture, pos=(x, y), size=texture.size)
