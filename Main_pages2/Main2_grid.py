import json, os
import math

from kivy.uix.widget import Widget
from kivy.graphics import (
    Color, Rectangle, Line, Ellipse, Triangle
)
from kivy.core.text import Label as CoreLabel
from kivy.clock import Clock

from OpenCV.code.ui_bridge import post, FrameBus


# -------------------------------
# 색상 팔레트 (로봇용)
# -------------------------------
_ID_COLORS = [
    (0.90, 0.30, 0.30, 1),
    (0.30, 0.70, 0.30, 1),
    (0.30, 0.40, 0.90, 1),
]

def color_for_id(rid):
    rid = int(rid)   # 🔥 rid를 강제로 정수 변환
    return _ID_COLORS[(rid - 1) % len(_ID_COLORS)]


class GridWidget(Widget):

    def __init__(self, grid_json_path=None, **kwargs):
        super().__init__(**kwargs)

        # 백엔드에서 받아오는 상태
        self.grid_state = []        # 2D list/array (0:빈칸, 1:장애물)
        self.agent_states = {}      # {rid: (r,c)}
        self.agent_headings = {}    # {rid: heading_deg}
        self.home_positions = {}    # {rid: (r,c)}
        self.goal_positions = {}    # {rid: (r,c)}
        self.paths = []             # [(rid, [(r,c), ...])]

        # JSON Grid가 있으면 초기 로딩
        if grid_json_path:
            self.load_grid_from_json(grid_json_path)

        # 리사이즈 / 이동 시 다시 그리기
        self.bind(pos=self.update_canvas, size=self.update_canvas)

        # 주기적 리프레시 (백엔드 상태 반영용)
        Clock.schedule_interval(self.update_canvas, 1 / 30)


    # =====================================================
    # 1) JSON GRID LOAD
    # =====================================================
    def load_grid_from_json(self, json_path):
        if not os.path.exists(json_path):
            print(f"❌ Grid JSON not found: {json_path}")
            return

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
            self.grid_state = data["grid"]
            print("✅ JSON Grid loaded")
        except Exception as e:
            print("❌ JSON load error:", e)


    # =====================================================
    # 2) Backend → UI 업데이트
    # =====================================================
    def update_backend_state(self,
                             grid_state=None,
                             agent_states=None,
                             agent_headings=None,
                             home_positions=None,
                             goal_positions=None,
                             paths=None):

        if grid_state is not None:
            self.grid_state = grid_state

        if agent_states is not None:
            self.agent_states = agent_states

        if agent_headings is not None:
            self.agent_headings = agent_headings

        if home_positions is not None:
            self.home_positions = home_positions

        if goal_positions is not None:
            self.goal_positions = goal_positions

        if paths is not None:
            self.paths = paths


    # =====================================================
    # 3) 마우스 클릭 → 목표지 설정
    #    (백엔드와 동일한 좌표계 맞추기 위해 row flip)
    # =====================================================
    def on_touch_down(self, touch):

        if not self.collide_point(*touch.pos):
            return super().on_touch_down(touch)

        if self.grid_state is None or len(self.grid_state) == 0:
            return True

        rows = len(self.grid_state)
        cols = len(self.grid_state[0])

        # ─ 1) 현재 위젯 안에서의 정사각형 영역 계산 (더미 그리드와 동일)
        side = min(self.width, self.height)
        real_x = self.x + (self.width - side) / 2
        real_y = self.y + (self.height - side) / 2
        cw = side / cols
        ch = side / rows

        # 그리드 영역 바깥 클릭이면 무시
        if not (real_x <= touch.x <= real_x + side and real_y <= touch.y <= real_y + side):
            return super().on_touch_down(touch)

        # ─ 2) 화면 좌표 → grid cell (Kivy 기준 row = 아래에서 위로)
        local_x = touch.x - real_x
        local_y = touch.y - real_y

        r_kivy = int(local_y // ch)
        c_kivy = int(local_x // cw)

        # ─ 3) 백엔드 row는 위에서 아래로이므로 flip
        r_backend = (rows - 1) - r_kivy
        c_backend = c_kivy

        rid = FrameBus.get_selected_robot()
        if rid is None:
            print("[GridWidget] No robot selected.")
            return True

        post("set_goal", rid=rid, row=r_backend, col=c_backend)
        print(f"[GridWidget] Goal set → rid={rid}, pos=({r_backend}, {c_backend})")

        return True


    # =====================================================
    # 4) 전체 그리기 (더미 그리드와 동일한 스타일)
    # =====================================================
    def update_canvas(self, *args):

        self.canvas.clear()

        if self.grid_state is None or len(self.grid_state) == 0:
            return

        rows = len(self.grid_state)
        cols = len(self.grid_state[0])

        # ─ 더미 그리드와 동일: 정사각형 영역 안에 그리드 정렬
        side = min(self.width, self.height)
        real_x = self.x + (self.width - side) / 2
        real_y = self.y + (self.height - side) / 2
        cw = side / cols
        ch = side / rows

        with self.canvas:

            # ---------------------------------------------
            # A) 배경
            # ---------------------------------------------
            Color(0.96, 0.97, 0.99, 1)
            Rectangle(pos=(real_x, real_y), size=(side, side))

            # ---------------------------------------------
            # B) 장애물 (검정색, row flip 적용)
            # ---------------------------------------------
            for r in range(rows):
                for c in range(cols):
                    if self.grid_state[r][c] == 1:
                        Color(0.1, 0.1, 0.1, 1)
                        # y에 (rows-1-r) 적용해서 백엔드와 정방향으로 보여줌
                        x = real_x + c * cw
                        y = real_y + (rows - 1 - r) * ch
                        Rectangle(pos=(x, y), size=(cw, ch))

            # ---------------------------------------------
            # C) CBS 경로 (each rid 색, 투명하게)
            #    paths: [(rid, [(r,c), ...]), ...]
            # ---------------------------------------------
            for rid, path in self.paths:
                col = color_for_id(rid)
                for (r, c) in path:
                    Color(col[0], col[1], col[2], 0.18)
                    x = real_x + c * cw
                    y = real_y + (rows - 1 - r) * ch
                    Rectangle(pos=(x, y), size=(cw, ch))

            # ---------------------------------------------
            # D) 출발지 (home) 칸 표시
            # ---------------------------------------------
            for rid, (r, c) in self.home_positions.items():
                Color(*color_for_id(rid), 0.35)
                x = real_x + c * cw
                y = real_y + (rows - 1 - r) * ch
                Rectangle(pos=(x, y), size=(cw, ch))

            # ---------------------------------------------
            # E) 도착지 (goal) — 장애물 양옆 원형 + g1 텍스트
            # ---------------------------------------------
            for rid, goal in self.goal_positions.items():
                if goal is None:
                    continue
                (r, c) = goal
                cx = real_x + (c + 0.5) * cw
                cy = real_y + (rows - 1 - r + 0.5) * ch  # row flip 적용
                rad = min(cw, ch) * 0.30
                col = color_for_id(rid)

                Color(col[0], col[1], col[2], 0.75)
                Ellipse(pos=(cx - rad, cy - rad), size=(2 * rad, 2 * rad))

                # g1, g2 ... 텍스트
                self._draw_small_text(
                    text=f"g{rid}",
                    x_cell=c, y_cell=r,
                    base_x=real_x, base_y=real_y,
                    cw=cw, ch=ch,
                    rows=rows
                )

            # ---------------------------------------------
            # F) 로봇 아이콘 + 방향 표시 (원 + 화살표 + s1 텍스트)
            # ---------------------------------------------
            for rid, (r, c) in self.agent_states.items():

                cx = real_x + (c + 0.5) * cw
                cy = real_y + (rows - 1 - r + 0.5) * ch
                rad = min(cw, ch) * 0.28
                col = color_for_id(rid)

                # 원형 로봇
                Color(*col)
                Ellipse(pos=(cx - rad, cy - rad), size=(2 * rad, 2 * rad))

                # 로봇 방향 화살표 (더미 그리드 스타일)
                if rid in self.agent_headings:
                    hd = self.agent_headings[rid]   # deg
                    th = math.radians(hd)

                    tip = (cx + rad * 0.80 * math.cos(th),
                           cy + rad * 0.80 * math.sin(th))
                    left = (cx + rad * 0.55 * math.cos(th + 2.4),
                            cy + rad * 0.55 * math.sin(th + 2.4))
                    right = (cx + rad * 0.55 * math.cos(th - 2.4),
                             cy + rad * 0.55 * math.sin(th - 2.4))

                    Color(1, 1, 1, 1)
                    Triangle(points=[
                        tip[0], tip[1],
                        left[0], left[1],
                        right[0], right[1]
                    ])

                # s1, s2 ... 텍스트 (출발지 표시)
                self._draw_small_text(
                    text=f"s{rid}",
                    x_cell=c, y_cell=r,
                    base_x=real_x, base_y=real_y,
                    cw=cw, ch=ch,
                    rows=rows
                )

            # ---------------------------------------------
            # G) 격자선
            # ---------------------------------------------
            Color(0.80, 0.84, 0.90, 1)
            for i in range(rows + 1):
                y = real_y + i * ch
                Line(points=[real_x, y, real_x + side, y], width=1)

            for j in range(cols + 1):
                x = real_x + j * cw
                Line(points=[x, real_y, x, real_y + side], width=1)


    # =====================================================
    # 5) 작은 텍스트 (s1, g1 표시용)
    # =====================================================
    def _draw_small_text(self, text, x_cell, y_cell,
                         base_x, base_y, cw, ch, rows):

        # row flip 반영해서 좌표 계산
        px = base_x + x_cell * cw + 3
        py = base_y + (rows - 1 - y_cell) * ch + 3

        lbl = CoreLabel(text=text, font_size=ch * 0.23, color=(0, 0, 0, 1))
        lbl.refresh()
        tex = lbl.texture

        Color(0, 0, 0, 1)
        Rectangle(texture=tex, pos=(px, py), size=tex.size)
