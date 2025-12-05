import json, os
import math

from kivy.uix.widget import Widget
from kivy.graphics import (
    Color, Rectangle, Line, Ellipse, Triangle
)
from kivy.core.text import Label as CoreLabel
from kivy.clock import Clock

from OpenCV.code.ui_bridge import post, FrameBus



# 각 로봇 ID별 색상

_ID_COLORS = [
    (0.90, 0.30, 0.30, 1),
    (0.30, 0.70, 0.30, 1),
    (0.30, 0.40, 0.90, 1),
]

def color_for_id(rid):
    rid = int(rid) 
    return _ID_COLORS[(rid - 1) % len(_ID_COLORS)]


class GridWidget(Widget):

    def __init__(self, grid_json_path=None, **kwargs):
        super().__init__(**kwargs)

        # 백엔드 상태
        self.grid_state = []        
        self.agent_states = {}      
        self.agent_headings = {}    
        self.home_positions = {}    
        self.goal_positions = {}    
        self.paths = [] 
        
        self.candidate_goals = set()           

        # JSON Grid 로딩
        if grid_json_path:
            self.load_grid_from_json(grid_json_path)

        # 리사이징
        self.bind(pos=self.update_canvas, size=self.update_canvas)

        # 주기적 상태 반영용
        Clock.schedule_interval(self.update_canvas, 1 / 30)


    
    # JSON GRID 
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


    
    # 2) Backend, UI 업데이트
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


   
    # 3) 마우스 클릭시 목표지 설정
    def on_touch_down(self, touch):

        if not self.collide_point(*touch.pos):
            return super().on_touch_down(touch)

        if self.grid_state is None or len(self.grid_state) == 0:
            return True

        rows = len(self.grid_state)
        cols = len(self.grid_state[0])


        side = min(self.width, self.height)
        real_x = self.x + (self.width - side) / 2
        real_y = self.y + (self.height - side) / 2
        cw = side / cols
        ch = side / rows

        
        if not (real_x <= touch.x <= real_x + side and real_y <= touch.y <= real_y + side):
            return super().on_touch_down(touch)

        
        local_x = touch.x - real_x
        local_y = touch.y - real_y

        r_kivy = int(local_y // ch)
        c_kivy = int(local_x // cw)

       
        r_backend = (rows - 1) - r_kivy
        c_backend = c_kivy

        rid = FrameBus.get_selected_robot()
        if rid is None:
            print("[GridWidget] No robot selected.")
            return True

        post("set_goal", rid=rid, row=r_backend, col=c_backend)
        print(f"[GridWidget] Goal set → rid={rid}, pos=({r_backend}, {c_backend})")

        return True


    
    # 4) 전체 그리기
    def update_canvas(self, *args):
        cands = FrameBus.get_candidate_goals()
        if cands:
            self.candidate_goals = set(tuple(p) for p in cands)
        self.canvas.clear()

        if self.grid_state is None or len(self.grid_state) == 0:
            return

        rows = len(self.grid_state)
        cols = len(self.grid_state[0])

        side = min(self.width, self.height)
        real_x = self.x + (self.width - side) / 2
        real_y = self.y + (self.height - side) / 2
        cw = side / cols
        ch = side / rows

        with self.canvas:

            
            # 배경
            Color(0.96, 0.97, 0.99, 1)
            Rectangle(pos=(real_x, real_y), size=(side, side))

           
            #  장애물 
            
            for r in range(rows):
                for c in range(cols):
                    if self.grid_state[r][c] == 1:
                        Color(0.1, 0.1, 0.1, 1)
                        x = real_x + c * cw
                        y = real_y + (rows - 1 - r) * ch
                        Rectangle(pos=(x, y), size=(cw, ch))

           
            #CBS 경로 
            for rid, path in self.paths:
                col = color_for_id(rid)
                for (r, c) in path:
                    Color(col[0], col[1], col[2], 0.18)
                    x = real_x + c * cw
                    y = real_y + (rows - 1 - r) * ch
                    Rectangle(pos=(x, y), size=(cw, ch))

           
            #출발지 칸 표시

            for rid, (r, c) in self.home_positions.items():
                Color(*color_for_id(rid), 0.35)
                x = real_x + c * cw
                y = real_y + (rows - 1 - r) * ch
                Rectangle(pos=(x, y), size=(cw, ch))

                
                self._draw_cell_top_left_text(
                    text=str(rid),       
                    x_cell=c, y_cell=r,
                    base_x=real_x, base_y=real_y,
                    cw=cw, ch=ch,
                    rows=rows
                )

            
            #도착지 
            for rid, goal in self.goal_positions.items():
                if goal is None:
                    continue
                (r, c) = goal
                cx = real_x + (c + 0.5) * cw
                cy = real_y + (rows - 1 - r + 0.5) * ch  
                rad = min(cw, ch) * 0.30
                col = color_for_id(rid)

                Color(col[0], col[1], col[2], 0.75)
                Ellipse(pos=(cx - rad, cy - rad), size=(2 * rad, 2 * rad))

               
                self._draw_small_text(
                    text=f"g{rid}",
                    x_cell=c, y_cell=r,
                    base_x=real_x, base_y=real_y,
                    cw=cw, ch=ch,
                    rows=rows
                )

            
            # 로봇 아이콘 
            for rid, (r, c) in self.agent_states.items():

                cx = real_x + (c + 0.5) * cw
                cy = real_y + (rows - 1 - r + 0.5) * ch
                rad = min(cw, ch) * 0.28
                col = color_for_id(rid)

                
                Color(*col)
                Ellipse(pos=(cx - rad, cy - rad), size=(2 * rad, 2 * rad))

   
                self._draw_small_text(
                    text=f"s{rid}",
                    x_cell=c, y_cell=r,
                    base_x=real_x, base_y=real_y,
                    cw=cw, ch=ch,
                    rows=rows
                )


           
            # 격자선
            Color(0xAB / 255, 0xAB / 255, 0xAB / 255, 1)
            for i in range(rows + 1):
                y = real_y + i * ch
                Line(points=[real_x, y, real_x + side, y], width=1)

            for j in range(cols + 1):
                x = real_x + j * cw
                Line(points=[x, real_y, x, real_y + side], width=1)
            
            
            # 🔶 7) 딜리버리 후보 목적지 노란 점
            Color(1, 1, 0, 1)
            for (r, c) in self.candidate_goals:
                cx = real_x + (c + 0.5) * cw
                cy = real_y + (rows - 1 - r + 0.5) * ch
                rad = min(cw, ch) * 0.15
                Ellipse(pos=(cx - rad, cy - rad), size=(2 * rad, 2 * rad))

  
    #s1, g1 표시용 텍스트

    def _draw_small_text(self, text, x_cell, y_cell,
                         base_x, base_y, cw, ch, rows):

      
        px = base_x + x_cell * cw + 3
        py = base_y + (rows - 1 - y_cell) * ch + 3

        lbl = CoreLabel(text=text, font_size=ch * 0.23, color=(0, 0, 0, 1))
        lbl.refresh()
        tex = lbl.texture

        Color(0, 0, 0, 1)
        Rectangle(texture=tex, pos=(px, py), size=tex.size)

    def _draw_cell_top_left_text(self, text, x_cell, y_cell,
                             base_x, base_y, cw, ch, rows):

     
        px = base_x + x_cell * cw + 3
        py = base_y + (rows - 1 - y_cell + 1) * ch - (ch * 0.30)

        lbl = CoreLabel(text=text, font_size=ch * 0.25, color=(0, 0, 0, 1))
        lbl.refresh()
        tex = lbl.texture

        Color(0, 0, 0, 1)
        Rectangle(texture=tex, pos=(px, py), size=tex.size)

