from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.graphics import Color, Rectangle, Line
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import Clock

from Utilities.UI_utilities import KLine, make_darkcell, make_brightcell, KButton
from OpenCV.code.ui_bridge import FrameBus, post
from OpenCV.code.config import grid_row, grid_col

import numpy as np


#그리드 생성 클래스

class GridTextureView(Image):
    """FrameBus.get_grid() (BGR ndarray)를 읽어 전체 영역에 그리드 텍스처 표시 + 클릭 이벤트 처리"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.allow_stretch = True
        self.keep_ratio = True
        self.center_widget = None  # CenterWidget 참조 (외부에서 주입)
        Clock.schedule_interval(self._update, 1/20)  # 20fps

    def _update(self, dt):
        frame = FrameBus.get_grid()
        if frame is None:
            return
        rgb = frame[:, :, ::-1].copy()
        h, w = rgb.shape[:2]

        if not self.texture or self.texture.width != w or self.texture.height != h:
            self.texture = Texture.create(size=(w, h))
            # OpenCV 좌표와 맞추기 위해 수직 플립
            self.texture.flip_vertical()
        self.texture.blit_buffer(rgb.tobytes(), colorfmt='rgb', bufferfmt='ubyte')
        self.canvas.ask_update()

    def on_touch_down(self, touch):
        if not self.collide_point(*touch.pos):
            return False

        # 텍스처(원본 이미지) 크기
        if not self.texture:
            return False
        img_w = float(self.texture.width)
        img_h = float(self.texture.height)

        # 위젯 화면상 크기
        W = float(self.width)
        H = float(self.height)

        # keep_ratio=True일 때 실제 그려지는 영역(레터박스 제외) 계산
        s = min(W / img_w, H / img_h)          # 축소 비율
        draw_w = img_w * s
        draw_h = img_h * s
        off_x = (W - draw_w) * 0.5
        off_y = (H - draw_h) * 0.5

        # 터치 좌표를 컨텐츠 기준 좌표로 변환 알고리즘
        lx = touch.x - self.x - off_x
        ly = touch.y - self.y - off_y

        # 컨텐츠 바깥 클릭 무시
        if lx < 0 or ly < 0 or lx > draw_w or ly > draw_h:
            return False

        # 0~1 정규화 (Kivy는 아래가 0, 위가 H) 
        u = lx / draw_w              # 좌→우
        v = ly / draw_h              # 아래→위
        
        # 영상/그리드는 위가 0이므로 y를 반전
        v_top = 1.0 - v

        # 그리드 셀 인덱스 계산 + 경계 클램프
        col = int(u * grid_col)
        row = int(v_top * grid_row)
        col = max(0, min(grid_col - 1, col))
        row = max(0, min(grid_row - 1, row))

        if self.center_widget and self.center_widget.selected_robot_id is not None:
            post("set_goal", rid=self.center_widget.selected_robot_id, row=row, col=col)
            print(f"[UI] Grid 클릭: row={row}, col={col}, robot={self.center_widget.selected_robot_id}")
        else:
            print("⚠️ 로봇 선택이 필요합니다. (상단의 '로봇 1~4' 버튼 클릭)")
        return True


# ---------------- CenterWidget ----------------
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

        # ── 상단 50%: 로봇 정보 + 그리드뷰 + 선택버튼
        upper_half = BoxLayout(orientation='vertical', size_hint_y=0.5, spacing=4, padding=[0,4,0,4])
        upper_half.add_widget(make_brightcell("총 로봇 대수"))
        self.robot_count_cell = make_brightcell("0")  # 초기값은 0
        upper_half.add_widget(self.robot_count_cell)

        # 그리드 뷰
        self.grid_view = GridTextureView(size_hint_y=1)
        self.grid_view.center_widget = self  # ★ CenterWidget 참조 주입
        upper_half.add_widget(self.grid_view)

        # 로봇 선택 버튼 4개
        robot_btns = BoxLayout(size_hint_y=None, height=40, spacing=4)
        for i in range(1, 5):
            btn = KButton(text=f"로봇 {i}")
            btn.bind(on_press=lambda inst, rid=i: self.select_robot(rid))
            robot_btns.add_widget(btn)
        upper_half.add_widget(robot_btns)

        # ── 하단 50%: 장애물 + 제어 버튼들
        lower_half = BoxLayout(orientation='vertical', size_hint_y=0.5, padding=[0,4,0,4])
        

        # 보드 고정 / 해제 / ROI 재선택 / 시각화 토글 버튼들
        board_btns = BoxLayout(size_hint_y=None, height=40, spacing=4)

        btn_lock = KButton(text="보드 고정") #키보드 n 대응
        btn_lock.bind(on_press=lambda inst: post("lock_board"))
        board_btns.add_widget(btn_lock)

        btn_unlock = KButton(text="보드 해제")
        btn_unlock.bind(on_press=lambda inst: post("unlock_board"))
        board_btns.add_widget(btn_unlock)

        btn_roi = KButton(text="보드 재선택(ROI)")
        btn_roi.bind(on_press=lambda inst: post("start_roi_selection"))
        board_btns.add_widget(btn_roi)

        btn_viz = KButton(text="시각화 토글")
        btn_viz.bind(on_press=lambda inst: post("toggle_visualization"))
        board_btns.add_widget(btn_viz)

        lower_half.add_widget(board_btns)

        # 정렬 버튼들 (센터 정렬 = a / 방향 정렬 = f)
        align_btns = BoxLayout(size_hint_y=None, height=40, spacing=4)

        btn_center = KButton(text="정렬(센터)")   # 키보드 'a' 대응
        btn_center.bind(on_press=lambda inst: post("center_align"))
        align_btns.add_widget(btn_center)

        btn_dir = KButton(text="정렬(방향)")      # 키보드 'f' 대응
        btn_dir.bind(on_press=lambda inst: post("direction_align"))
        align_btns.add_widget(btn_dir)

        lower_half.add_widget(align_btns)

        # CBS 제어 버튼들 (경로탐색 / 정지 / 재개)
        cbs_btns = BoxLayout(size_hint_y=None, height=40, spacing=4)

        btn_run = KButton(text="경로탐색")        # 키보드 'c' 대응
        btn_run.bind(on_press=lambda inst: post("compute_cbs"))
        cbs_btns.add_widget(btn_run)

        btn_stop = KButton(text="정지")
        btn_stop.bind(on_press=lambda inst: post("pause"))
        cbs_btns.add_widget(btn_stop)

        btn_resume = KButton(text="재개")
        btn_resume.bind(on_press=lambda inst: post("resume"))
        cbs_btns.add_widget(btn_resume)

        lower_half.add_widget(cbs_btns)

        # 중앙 가로선
        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.middle_line = Line(points=[], width=1)
        self.bind(pos=self.update_middle_line, size=self.update_middle_line)

        # 최종 배치
        self.add_widget(upper_half)
        self.add_widget(lower_half)

    # 로봇 선택 핸들러
    def select_robot(self, rid):
        self.selected_robot_id = rid
        # ★ 백엔드의 SELECTED_RIDS 를 이 로봇 하나로 동기화
        post("select_robot", rid=rid)
        print(f"[UI] 로봇 {rid} 선택됨")

    def update_bg_and_border(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def update_middle_line(self, *args):
        x = self.x
        y = self.y + self.height / 2
        self.middle_line.points = [x, y, x + self.width, y]
