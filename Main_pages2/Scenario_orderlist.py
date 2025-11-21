# Scenario_order_list_demo.py
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.scrollview import ScrollView
from kivy.uix.gridlayout import GridLayout
from kivy.clock import Clock
from kivy.graphics import Color, RoundedRectangle, Rectangle
from Utilities.UI_utilities import KLabel
import random


# =======================================================
# 주문 하나를 감싸는 카드 UI (라운드 박스)
# =======================================================
class OrderCard(BoxLayout):
    def __init__(self, text, status_color, **kwargs):
        super().__init__(
            orientation="vertical",
            size_hint_y=None,
            height=45,
            padding=[12, 8, 12, 8],
            **kwargs
        )

        # -------------------------
        # 카드 배경 (라운드 10px)
        # -------------------------
        with self.canvas.before:
            Color(0.86, 0.87, 0.90, 1)     # 연한 회색 배경
            self.bg = RoundedRectangle(
                pos=self.pos,
                size=self.size,
                radius=[10, 10, 10, 10]
            )

        self.bind(pos=self._update_bg, size=self._update_bg)

        # -------------------------
        # 텍스트 (상태만 색 표시)
        # -------------------------
        label = KLabel(
            text=text,
            markup=True,
            halign="left",
            valign="middle",
            font_size=13,
            color=(0, 0, 0, 1),  # 전체 텍스트는 검정
        )

        self.add_widget(label)

    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size



# =======================================================
# 시나리오 주문 리스트 (로그 30개, 데모용)
# =======================================================
class ScenarioOrderList(BoxLayout):

    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", spacing=5, padding=5, **kwargs)
        with self.canvas.before:
            Color(1, 1, 1, 1)  # 흰색
            self.bg = Rectangle(pos=self.pos, size=self.size)

        self.bind(pos=self._update_bg, size=self._update_bg)
        # 주문 번호 카운터
        self.order_count = 0

        # ---------------------
        # 리스트 제목
        # ---------------------
        title = KLabel(
            text="식당 시나리오 · 로봇 주문 로그",
            size_hint_y=None,
            height=32,
            font_size=20,
            halign="center",
            valign="middle",
            color=(0, 0, 0, 1),
        )
        self.add_widget(title)

        with self.canvas.after:
            Color(0.86, 0.87, 0.90, 1)     # 검은색
            self.separator = Rectangle(
                pos=(self.x, self.y + self.height - 40),
                size=(self.width, 1)
            )
        self.bind(pos=self._update_separator, size=self._update_separator)

        # ---------------------
        # 스크롤 + 리스트
        # ---------------------
        self.scroll = ScrollView(size_hint=(1, 1))

        self.list_layout = GridLayout(
            cols=1,
            size_hint_y=None,
            spacing=8,
            padding=[0, 6, 0, 6],
        )
        self.list_layout.bind(minimum_height=self.list_layout.setter("height"))

        self.scroll.add_widget(self.list_layout)
        self.add_widget(self.scroll)

        # ---------------------
        # 데모 주문 자동 생성
        # ---------------------
        Clock.schedule_interval(self._demo_add_random_order, 1.5)


    # ======================================================
    # 주문 로그 추가 (최대 30개 유지)
    # ======================================================
    def append_order(self, robot_id, target, status):

        # 1) 주문 번호 증가
        self.order_count += 1
        order_no = self.order_count

        # 2) 상태 색상 지정
        status_colors = {
            "주문 접수": (0.2, 0.4, 1.0, 1),     # 파랑
            "이동중":   (1.0, 0.55, 0.1, 1),    # 주황
            "픽업 완료": (0.2, 0.7, 0.2, 1),     # 초록
            "배달 완료": (0.6, 0.2, 0.9, 1),     # 보라
        }
        s_col = status_colors.get(status, (0, 0, 0, 1))

        # HEX 변환
        s_hex = _rgb_to_hex(s_col)

        # 3) 카드 안에 들어갈 텍스트
        card_text = (
            f"[{order_no}번 주문]  "
            f"로봇 {robot_id}  →  목적지 {target}  →  "
            f"[color={s_hex}]{status}[/color]"
        )

        # 4) 카드 생성 (라운딩 박스)
        card = OrderCard(text=card_text, status_color=s_col)

        # 5) 30개 초과 시 가장 오래된 것 삭제
        if len(self.list_layout.children) >= 30:
            oldest = self.list_layout.children[-1]
            self.list_layout.remove_widget(oldest)

        # 6) 카드 추가
        self.list_layout.add_widget(card)

        # 7) 자동 스크롤 맨 아래로
        Clock.schedule_once(lambda *_: setattr(self.scroll, 'scroll_y', 0))


    # ======================================================
    # 데모용 랜덤 주문 자동 생성
    # ======================================================
    def _demo_add_random_order(self, dt):
        robot_id = random.randint(1, 3)
        target = (random.randint(0, 5), random.randint(0, 5))
        status = random.choice(["주문 접수", "이동중", "픽업 완료", "배달 완료"])

        self.append_order(robot_id, target, status)


    # ======================================================
    # 리스트 초기화
    # ======================================================
    def clear_items(self):
        self.list_layout.clear_widgets()
        self.order_count = 0

    def _update_bg(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    # ------------------------------------------------------
    # 구분선 업데이트
    # ------------------------------------------------------
    def _update_separator(self, *args):
        # 제목 height=38 기준 → separator y는 제목 바로 아래
        self.separator.pos = (self.x, self.top - 40)
        self.separator.size = (self.width, 1)
        
# ----------------------------------------------------------
# RGB → HEX 변환 (Kivy 색상 markup용)
# ----------------------------------------------------------
def _rgb_to_hex(col):
    r, g, b, a = col
    return "#{:02x}{:02x}{:02x}".format(
        int(r * 255), int(g * 255), int(b * 255)
    )
