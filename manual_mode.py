"""
manual_mode.py — 외부 수동 경로 시스템 (Global MANUAL/CBS 토글)

목표
- main을 거의 건드리지 않고도 수동 경로 생성 기능을 붙일 수 있도록 모듈화.
- 'z' 키로 MANUAL ↔ CBS 모드 전환(모드는 모든 로봇에 전역으로 동일하게 적용).
- MANUAL 모드에서 번호키(1~4 등)로 선택한 로봇의 경로를 클릭으로 작성.
- 'c' 키: MANUAL 모드면 수동 경로 전송(start_sequence), CBS 모드면 main 쪽 기존 로직 사용.
- 'r' 키: 수동 경로 전체 초기화(선택 로봇만 초기화하고 싶으면 provide API 사용).
- draw_overlay(vis): 그리드 위 오버레이 렌더링.

통합 가이드 (main.py 변경 최소화)
---------------------------------------------------
# 1) import 및 인스턴스 생성 (main.py 상단 또는 초기화 위치)
from manual_mode import ManualPathSystem
manual = ManualPathSystem(
    get_selected_rids=lambda: SELECTED_RIDS,           # main이 유지하는 선택 집합
    get_preset_ids=lambda: PRESET_IDS,                 # 현재 보이는/접속중 로봇 ID 목록
    grid_shape=(grid_row, grid_col),
    cell_size_px=cell_size,                            # 그리드 1셀 픽셀 크기
    cell_size_cm=cell_size_cm,                         # 1셀의 실제 이동 거리(cm)
    path_to_commands=path_to_commands,                 # (path, init_hd) -> [{command: ...}] 제공
    start_sequence=start_sequence,                     # {rid:[cmd,...]}를 순차 전송하는 콜백
    get_initial_hd=get_initial_hd                      # rid -> 0/1/2/3(N/E/S/W)
)

# 2) 마우스 콜백 래퍼로 교체
#   - MANUAL 모드: manual.on_mouse가 클릭 처리
#   - CBS 모드: 기존 mouse_event 로직 유지

def unified_mouse(event, x, y, flags, param):
    if manual.is_manual_mode():
        manual.on_mouse(event, x, y)
    else:
        # 기존 CBS 마우스 핸들러(출발지/도착지 지정)
        mouse_event(event, x, y, flags, param)

cv2.setMouseCallback("CBS Grid", unified_mouse)

# 3) 키 처리에 몇 줄 추가
elif key == ord('z'):
    manual.toggle_mode()
elif key == ord('c'):
    if manual.is_manual_mode():
        manual.commit()   # 수동 경로 전송
    else:
        # 기존 CBS 실행 로직 그대로
        ready_agents = [a for a in agents if a.start and a.goal]
        if ready_agents:
            compute_cbs_only(ready_agents)
elif key == ord('r'):
    manual.reset_paths()  # 수동 경로만 초기화(원하면 기존 reset과 함께 사용)

# 4) 그리드 렌더링 직후 오버레이 호출
manual.draw_overlay(vis)
---------------------------------------------------

주의
- "번호키로 선택"은 main이 유지하는 SELECTED_RIDS를 그대로 사용합니다.
- MANUAL 모드에서 한 번에 한 로봇만 선택되어 있는 것을 권장합니다(2개 이상 선택 시 경고).
- path_to_commands는 (연속 셀) 경로를 가정합니다. 본 모듈은 같은 행/열인 비연속 클릭 지점 사이의 중간 셀을 자동 보간합니다(맨해튼). 대각 이동 클릭은 "행 먼저, 열 다음" 순서로 자동 분해합니다.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Tuple
import cv2
import numpy as np

GridCell = Tuple[int, int]  # (row, col)

# ---------- 유틸 ----------

def _in_bounds(rc: GridCell, shape: Tuple[int, int]) -> bool:
    r, c = rc
    rows, cols = shape
    return 0 <= r < rows and 0 <= c < cols


def _manhattan_fill(a: GridCell, b: GridCell) -> List[GridCell]:
    """a->b 를 같은 행/열이면 직선 보간, 대각이면 (행→열) 순서로 보간.
    반환: a 이후에 이어 붙일 중간 셀들(끝점 b 포함, 단 a는 포함하지 않음)
    """
    r0, c0 = a
    r1, c1 = b
    out: List[GridCell] = []
    if r0 == r1:
        step = 1 if c1 > c0 else -1
        for c in range(c0 + step, c1 + step, step):
            out.append((r0, c))
    elif c0 == c1:
        step = 1 if r1 > r0 else -1
        for r in range(r0 + step, r1 + step, step):
            out.append((r, c0))
    else:
        # 대각: 행 먼저, 그다음 열
        out.extend(_manhattan_fill(a, (r1, c0)))
        out.extend(_manhattan_fill((r1, c0), b))
    return out


# ---------- 메인 클래스 ----------

@dataclass
class ManualPathSystem:
    # 외부 의존성 주입(콜백)
    get_selected_rids: Callable[[], Iterable[int]]
    get_preset_ids: Callable[[], Iterable[int]]
    grid_shape: Tuple[int, int]
    cell_size_px: int
    cell_size_cm: float
    path_to_commands: Callable[[List[GridCell], int], List[Dict]]
    start_sequence: Callable[[Dict[str, List[str]]], None]
    get_initial_hd: Callable[[int], int]

    # 내부 상태
    mode: str = "CBS"  # "CBS" or "MANUAL"
    manual_paths: Dict[int, List[GridCell]] = field(default_factory=dict)

    # 스타일
    color_map: Dict[int, Tuple[int, int, int]] = field(default_factory=lambda: {
        1: (60, 180, 255),  # BGR
        2: (0, 200, 0),
        3: (240, 180, 70),
        4: (200, 100, 255),
    })
    # 표시 옵션
    show_mode_badge: bool = False  # ← 기본값: 안 보이게

    # ---------------- 모드 제어 ----------------
    def is_manual_mode(self) -> bool:
        return self.mode == "MANUAL"

    def toggle_mode(self) -> None:
        self.mode = "MANUAL" if self.mode == "CBS" else "CBS"
        print(f"🔁 모드 전환 → {self.mode} (전역 적용)")
        if self.mode == "CBS":
            # 필요시 수동 경로 유지하거나 초기화
            pass

    # ---------------- 입력 처리 ----------------
    def on_mouse(self, event: int, x: int, y: int) -> None:
        """MANUAL 모드에서만 마우스 클릭으로 경로 편집.
        CBS 모드일 때는 main의 기존 mouse_event가 처리하게 두세요.
        """
        if not self.is_manual_mode():
            return

        row, col = y // self.cell_size_px, x // self.cell_size_px
        if not _in_bounds((row, col), self.grid_shape):
            return

        if event != cv2.EVENT_LBUTTONDOWN:  # 좌클릭만 사용
            return

        selected = list(self.get_selected_rids())
        if len(selected) == 0:
            print("⚠️ 선택된 로봇이 없습니다. 번호키(1~4 등)로 대상을 선택하세요.")
            return
        if len(selected) > 1:
            print("⚠️ 한 번에 하나의 로봇만 경로를 작성할 수 있습니다. 하나만 선택하세요.")
            return

        rid = selected[0]
        visible = set(self.get_preset_ids())
        if rid not in visible:
            print(f"⚠️ Robot_{rid} 가 현재 화면/접속 목록에 없습니다.")
            return

        path = self.manual_paths.setdefault(rid, [])
        new_pt = (row, col)
        if len(path) == 0:
            path.append(new_pt)
            print(f"[MANUAL] Robot_{rid} 시작 셀 → {new_pt}")
            return

        last = path[-1]
        if new_pt == last:
            return  # 중복 클릭 무시

        # 연속 셀 보장(맨해튼 자동 보간)
        seg = _manhattan_fill(last, new_pt)
        # 경계 밖 셀 제거
        seg = [p for p in seg if _in_bounds(p, self.grid_shape)]
        if not seg:
            return
        path.extend(seg)
        print(f"[MANUAL] Robot_{rid} 경로 추가 → {seg[-1]} (총 {len(path)} 셀)")

    # ---------------- 전송/리셋 ----------------
    def commit(self) -> None:
        """MANUAL 모드의 현재 경로들을 명령으로 변환 후 start_sequence 전송.
        - 접속중인 모든 로봇을 고려(빈 경로는 건너뜀)
        - path_to_commands(path, init_hd) 사용
        """
        if not self.is_manual_mode():
            print("⚠️ 현재 CBS 모드입니다. 수동 경로 전송은 MANUAL 모드에서만 가능합니다.")
            return

        visible = list(self.get_preset_ids())
        if not visible:
            print("⚠️ 접속중인 로봇이 없습니다.")
            return

        cmd_map: Dict[str, List[str]] = {}
        empty: List[int] = []
        for rid in visible:
            path = self.manual_paths.get(rid, [])
            if len(path) < 2:
                empty.append(rid)
                continue
            init_hd = self.get_initial_hd(rid)
            cmds_obj = self.path_to_commands(path, init_hd)
            cmds = [c["command"] for c in cmds_obj]
            if cmds:
                cmd_map[str(rid)] = cmds

        if not cmd_map:
            print("⚠️ 전송할 경로가 없습니다. (각 로봇 경로는 최소 2셀 이상 필요)")
            if empty:
                print(f"  ↳ 빈 경로: {sorted(empty)}")
            return

        print("▶ [MANUAL] 순차 전송 시작:", cmd_map)
        self.start_sequence(cmd_map)

    def reset_paths(self) -> None:
        self.manual_paths.clear()
        print("🔄 [MANUAL] 모든 수동 경로 초기화 완료")

    def clear_selected_path(self) -> None:
        selected = list(self.get_selected_rids())
        if len(selected) != 1:
            print("⚠️ 선택이 1개일 때만 해당 경로를 지울 수 있습니다.")
            return
        rid = selected[0]
        if rid in self.manual_paths:
            self.manual_paths.pop(rid, None)
            print(f"🧹 [MANUAL] Robot_{rid} 경로 초기화")

    def undo_last(self) -> None:
        selected = list(self.get_selected_rids())
        if len(selected) != 1:
            print("⚠️ 선택이 1개일 때만 되돌리기가 가능합니다.")
            return
        rid = selected[0]
        path = self.manual_paths.get(rid)
        if path:
            path.pop()
            print(f"↩️ [MANUAL] Robot_{rid} 마지막 셀 제거 (남은 {len(path)} 셀)")

    # ---------------- 오버레이 ----------------
    def draw_overlay(self, vis_img: np.ndarray) -> None:
        """그리드 렌더링된 vis_img 위에 수동 경로를 그립니다."""
        if vis_img is None:
            return
        # 보이지 않는 로봇의 경로는 정리(선택적)
        visible = set(self.get_preset_ids())
        for rid in list(self.manual_paths.keys()):
            if rid not in visible:
                self.manual_paths.pop(rid, None)

        # 그리기
        h, w = vis_img.shape[:2]
        cs = self.cell_size_px
        for rid, path in self.manual_paths.items():
            if len(path) == 0:
                continue
            color = self.color_map.get(rid, (255, 255, 255))
            # 점 및 선분
            pts_px = [((c * cs + cs // 2), (r * cs + cs // 2)) for r, c in path]
            for i, (x, y) in enumerate(pts_px):
                cv2.circle(vis_img, (x, y), max(2, cs // 6), color, -1)
                if i > 0:
                    cv2.line(vis_img, pts_px[i - 1], (x, y), color, max(1, cs // 10), lineType=cv2.LINE_AA)
            # 라벨
            x0, y0 = pts_px[0]
            cv2.putText(vis_img, f"R{rid}", (x0 + 6, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        # 모드 배지(기본 숨김)
        if self.show_mode_badge:
            badge = f"MODE: {self.mode}"
            cv2.rectangle(vis_img, (8, 8), (168, 34), (0, 0, 0), -1)
            cv2.putText(vis_img, badge, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
