import sys
import os
import cv2
import numpy as np
import subprocess 
import math
import time
import threading
from queue import Queue, Empty


# UI 연동 관련

SHOW_CV_WINDOWS = bool(int(os.environ.get("SHOW_CV_WINDOWS", "1")))

_KEYQ: "Queue[int]" = Queue()

def push_keycode(code: int):
    """외부(Kivy)에서 보낸 가상 키코드를 백엔드에 전달"""
    _KEYQ.put(code)


# Path & Import

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))

from OpenCV.code.grid import load_grid, GRID_FOLDER
from OpenCV.code.interface import grid_visual, slider_create, slider_value, draw_agent_points, draw_paths
from OpenCV.code.config import (
    grid_row, grid_col, cell_size, camera_cfg,
    IP_address_, MQTT_TOPIC_COMMANDS_, MQTT_PORT,
    NORTH_TAG_ID, CORRECTION_COEF, critical_dist, cell_size_cm
)
from OpenCV.code.vision.visionsystem import VisionSystem 
from OpenCV.code.vision.camera import camera_open, Undistorter 
from OpenCV.code.cbs.pathfinder import PathFinder, Agent
from OpenCV.code.controller.RobotController import RobotController
from OpenCV.code.controller.manual_mode import ManualPathSystem
from OpenCV.code.controller.collision_guard import CollisionGuard, GuardConfig
from OpenCV.code.controller.release_manager import ReleaseManager, ReleasePolicy
from OpenCV.code.ui_bridge import FrameBus, get_cmd_nowait


# 글로벌 상태

SELECTED_RIDS = set()
GOAL_ALIGN_MODE = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CTS_SCRIPT = os.path.join(SCRIPT_DIR, "command_transfer.py")

# 메인 로직이 실행되기 전에 커맨드 전송 스크립트를 백그라운드로 시작
# sys.executable: 현재 사용 중인 파이썬 인터프리터 경로
proc = subprocess.Popen([sys.executable, CTS_SCRIPT], creationflags=subprocess.CREATE_NEW_CONSOLE)
print(f"▶ command_transfer.py 별도 콘솔에서 실행: {CTS_SCRIPT}")


# 브로커 정보
# main.py 상단에 USE_MQTT 정의

USE_MQTT = 1  # 0: 비사용, 1: 사용

if USE_MQTT:
    from OpenCV.code.recieve_message import init_mqtt_client
    client = init_mqtt_client()
else:
    MQTT_TOPIC_COMMANDS_ = None
    class _DummyClient:
        def publish(self, topic, payload):
            print(f"[MQTT_DISABLED] publish → topic={topic}, payload={payload}")
    client = _DummyClient()

controller = RobotController(
    client=client,
    mqtt_topic_commands=MQTT_TOPIC_COMMANDS_,
    done_topic="robot/done",
    north_tag_id=NORTH_TAG_ID,
    direction_corr_threshold_deg=3.0,
    alignment_delay_sec=0.8,
    alignment_angle=1.0,
    alignment_dist=1.0,
) 
#VisionSystem 인스턴스를 리턴
controller.set_vision_system_provider(lambda: vision if "vision" in globals() else None)

if USE_MQTT:
    def _on_msg(c, u, m):
        try:
            controller.on_mqtt_message(m.topic, m.payload)
        except Exception as e:
            print(f"[on_message error] {e}")

    client.on_message = _on_msg
    try:
        client.subscribe(controller.done_topic)
    except Exception:
        pass


# 보정 패널

correction_coef_value = CORRECTION_COEF

def correction_trackbar_callback(val):
    global correction_coef_value
    correction_coef_value = val / 100.0
    print(f"[INFO] 실시간 보정계수: {correction_coef_value:.2f}")

if SHOW_CV_WINDOWS:
    cv2.namedWindow("CorrectionPanel", cv2.WINDOW_NORMAL)
    cv2.createTrackbar(
        "Correction Coef", "CorrectionPanel",
        int(CORRECTION_COEF * 100), 200, correction_trackbar_callback
    )
correction_trackbar_callback(int(CORRECTION_COEF * 100))


# 안전 제어 (CollisionGuard, ReleaseManager)

def immediate_stop(client, ids):
    for rid in ids:
        client.publish(f"robot/{rid}/cmd", "im_S")
        print(f"🛑 [Robot_{rid}] 즉시정지(im_S) 전송")

guard = CollisionGuard(
    stop_fn=lambda ids: immediate_stop(client, ids),
    config=GuardConfig(
        step_cm=15.0,
        collision_radius_cm=10.0,
        tau_latency_s=0.15,
        eps_step_cm=1.0,
        vmin_cmps=2.0,
    ),
)

guard.set_goal_provider(lambda rid: (
    tuple(controller.get_step_goal_cm(rid)) if hasattr(controller, "get_step_goal_cm") else None
))

rm = ReleaseManager(
    guard=guard,
    controller=controller,
    client=client,
    policy=ReleasePolicy(arming_delay_s=3.0, manage_window_s=1.25),
)


# 전역 변수

grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)
agents = []
paths = []
pathfinder = None
grid_array = None
visualize = True
tag_info = {}



# 비전 시스템

video_path = r"C:/img/test2.mp4"
cap, fps = camera_open(source=None)

undistorter = Undistorter(
    camera_cfg['type'],
    camera_cfg['matrix'],
    camera_cfg['dist'],
    camera_cfg['size']
)
vision = VisionSystem(undistorter=undistorter, visualize=True)
vision.correction_coef_getter = lambda: correction_coef_value

#로봇 ID관련
PRESET_IDS = []
selected_robot_id = None

# 유틸 함수

def compute_visible_robot_ids(tag_info: dict) -> list[int]:
    """카메라에 잡힌 '로봇' 태그 ID를 정렬 리스트로 반환 (보드/NORTH_TAG_ID 제외)."""
    visible = []
    for tid, data in tag_info.items():
        # tid는 정수, 'On' 상태, 보드 태그는 제외
        if isinstance(tid, int) and data.get("status") == "On" and tid != NORTH_TAG_ID:
            visible.append(tid)
    visible.sort()
    return visible

def _get_tag_cm(tag_info: dict, rid: int):
    d = tag_info.get(rid, {})
    if d.get("status") == "On" and "corrected_center" in d:
        return d["corrected_center"]  # (X_cm, Y_cm)
    return None


# 마우스 콜백 함수
def mouse_event(event, x, y, flags, param):
    global agents, paths, pathfinder, selected_robot_id

    if event != cv2.EVENT_RBUTTONDOWN:
        return  # 우클릭만 처리

    try:
        row, col = y // cell_size, x // cell_size
        if not (0 <= row < grid_row and 0 <= col < grid_col):
            return

        # 1) 선택된 로봇이 없다면
        if selected_robot_id is None:
            print("⚠️ 목표를 지정할 로봇이 선택되지 않았습니다. 숫자(1~9)로 로봇을 먼저 선택하세요.")
            return

        # 2) 실제 로봇/에이전트 존재 확인
        target = next((a for a in agents if a.id == selected_robot_id), None)
        if target is None:
            print(f"❌ 로봇 {selected_robot_id} 을(를) 찾을 수 없습니다. 선택을 해제합니다.")
            selected_robot_id = None
            return

        # 3) goal만 갱신 (CBS 실행/후처리 없음)
        target.goal = (row, col)
        print(f"✅ 로봇 {selected_robot_id} 의 목표를 ({row}, {col}) 로 설정했습니다.")

    except Exception as e:
        print(f"[mouse_event error] {e}")
    finally:
        # 우클릭 한 번으로 끝 — 선택은 해제
        selected_robot_id = None


def update_agents_from_tags(tag_info):        # cm → 셀 좌표
    for tag_id, data in tag_info.items():
        if tag_id not in PRESET_IDS:
            continue
        if data.get("status") != "On":
            continue
        start_cell = data["grid_position"]
        existing = next((a for a in agents if a.id == tag_id), None)
        if existing:
            if existing.start != start_cell:
                existing.start = start_cell
        else:
            agents.append(Agent(id=tag_id, start=start_cell, goal=None, delay=0))


def path_to_commands(path, init_hd=0):
    """
    path: [(r0,c0), (r1,c1), ...]
    init_hd: 0=북,1=동,2=남,3=서
    반환: [{'command': 'Stay'|'L90'|'R90'|'T185'|'F10_modeA'}, ...]
    """
    cmds = []
    hd = init_hd

    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        # 0) 같은 좌표 → '대기'
        if r0 == r1 and c0 == c1:
            cmds.append({'command': 'Stay'})
            continue

        # 1) 목표 방향
        if   r1 < r0:  desired = 0  # 북
        elif c1 > c0:  desired = 1  # 동
        elif r1 > r0:  desired = 2  # 남
        else:          desired = 3  # 서

        # 2) 회전/이동 단일 명령
        diff = (desired - hd) % 4
        if diff == 0:
            # 회전 불필요 → 전진만
            cmds.append({'command': f'F{cell_size_cm:.1f}_modeA'})
        elif diff == 1:
            cmds.append({'command': 'R90'})
        elif diff == 2:
            cmds.append({'command': 'T185'})  # 180도 보정치
        else:  # diff == 3
            cmds.append({'command': 'L90'})

        # 3) 헤딩 갱신
        hd = desired

    return cmds


YAW_TO_NORTH_OFFSET_DEG = 0  # 필요시 -90 / +90 / 180 등으로 보정

def yaw_to_hd(yaw_deg: float, offset_deg: float = 0) -> int:
    """연속각(yaw_deg)을 90° 섹터로 양자화하여 hd(0~3)로 변환"""
    ang = (yaw_deg + offset_deg) % 360.0
    return int(((ang + 45.0) // 90.0) % 4)

def get_initial_hd(robot_id: int) -> int:
    data = tag_info.get(robot_id)
    if not data or data.get('status') != 'On':
        return 0
    
    # 화면 표시용 방향/오차 값 사용
    delta = data.get("heading_offset_deg")
    if delta is None:
        return 0

    # base_dir 추출
    yaw_deg = (data.get("yaw_front_deg", 0) + 360) % 360
    direction_angles = [90, 0, 270, 180]  # N=90, W=0, S=270, E=180
    diffs = [abs(((yaw_deg - a + 180) % 360) - 180) for a in direction_angles]
    min_idx = diffs.index(min(diffs))
    hd = [0, 3, 2, 1][min_idx]  # N=0, E=1, S=2, W=3 로 매핑

    return hd

def _start_sequence_wrapper(cmd_map: dict, step_cell_plan: dict | None = None):
    controller.start_sequence(cmd_map, step_cell_plan)  # ← plan을 통과시킴

manual = ManualPathSystem(
    get_selected_rids=lambda: SELECTED_RIDS,
    get_preset_ids=lambda: PRESET_IDS,
    grid_shape=(grid_row, grid_col),
    cell_size_px=cell_size,
    cell_size_cm=cell_size_cm,
    path_to_commands=path_to_commands,
    start_sequence=_start_sequence_wrapper,
    get_initial_hd=get_initial_hd,
)

# 마우스 콜백(수동 모드일 때는 수동 핸들러로 보냄)
def unified_mouse(event, x, y, flags, param):
    # 1) 수동 모드면 기존처럼 수동 핸들러로
    if manual.is_manual_mode():
        manual.on_mouse(event, x, y)
        return

    # 2) Goal Align 모드: 좌클릭으로 지정 셀로 회전+이동 퍼블리시
    if GOAL_ALIGN_MODE and event == cv2.EVENT_LBUTTONDOWN:
        try:
            # 화면 좌표 → 셀(row,col) : 기존 mouse_event와 동일 계산
            row, col = y // cell_size, x // cell_size
            if not (0 <= row < grid_row and 0 <= col < grid_col):
                return

            # 선택된 로봇이 있어야 함 (숫자키로 채우는 SELECTED_RIDS 사용)
            if not SELECTED_RIDS:
                print("[GoalAlign] 선택된 로봇이 없습니다. 숫자키(1~9)로 선택하세요.")
                return

            # 목표 맵 구성: {tag_id: (row,col)}
            goals = {int(rid): (row, col) for rid in SELECTED_RIDS}
            controller.register_step_goals_for_current(goals)

            # 퍼블리시 (회전 modeOnly → 이동 modeC)
            from controller.align import send_goal_align
            send_goal_align(client, tag_info, MQTT_TOPIC_COMMANDS_, vision, goals, alignment_pending=None)
            print(f"[GoalAlign] ({row},{col}) ← {sorted(SELECTED_RIDS)} 로 전송 완료")
        except Exception as e:
            print(f"[GoalAlign error] {e}")
        return  # Goal Align 처리 후 여기서 종료

    # 3) 평소처럼 CBS용 마우스 콜백으로
    mouse_event(event, x, y, flags, param)



def compute_cbs():
    global paths, pathfinder, grid_array

    # 0) 준비된/대기 에이전트 분리
    ready_agents = [a for a in agents if a.start and a.goal]   # 경로계산 대상
    waiters      = [a for a in agents if a.start and not a.goal]  # 장애물로 쓸 대기자

    if not ready_agents:
        print("⚠️ start·goal이 모두 지정된 에이전트를 찾을 수 없습니다.")
        return

    # 1) '대기자'를 장애물로 올린 그리드 만들기
    aug_grid = grid_array.copy()
    for w in waiters:
        try:
            r, c = w.start
            if 0 <= r < grid_row and 0 <= c < grid_col:
                aug_grid[r, c] = 1  # 점유(장애물)로 마킹
        except Exception:
            pass

    # 2) PathFinder는 매번 최신 그리드(aug_grid)로 생성
    pathfinder_local = PathFinder(aug_grid)

    # 3) 계산 및 결과 반영
    solved_agents = pathfinder_local.compute_paths(ready_agents)
    valid_agents = [a for a in solved_agents if a.get_final_path()]

    if not valid_agents:
        print("No solution found.")
        return

    paths.clear()
    paths.extend([a.get_final_path() for a in valid_agents])
    print("Paths updated via PathFinder (waiters treated as obstacles).")

    # 4) 하드웨어 명령 제작 + 전송
    payload_commands = []
    step_cell_plan: dict[int, dict[str, dict]] = {}  # {step_idx: {rid: {"src":(r,c), "dst":(r,c)}}}
    for agent in valid_agents:
        raw_path = agent.get_final_path()
        hd0 = get_initial_hd(agent.id)
        cmd_objs = path_to_commands(raw_path, hd0)
        command_set = [c["command"] for c in cmd_objs]
        payload_commands.append({
            "robot_id": str(agent.id),
            "command_count": len(command_set),
            "command_set": command_set
        })

        # --- 스텝별 src/dst 셀 기록 ---
        # raw_path는 [(r0,c0), (r1,c1), ...] 형태라고 가정
        # i번째 MOVE의 src=raw_path[i], dst=raw_path[i+1]
        for i in range(len(raw_path)-1):
            step_cell_plan.setdefault(i, {})
            step_cell_plan[i][str(agent.id)] = {
                "src": tuple(raw_path[i]),
                "dst": tuple(raw_path[i+1]),
            }

    cmd_map = {p["robot_id"]: p["command_set"] for p in payload_commands}
    print("▶ 순차 전송 시작:", cmd_map)
    # 🔗 yield 판정을 위해 셀 계획을 함께 전달
    controller.start_sequence(cmd_map, step_cell_plan=step_cell_plan)


#정지 함수
def send_emergency_stop(client):
    print("!! Emergency Stop 명령 전송: 'S' to robots 1~4")
    for rid in range(1, 5):
        topic = f"robot/{rid}/cmd"
        client.publish(topic, "S")
        print(f"  → Published to {topic}")
        
# #정지 해제 함수        
def send_release_all(client, ids):
    for rid in ids:
        client.publish(f"robot/{rid}/cmd", "RE")
        print(f"▶ [Robot_{rid}] RE 전송")

#즉시 모터 정지 함수
def immediate_stop(client, ids):
    """선택된 로봇(들)에게 즉시 정지 im_S 전송"""
    for rid in ids:
        client.publish(f"robot/{rid}/cmd", "im_S")
        print(f"🛑 [Robot_{rid}] 즉시정지(im_S) 전송")
        

def main():
    #초기 설정
    global agents, paths, grid_array, tag_info, selected_robot_id, PRESET_IDS

    #그리드 불러오기
    grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)
    slider_create()
    detect_params = slider_value()

    # 슬라이더 생성
    slider_create()
    detect_params = slider_value()  # 슬라이더에서 받아오기


    if SHOW_CV_WINDOWS:
        cv2.namedWindow("Video_display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Video_display", vision.mouse_callback)
        cv2.namedWindow("CBS Grid")
        cv2.setMouseCallback("CBS Grid", unified_mouse)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 획득 실패")
            continue
        
        # 1) 프레임 처리
        visionOutput = vision.process_frame(frame, detect_params)
        if visionOutput is None:
            continue
        ob_grid = vision.get_obstacle_grid()
        if ob_grid is not None:
            grid_array = ob_grid.copy()

        vis = grid_visual(grid_array.copy())

        # 2) 새 프레임 기반으로 화면/태그 정보 먼저 갱신
        frame = visionOutput["frame"]
        tag_info = visionOutput["tag_info"]
        controller.set_tag_info_provider(lambda: tag_info)

        # 3) 새 tag_info로 PRESET_IDS 갱신 (리스트 객체 유지)
        _prev = PRESET_IDS[:]                           # 이전 목록 백업
        new_ids = compute_visible_robot_ids(tag_info)   # 반드시 최신 tag_info 기반
        PRESET_IDS[:] = new_ids
        
        guard.tick(tag_info, PRESET_IDS)
        rm.tick(tag_info, PRESET_IDS)
        visible_set = set(PRESET_IDS)
        agents[:] = [a for a in agents if a.id in visible_set]

        if any("grid_position" in data for data in visionOutput["tag_info"].values()):
            update_agents_from_tags(visionOutput["tag_info"])
        
        
        # UI 연동
        # =============================
        # =============================
        FrameBus.set_video(frame)
        FrameBus.set_grid(vis)

        
        # UI 명령 처리 (버튼 클릭, 키 입력 등)
        # =============================
        # =============================
        cmd, kwargs = get_cmd_nowait()
        if cmd == "select_robot":                 # (로봇 선택 버튼, 숫자키 1~4와 유사 동작)
            rid = int(kwargs["rid"])
            SELECTED_RIDS.clear()
            SELECTED_RIDS.add(rid)
            print(f"[UI] 선택 로봇 동기화 → {sorted(SELECTED_RIDS)}")

        elif cmd == "compute_cbs":                # 키: 'c'
            compute_cbs()

        elif cmd == "lock_board":                 # 키: 'n'
            vision.lock_board()
            print("[UI] 보드 고정됨")

        elif cmd == "unlock_board":               # 키: 'b'
            vision.reset_board()
            print("[UI] 보드 고정 해제")

        elif cmd == "toggle_visualization":       # 키: 'v'
            vision.toggle_visualization()
            print(f"[UI] 시각화 모드: {'ON' if vision.visualize else 'OFF'}")

        elif cmd == "start_roi_selection":        # 키: 's'
            vision.start_roi_selection()
            print("[UI] ROI 재선택 시작")

        elif cmd == "center_align":               # 키: 'a'
            send_release_all(client, PRESET_IDS)
            controller.run_center_align(PRESET_IDS, do_release=False)
            print("[UI] 센터 정렬 전송")

        elif cmd == "direction_align":            # 키: 'f'
            send_release_all(client, PRESET_IDS)
            controller.run_direction_align(PRESET_IDS, do_release=False)
            print("[UI] 방향 정렬 전송")

        elif cmd == "pause":                      # 키: 't'
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                controller.pause([str(r) for r in targets])
                print(f"[UI] 정지: {targets}")
            else:
                print("[UI] 정지 대상 없음")

        elif cmd == "resume":                     # (키: 기본 없음, 과거 'y'와 유사 동작)
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                controller.resume([str(r) for r in targets])
                print(f"[UI] 재개: {targets}")
            else:
                print("[UI] 재개 대상 없음")

        elif cmd == "immediate_stop":             # 키: 'u'
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                immediate_stop(client, targets)
                print(f"[UI] 즉시정지: {targets}")
            else:
                print("[UI] 즉시정지 대상 없음")

        elif cmd == "save_grid":                  # 키: 'g'
            saved = None
            if vision.obstacle_detector is not None and vision.obstacle_detector.last_occupancy is not None:
                saved = vision.obstacle_detector.save_grid(save_dir=GRID_FOLDER)
            print(f"[UI] Grid 저장: {saved}" if saved else "[UI] 저장할 Grid 없음")

        elif cmd == "reset_all":                  # 키: 'r'
            agents.clear()
            paths.clear()
            manual.reset_paths()
            print("[UI] Reset all")

        elif cmd == "manual_toggle":              # 키: 'z'
            manual.toggle_mode()
            print(f"[UI] 수동 모드: {'ON' if manual.is_manual_mode() else 'OFF'}")

        elif cmd == "goalalign_toggle":           # 키: 'h'
            GOAL_ALIGN_MODE = not GOAL_ALIGN_MODE
            print(f"[UI][GoalAlign] {'ON' if GOAL_ALIGN_MODE else 'OFF'}")

        elif cmd == "quit":                       # 키: 'q'
            raise SystemExit("[UI] Quit 요청")

        elif cmd == "set_goal":                   # (그리드 클릭, 키 없음)
            rid = int(kwargs["rid"])
            row = int(kwargs["row"])
            col = int(kwargs["col"])
            tgt = next((a for a in agents if a.id == rid), None)
            if tgt is None:
                print(f"[UI] set_goal 실패: 에이전트 {rid} 없음")
            else:
                tgt.goal = (row, col)
                print(f"[UI] 로봇 {rid} 목표=({row},{col}) 설정")

        # =============================
        # =============================


        # UI 시각화 화면
        draw_paths(vis, paths)
        draw_agent_points(vis, agents)
        manual.draw_overlay(vis) # ← 수동 경로 오버레이

        if SHOW_CV_WINDOWS:
            cv2.imshow("CBS Grid", vis)
            cv2.imshow("Video_display", frame)
            key = cv2.waitKey(1)
        else:
            try:
                key = _KEYQ.get_nowait()
            except Empty:
                key = -1

        if key == ord('q'):  # 'q' 키 -> 종료 (저장 없이)
            break
        elif key == ord('r'):
            print("Reset all")
            agents.clear()
            paths.clear()
            manual.reset_paths()  # ← 수동 경로만 초기화 추가
        
        elif key == ord('h'):
            GOAL_ALIGN_MODE = not GOAL_ALIGN_MODE
            print(f"[GoalAlign] {'ON' if GOAL_ALIGN_MODE else 'OFF'} — 좌클릭으로 목표 셀을 지정합니다.")

        
        elif key == ord('c'):
            if manual.is_manual_mode():
                # 수동 경로 전송(선택된 로봇의 수동 경로를 command로 변환한 뒤 전송)
                manual.commit()
            else:
                send_release_all(client, PRESET_IDS)
                compute_cbs()
        elif key == ord('n'):
            vision.lock_board()
            print("보드 고정됨")
        elif key == ord('b'):
            vision.reset_board()
            print("🔄 고정된 보드를 해제")
        elif key == ord('v'):
            vision.toggle_visualization()
            print(f"시각화 모드: {'ON' if vision.visualize else 'OFF'}")
        elif key == ord('s'):
            vision.start_roi_selection()
        
        elif key == ord('g'):
            saved = None
            if vision.obstacle_detector is not None and vision.obstacle_detector.last_occupancy is not None:
                saved = vision.obstacle_detector.save_grid(save_dir=GRID_FOLDER)
            print(f"Saved: {saved}" if saved else "No grid to save yet")
        
        # elif key == ord('x'):
        #     send_release_all(client, PRESET_IDS)
        #     controller.run_north_align(PRESET_IDS, do_release=False)

        elif key == ord('f'):
            send_release_all(client, PRESET_IDS)
            controller.run_direction_align(PRESET_IDS, do_release=False)


        elif key == ord('a'):
            send_release_all(client, PRESET_IDS)
            controller.run_center_align(PRESET_IDS, do_release=False)

        # 숫자키로 대상 선택/토글 (예: 1~4)
        elif key in tuple(ord(str(i)) for i in range(1, 10)):
            rid = int(chr(key))
            if rid in SELECTED_RIDS:
                SELECTED_RIDS.remove(rid)
                print(f"[-] 선택 해제: {rid} / 현재 선택: {sorted(SELECTED_RIDS)}")
            else:
                SELECTED_RIDS.add(rid)
                print(f"[+] 선택 추가: {rid} / 현재 선택: {sorted(SELECTED_RIDS)}")

            selected_robot_id = rid
            print(f"🎯 목표지정 대상 로봇: {selected_robot_id}")
        # 선택 로봇 정지 (그냥 누르면 전체 정지)
        elif key == ord('t'):
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                controller.pause([str(r) for r in targets])
            else:
                print("⚠️ 정지할 접속 로봇이 없습니다.")

        # elif key == ord('y'):
        #     targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
        #     if targets:
        #         controller.resume([str(r) for r in targets]) 
        #         for r in targets:
        #             PROXIMITY_STOP_LATCH.discard(int(r))
        #     else:
        #         print("⚠️ 재개할 대상이 없습니다.")


        # elif key == ord('d'):
        #     send_release_all(client, PRESET_IDS)
        #     controller.run_center_align(PRESET_IDS, do_release=False)
        #     controller.run_direction_align(PRESET_IDS, do_release=False)
        #     compute_cbs()

            
        elif key in (ord('u'), ord('U')):  # 숫자 선택 후 U → 선택 대상 즉시 정지
            if SELECTED_RIDS:
                immediate_stop(client, sorted(SELECTED_RIDS))
            else:
                # 선택이 없으면 현재 화면에 잡힌 모든 로봇 즉시 정지
                if PRESET_IDS:
                    immediate_stop(client, PRESET_IDS)
                    print(f"🛑 모든 접속 로봇 즉시 정지(im_S): {PRESET_IDS}")
                else:
                    print("⚠️ 즉시 정지할 접속 로봇이 없습니다.")
        
        elif key == ord('z'):
            manual.toggle_mode()  # ← 수동 모드 토글
        

    cap.release()
    if SHOW_CV_WINDOWS:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    finally:
        if proc is not None:
            proc.terminate()
            proc.wait()
            print("▶ command_transfer.py 프로세스 종료됨")
