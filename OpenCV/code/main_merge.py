import sys
import os
import cv2
import numpy as np
import subprocess 
import math
import time
from queue import Queue, Empty
import threading  # ◀◀◀ [추가]
import copy

# UI 연동 관련

SHOW_CV_WINDOWS = bool(int(os.environ.get("SHOW_CV_WINDOWS", "1")))

_KEYQ: "Queue[int]" = Queue()

def push_keycode(code: int):
    """외부(Kivy)에서 보낸 가상 키코드를 백엔드에 전달"""
    _KEYQ.put(code)


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))


from OpenCV.code.grid import load_grid, GRID_FOLDER
from OpenCV.code.interface import grid_visual, slider_create, slider_value, draw_agent_points, draw_paths,draw_home_positions
from OpenCV.code.config import grid_row, grid_col, cell_size, camera_cfg, IP_address_, MQTT_TOPIC_COMMANDS_ , MQTT_PORT , NORTH_TAG_ID, CORRECTION_COEF, critical_dist 
from OpenCV.code.vision.visionsystem_mjk import VisionSystem
from OpenCV.code.vision.camera import camera_open, Undistorter 
from OpenCV.code.cbs.pathfinder import PathFinder, Agent
from OpenCV.code.RobotController_merge import RobotController
from OpenCV.code.config import cell_size_cm
from OpenCV.code.manual_mode import ManualPathSystem
from OpenCV.code.recieve_message import set_tag_info_provider
from OpenCV.code.ScenarioManager import ScenarioManager
from OpenCV.code.TestMode import TestMode
from OpenCV.code.RandomMode import RandomMode
from OpenCV.code.RestaurantMode import RestaurantMode  
from OpenCV.code.ui_bridge import FrameBus, get_cmd_nowait

SELECTED_RIDS = set()

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CTS_SCRIPT = os.path.join(SCRIPT_DIR, "command_transfer.py") #별도의 창으로 command_transfer 실행

# 메인 로직이 실행되기 전에 커맨드 전송 스크립트를 백그라운드로 시작
# sys.executable: 현재 사용 중인 파이썬 인터프리터 경로
cts_process = subprocess.Popen([sys.executable, CTS_SCRIPT],creationflags=subprocess.CREATE_NEW_CONSOLE)
print(f"▶ command_transfer_encoderSelf.py 별도 콘솔에서 실행: {CTS_SCRIPT}")

# 로봇 home지정
ROBOT_HOME_POSITIONS = {
    # 형식: 로봇ID: (행, 열)
    1: (0, 0),
    2: (2, 0),
    3: (0, 5), 
}

MODE_FACTORY = {
    "test": lambda: TestMode(),
    "restaurant": lambda: RestaurantMode(
        home_provider=lambda rid: ROBOT_HOME_POSITIONS.get(rid),
        order_span_sec=(0, 30),
    ),
}
_mode_keys = list(MODE_FACTORY.keys())
_mode_idx = [0]  # 가변 캡쳐용(리스트)

# CBS 설정
SOLVER_CHOICES = ["CBS", "ICBS_CB", "ICBS"]
_SOLVER_IDX = 0   # 0:CBS, 1:ICBS_CB, 2:ICBS  (기본 CBS)
DISJOINT = True

def current_solver() -> str:
    return SOLVER_CHOICES[_SOLVER_IDX]

# 브로커 정보
# main.py 상단에 USE_MQTT 정의
USE_MQTT = 0 # 0: 비사용, 1: 사용

if USE_MQTT:
    from OpenCV.code.recieve_message import init_mqtt_client
    client = init_mqtt_client()   # ← recieve_message의 '그' 클라이언트 단일 사용
else:
    MQTT_TOPIC_COMMANDS_ = None
    class _DummyClient:
        def publish(self, topic, payload):
            print(f"[MQTT_DISABLED] publish → topic={topic}, payload={payload}")
    client = _DummyClient()

controller = RobotController(
    client=client,
    mqtt_topic_commands=MQTT_TOPIC_COMMANDS_,
    grid_col=grid_col, # <<< [추가] grid_col 전달
    done_topic="robot/done",
    north_tag_id=NORTH_TAG_ID,
    direction_corr_threshold_deg=3.0,
    alignment_delay_sec=0.8,
    alignment_angle=1.0,
    alignment_dist=1.0,
)

if USE_MQTT:
    def _on_msg(c, u, m):  # ◀◀◀ [수정 시작]
            try:
                topic = m.topic
                payload_str = m.payload.decode("utf-8", "ignore")

                # 1. 컨트롤러가 처리할 'DONE' 메시지
                if topic == controller.done_topic:
                    controller.on_mqtt_message(topic, payload_str)
                
                # 2. 새로 추가된 'STATUS' 메시지 (예: "robot/3/status")
                elif topic.endswith("/status"):
                    # (페이로드 예: "STATUS;Robot_3;msg=QueueCleared")
                    if "QueueCleared" in payload_str:
                        print(f"✅ [Robot Status] 로봇 큐가 비워졌습니다! ({payload_str})")
                    elif "QueueEmpty" in payload_str:
                        print(f"ℹ️ [Robot Status] 로봇이 연결되었으며, 큐가 비어있습니다. ({payload_str})")
                    elif "QueueNotEmpty" in payload_str:
                        # '유지된 메시지'가 있었다는 뜻입니다.
                        print(f"⚠️ [Robot Status] 로봇 연결됨. 큐가 비어있지 않습니다! ({payload_str})")
                    else:
                        print(f"[Robot Status] {topic}: {payload_str}")
                
                # 3. 그 외 (필요시)
                # else:
                #    print(f"[MQTT Recv] {topic}: {payload_str}")

            except Exception as e:
                print(f"[on_message error] {e}")

    client.on_message = _on_msg

    try:
        client.subscribe(controller.done_topic)
        # client.loop_start()  # init_mqtt_client 안에서 이미 실행 중이면 생략
        status_topic = "robot/+/status"  # ◀◀◀ [추가]
        client.subscribe(status_topic)   # ◀◀◀ [추가]
        print(f"▶ MQTT 구독: {controller.done_topic}, {status_topic}") # ◀◀◀ [추가]
    except Exception:
        pass

correction_coef_value = CORRECTION_COEF

def correction_trackbar_callback(val):
    global correction_coef_value
    correction_coef_value = val / 100.0
    print(f"[INFO] 실시간 보정계수: {correction_coef_value:.2f}")

cv2.namedWindow("CorrectionPanel", cv2.WINDOW_NORMAL)
cv2.createTrackbar(
    "Correction Coef", "CorrectionPanel",
    int(CORRECTION_COEF * 100), 200, correction_trackbar_callback
)


# 전역 변수

#근접 시 즉시 정지 기능
PROXIMITY_GUARD_ENABLED = True   # 끄려면 False
PROXIMITY_STOP_LATCH = set()     # 이미 proximity로 im_S 보낸 로봇 ID(int)

grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)
agents = []
paths = []
pathfinder = None
grid_array = None
visualize = True
# tag_info 전역 변수 초기화

def get_tag_info_safe():  # ◀◀◀ [추가 시작]
    """
    스레드 충돌을 방지하며 tag_info의 '깊은 복사본(deepcopy)'을 반환합니다.
    (deepcopy를 사용하면, 컨트롤러가 데이터를 읽는 도중에 
     메인 스레드가 원본을 수정해도 컨트롤러가 가진 데이터는 안전합니다.)
    """
    with TAG_INFO_LOCK:
        return copy.deepcopy(tag_info)

tag_info = {}
set_tag_info_provider(get_tag_info_safe)

# 비전 시스템 초기화
#video_path = r"C:/img/test2.mp4"
cap, fps = camera_open(source=None)

undistorter = Undistorter(
    camera_cfg['type'],
    camera_cfg['matrix'],
    camera_cfg['dist'],
    camera_cfg['size']
)
vision = VisionSystem(undistorter=undistorter, visualize=True)
vision.correction_coef_getter = lambda: correction_coef_value

# 로봇 ID 관련
PRESET_IDS = []
selected_robot_id = None

TAG_INFO_LOCK = threading.Lock()


def compute_visible_robot_ids(tag_info: dict) -> list[int]:
    """카메라에 잡힌 '로봇' 태그 ID를 정렬 리스트로 반환 (보드/NORTH_TAG_ID 제외)."""
    visible = []
    for tid, data in tag_info.items():
        # tid는 정수, 'On' 상태, 보드 태그는 제외
        if isinstance(tid, int) and data.get("status") == "On" and tid != NORTH_TAG_ID:
            visible.append(tid)
    visible.sort()
    return visible


def show_orders_text_panel(ui_state: dict):
    import numpy as np, cv2
    H, W = 400, 360
    PAD_X = 10
    LINE_SP = 26

    img = np.full((H, W, 3), 255, np.uint8)

    # --- 상단: HOME만 표시 (대괄호 제거, 숫자만) ---
    home_set = sorted(ui_state.get("home_set", []))
    home_str = " ".join(str(x) for x in home_set) if home_set else ""
    y = 22
    cv2.putText(img, f"Home: {home_str}", (PAD_X, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2, cv2.LINE_AA)

    # --- 중앙 제목: Orders (가운데 정렬 + 큰 글씨) ---
    title = "Orders"
    (tw, th), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 3)
    y = 22 + LINE_SP + 8
    tx = (W - tw) // 2
    cv2.putText(img, title, (tx, y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,0), 3, cv2.LINE_AA)

    # --- 제목 아래 가로줄 ---
    y_line = y + 10
    cv2.line(img, (PAD_X, y_line), (W - PAD_X, y_line), (0,0,0), 2)

    # --- 주문 목록: '진행 중'은 굵게, 대기중은 일반 ---
    y = y_line + 20
    # 진행 중(테이블 가는 중/홈 복귀 중)
    active_to_table = set((int(r), tuple(dst)) for (r, dst) in ui_state.get("order_to_table", []))
    active_to_home  = set((int(r), tuple(dst)) for (r, dst) in ui_state.get("order_to_home", []))
    active = active_to_table | active_to_home

    # 대기 중 큐
    pending = [(int(r), tuple(dst)) for (r, dst) in ui_state.get("order_list", [])]

    # 화면에 보여줄 최종 목록: 진행중 먼저, 그 다음 대기
    render_list = list(active) + pending

    if not render_list:
        cv2.putText(img, "(no orders)", (PAD_X, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128,128,128), 2, cv2.LINE_AA)
    else:
        shown = 0
        for (rid, dst) in render_list:
            txt = f"{rid} > {tuple(dst)}"
            # 진행중은 두껍게(굵게)
            is_active = (rid, dst) in active
            thickness = 3 if is_active else 2
            scale = 0.9 if is_active else 0.8
            cv2.putText(img, txt, (PAD_X, y),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thickness, cv2.LINE_AA)
            y += LINE_SP
            shown += 1
            if shown >= 14:  # 너무 많으면 컷
                break

    cv2.imshow("Orders", img)
    FrameBus.set_orders(img)



def _get_tag_cm(tag_info: dict, rid: int):
    d = tag_info.get(rid, {})
    if d.get("status") == "On" and "corrected_center" in d:
        return d["corrected_center"]  # (X_cm, Y_cm)
    return None

def compute_pairwise_distances_cm(tag_info: dict, ids: list[int]):
    """ids 목록에서 보이는 태그들 간의 모든 쌍 거리를 cm로 반환"""
    pairs = []
    for i, a in enumerate(ids):
        pa = _get_tag_cm(tag_info, a)
        if not pa:
            continue
        for b in ids[i+1:]:
            pb = _get_tag_cm(tag_info, b)
            if not pb:
                continue
            dx = pa[0] - pb[0]
            dy = pa[1] - pb[1]
            dist = math.hypot(dx, dy)
            pairs.append(((a, b), dist))
    return pairs

def proximity_guard(tag_info: dict, ids: list[int], threshold_cm: float):
    """
    ids에 대해 임계거리 이하 쌍이 하나라도 연결된 '충돌 클러스터'를 찾고,
    그 클러스터에 속한 모든 로봇 집합(to_stop)과 트리거 페어 목록을 반환.
    """
    pairs = compute_pairwise_distances_cm(tag_info, ids)
    adj = {rid: set() for rid in ids}
    trigger_pairs = []
    for (a, b), dist in pairs:
        if dist <= threshold_cm:
            adj[a].add(b); adj[b].add(a)
            trigger_pairs.append(((a, b), dist))

    to_stop = set()
    visited = set()
    for rid in ids:
        if rid in visited:
            continue
        # DFS로 연결 성분(클러스터) 추출
        stack = [rid]
        comp = []
        while stack:
            u = stack.pop()
            if u in visited:
                continue
            visited.add(u)
            comp.append(u)
            stack.extend(v for v in adj[u] if v not in visited)
        if len(comp) >= 2:        # 2대 이상 연결 → 충돌 클러스터
            to_stop.update(comp)

    return to_stop, trigger_pairs

# ---------------------------
# 기존 우클릭 목표 지정 핸들러 (수동모드 아닐 때만 사용)
# ---------------------------
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

def handle_number_key_unified(key):
    """
    숫자키(1~9) 공통 핸들러:
      1) 숫자키 이벤트를 ScenarioManager로 라우팅
      2) 기존 UI 선택/토글 및 selected_robot_id 갱신
    """
    global SELECTED_RIDS, selected_robot_id
    rid = int(chr(key))

    # 1) 모드에 필요한 행동은 매니저가 수행(CBS/정렬 포함)
    #scenario.on_number_key(rid)

    # 2) 기존 UI 토글 유지
    if rid in SELECTED_RIDS:
        SELECTED_RIDS.remove(rid)
        print(f"[-] 선택 해제: {rid} / 현재 선택: {sorted(SELECTED_RIDS)}")
    else:
        SELECTED_RIDS.add(rid)
        print(f"[+] 선택 추가: {rid} / 현재 선택: {sorted(SELECTED_RIDS)}")

    selected_robot_id = rid
    print(f"🎯 목표지정 대상 로봇: {selected_robot_id}")


# ---------------------------
# 수동 경로 시스템 연결부
# ---------------------------
YAW_TO_NORTH_OFFSET_DEG = 0

def yaw_to_hd(yaw_deg: float, offset_deg: float = 0) -> int:
    ang = (yaw_deg + offset_deg) % 360.0
    return int(((ang + 45.0) // 90.0) % 4)

def get_initial_hd(robot_id: int) -> int:
    data = tag_info.get(robot_id)
    if not data or data.get('status') != 'On':
        return 0
    delta = data.get("heading_offset_deg")
    if delta is None:
        return 0
    yaw_deg = (data.get("yaw_front_deg", 0) + 360) % 360
    direction_angles = [90, 0, 270, 180]  # N=90, W=0, S=270, E=180
    diffs = [abs(((yaw_deg - a + 180) % 360) - 180) for a in direction_angles]
    min_idx = diffs.index(min(diffs))
    hd = [0, 3, 2, 1][min_idx]  # N=0, E=1, S=2, W=3
    return hd

def path_to_commands(path, init_hd=0):
    cmds = []
    hd = init_hd
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if r0 == r1 and c0 == c1:
            cmds.append({'command': 'Stay'})
            continue
        if   r1 < r0:  desired = 0  # 북
        elif c1 > c0:  desired = 1  # 동
        elif r1 > r0:  desired = 2  # 남
        else:          desired = 3  # 서
        diff = (desired - hd) % 4
        if diff == 0:
            cmds.append({'command': f'F{cell_size_cm:.1f}_modeA'})
        elif diff == 1:
            cmds.append({'command': 'R90'})
        elif diff == 2:
            cmds.append({'command': 'T185'})  # 180도 보정치
        else:
            cmds.append({'command': 'L90'})
        hd = desired
    return cmds

# controller.start_sequence를 수동 시스템에 전달하기 위한 래퍼
def _start_sequence_wrapper(cmd_map: dict):
    # 수동 경로는 step_cell_plan이 없어도 동작하도록 간단 호출
    controller.start_sequence(cmd_map)

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

# ▶ 새로운 시나리오 매니저 + 플러그형 모드(TestMode)
scenario = ScenarioManager(
    controller=controller,
    agents_ref=agents,
    paths_ref=paths,
    get_grid=lambda: grid_array,
    get_tag_info=get_tag_info_safe,
    path_to_commands=path_to_commands,
    get_initial_hd=get_initial_hd,
    mode=TestMode()  # 필요시 다른 모드로 교체
)


# 마우스 콜백(수동 모드일 때는 수동 핸들러로 보냄)
def unified_mouse(event, x, y, flags, param):
    if manual.is_manual_mode():
        manual.on_mouse(event, x, y)
    else:
        mouse_event(event, x, y, flags, param)


# 태그를 통해 에이전트 업데이트 (cm → 셀 좌표)
def update_agents_from_tags(tag_info):
    for tag_id, data in tag_info.items():
        if tag_id not in PRESET_IDS:
            continue
        if data.get("status") != "On":
            continue
        start_cell = data["grid_position"]
        existing = next((a for a in agents if a.id == tag_id), None)
        if existing:
            if existing.start == start_cell:
                continue
            existing.start = start_cell
        else:
            agents.append(Agent(id=tag_id, start=start_cell, goal=None, delay=0))


# ---------------------------
# CBS 실행(컨트롤러 유지)
# ---------------------------
def compute_cbs():
    global paths, pathfinder, grid_array

    # 0) 준비된/대기 에이전트 분리
    ready_agents = [a for a in agents if a.start and a.goal]
    waiters      = [a for a in agents if a.start and not a.goal]
    if not ready_agents:
        print("⚠️ start·goal이 모두 지정된 에이전트를 찾을 수 없습니다.")
        return

    # 1) 대기자를 장애물로 올린 그리드
    aug_grid = grid_array.copy()
    for w in waiters:
        try:
            r, c = w.start
            if 0 <= r < grid_row and 0 <= c < grid_col:
                aug_grid[r, c] = 1
        except Exception:
            pass

    # 2) PathFinder는 매번 최신 그리드로 생성
    pathfinder_local = PathFinder(aug_grid, 
                                  solver_type=current_solver(),
                                  disjoint=DISJOINT,
                                  visualize_result=False)

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
    step_cell_plan: dict[int, dict[str, dict]] = {}
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
        for i in range(len(raw_path)-1):
            step_cell_plan.setdefault(i, {})
            step_cell_plan[i][str(agent.id)] = {
                "src": tuple(raw_path[i]),
                "dst": tuple(raw_path[i+1]),
            }

    cmd_map = {p["robot_id"]: p["command_set"] for p in payload_commands}
    print("▶ 순차 전송 시작:", cmd_map)
    controller.start_sequence(cmd_map, step_cell_plan=step_cell_plan)


# ---------------------------
# 유틸: 정지/재개/즉시정지
# ---------------------------
def send_emergency_stop(client):
    print("!! Emergency Stop 명령 전송: 'S' to robots 1~4")
    for rid in range(1, 5):
        topic = f"robot/{rid}/cmd"
        client.publish(topic, "S")
        print(f"  → Published to {topic}")
        
#정지 해제 함수        
def send_release_all(client, ids):
    for rid in ids:
        client.publish(f"robot/{rid}/cmd", "RE")
        print(f"▶ [Robot_{rid}] RE 전송")

#즉시 모터 정지 함수
def immediate_stop(client, ids):
    for rid in ids:
        client.publish(f"robot/{rid}/cmd", "im_S")
        print(f"🛑 [Robot_{rid}] 즉시정지(im_S) 전송")
        
def main():
    global agents, paths, visualize, tag_info, grid_array, selected_robot_id, _SOLVER_IDX, DISJOINT

    # 그리드 불러오기(비전 결과로 대체되기 전까지 0으로 시작)
    grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)

    # 슬라이더 생성
    slider_create()
    detect_params = slider_value()
    
    # UI 맞게 수정
    #=========================================
    #=========================================
    
    if SHOW_CV_WINDOWS:
        cv2.namedWindow("Video_display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Video_display", vision.mouse_callback)
        cv2.namedWindow("CBS Grid", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("CBS Grid", unified_mouse)  # ← 수동 모드 대응
        controller.set_board_info_provider(lambda: vision.board_result) # <<< [추가]

    while True:
        path_viz_data = controller.get_current_step_info()
        ret, frame = cap.read()
        if not ret:
            print("프레임 획득 실패")
            continue

        # 1) 프레임 처리
        visionOutput = vision.process_frame(frame, detect_params, path_viz_data=path_viz_data)
        if visionOutput is None:
            continue
        ob_grid = vision.get_obstacle_grid()
        if ob_grid is not None:
            grid_array = ob_grid.copy()
        
        vis = grid_visual(grid_array.copy())
        draw_home_positions(vis, ROBOT_HOME_POSITIONS) 

        # 2) 새 프레임 기반으로 화면/태그 정보 먼저 갱신
        frame = visionOutput["frame"]
        with TAG_INFO_LOCK:  # ◀◀◀ [추가] (쓰기 보호)
            tag_info = visionOutput["tag_info"]
        controller.set_tag_info_provider(lambda: tag_info)
        

        # 3) 새 tag_info로 PRESET_IDS 갱신
        _prev = PRESET_IDS[:]
        current_tags_safe = get_tag_info_safe()
        new_ids = compute_visible_robot_ids(tag_info)
        PRESET_IDS[:] = new_ids
        scenario.tick()
        
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
        if cmd == "select_robot":                 # (UI 숫자 버튼 → 숫자키 동일 처리)
            rid = int(kwargs["rid"])
            fake_key = ord(str(rid))                # 숫자키 코드로 변환 (예: 1 → 49)
            handle_number_key_unified(fake_key)     # 🔹 숫자키와 동일한 로직 수행

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

        elif cmd == "quit":                       # 키: 'q'
            raise SystemExit("[UI] Quit 요청")

        elif cmd == "toggle_scenario_mode":          # 키: 'm'
            _mode_idx[0] = (_mode_idx[0] + 1) % len(_mode_keys)
            name = _mode_keys[_mode_idx[0]]
            scenario.set_mode(MODE_FACTORY[name]())
            FrameBus.set_mode(name) # UI에 현재 모드명 전달
            print(f"[UI][Scenario] mode ← {name} (실행상태는 유지)")

        elif cmd == "toggle_scenario_run":           # 키: Spacebar
            scenario.toggle_enabled()
            print(f"[UI][Scenario] 실행 상태: {'ON' if scenario.enabled else 'OFF'}")

        elif cmd == "resume":                        # 키: 'y'
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                controller.resume([str(r) for r in targets])
                for r in targets:
                    PROXIMITY_STOP_LATCH.discard(int(r))
                print(f"[UI] 재개: {targets}")
            else:
                print("[UI] 재개 대상 없음")

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


        elif cmd == "auto_release_align_cbs":  # (UI 버튼용)
            # 기존 o키 기능 그대로 복제
            send_release_all(client, PRESET_IDS)
            ready_agents = [a for a in agents if a.start and a.goal]
            waiters = [a for a in agents if a.start and not a.goal]

            waiter_ids = [a.id for a in waiters]
            if waiter_ids:
                controller.run_align_sequence(waiter_ids, do_release=False)

            compute_cbs()
            print("[UI] 전체 Release + Align + CBS 실행 완료")

        elif cmd == "solver_next":
            _SOLVER_IDX = (_SOLVER_IDX + 1) % len(SOLVER_CHOICES)
            print(f"[UI][MAPF] solver_type = {current_solver()} (disjoint={DISJOINT})")

        elif cmd == "solver_prev":
            _SOLVER_IDX = (_SOLVER_IDX - 1) % len(SOLVER_CHOICES)
            print(f"[UI][MAPF] solver_type = {current_solver()} (disjoint={DISJOINT})")

        elif cmd == "toggle_disjoint":
            DISJOINT = not DISJOINT
            print(f"[UI][MAPF] disjoint = {DISJOINT}")

        # =============================
        # =============================

        # UI 시각화 화면
        draw_paths(vis, paths)
        draw_agent_points(vis, agents)
        manual.draw_overlay(vis)  # ← 수동 경로 오버레이
        ui_state = scenario.get_mode_ui_state(drain_new=True)
        
        if ui_state:
            show_orders_text_panel(ui_state)
             
        cv2.imshow("CBS Grid", vis)
        cv2.imshow("Video_display", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('r'):
            print("Reset all")
            agents.clear()
            paths.clear()
            manual.reset_paths()  # ← 수동 경로만 초기화 추가
        elif key == ord('c'):
            scenario.set_enabled(False)
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
        elif key == ord('f'):
            send_release_all(client, PRESET_IDS)
            controller.run_direction_align(PRESET_IDS, do_release=False)
        elif key == ord('a'):
            send_release_all(client, PRESET_IDS)
            controller.run_center_align(PRESET_IDS, do_release=False)
        # 숫자키로 대상 선택/토글 (예: 1~9)
        elif key in tuple(ord(str(i)) for i in range(1, 10)):
            handle_number_key_unified(key)

        # 선택 로봇 정지 (그냥 누르면 전체 정지)
        elif key == ord('t'):
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                controller.pause([str(r) for r in targets])
            else:
                print("⚠️ 정지할 접속 로봇이 없습니다.")
        elif key == ord('y'):
            targets = sorted(SELECTED_RIDS) if SELECTED_RIDS else list(PRESET_IDS)
            if targets:
                controller.resume([str(r) for r in targets]) 
                for r in targets:
                    PROXIMITY_STOP_LATCH.discard(int(r))
            else:
                print("⚠️ 재개할 대상이 없습니다.")
        elif key in (ord('u'), ord('U')):
            if SELECTED_RIDS:
                immediate_stop(client, sorted(SELECTED_RIDS))
            else:
                if PRESET_IDS:
                    immediate_stop(client, PRESET_IDS)
                    print(f"🛑 모든 접속 로봇 즉시 정지(im_S): {PRESET_IDS}")
                else:
                    print("⚠️ 즉시 정지할 접속 로봇이 없습니다.")

        elif key == ord('o'):
            # 1) 전체 RE
            send_release_all(client, PRESET_IDS)

            # 2) 대기(waiter)와 준비(ready) 분리
            ready_agents = [a for a in agents if a.start and a.goal]
            waiters      = [a for a in agents if a.start and not a.goal]

            waiter_ids = [a.id for a in waiters]
            if waiter_ids:
                # 3) 대기는 중앙정렬 → 방향정렬 병렬 실행
                #    RE는 이미 보냈으므로 do_release=False
                controller.run_align_sequence(waiter_ids, do_release=False)

            # 4) ready만 대상으로 CBS 경로 계산&송신
            #    compute_cbs()는 대기자를 장애물로 올려서 경로를 짬
            compute_cbs()

        # 수동 모드
        elif key == ord('z'):
            manual.toggle_mode() 
            
        # 랜덤 모드    
        elif key == ord('m'): 
            _mode_idx[0] = (_mode_idx[0] + 1) % len(_mode_keys)
            name = _mode_keys[_mode_idx[0]]
            scenario.set_mode(MODE_FACTORY[name]())
            FrameBus.set_mode(name) # UI에 현재 모드명 전달
            print(f"[Scenario] mode ← {name} (실행상태는 유지)")

        elif key == 32:  # Spacebar
            scenario.toggle_enabled()

        elif key == ord(']'):   # 옵션 다음으로
            _SOLVER_IDX = (_SOLVER_IDX + 1) % len(SOLVER_CHOICES)
            print(f"[MAPF] solver_type = {current_solver()}  (disjoint={DISJOINT})")
        elif key == ord('['):   # 옵션 이전으로
            _SOLVER_IDX = (_SOLVER_IDX - 1) % len(SOLVER_CHOICES)
            print(f"[MAPF] solver_type = {current_solver()}  (disjoint={DISJOINT})")
        elif key == ord('p'):  
            DISJOINT = not DISJOINT
            print(f"[MAPF] disjoint = {DISJOINT}")
        
        # elif key == ord('d'):
        #     if manual.is_manual_mode():
        #         print("ℹ️ 수동모드에서는 d(자동시퀀스) 비활성화. Z로 해제 후 사용하세요.")
        #     else:
        #         send_release_all(client, PRESET_IDS)
        #         start_auto_sequence(
        #             client, tag_info, PRESET_IDS, agents, MQTT_TOPIC_COMMANDS_, NORTH_TAG_ID,
        #             set_alignment_pending, alignment_pending,
        #             check_center_alignment_ok,            # 중앙정렬 판정
        #             check_direction_alignment_ok,         # 방향정렬 판정
        #             send_center_align,                    # 필요 시 마무리 중앙정렬 전송
        #             compute_cbs,                          # 경로계산/전송
        #             check_all_completed                   # 완료 확인
        #         )
    cts_process.terminate()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()