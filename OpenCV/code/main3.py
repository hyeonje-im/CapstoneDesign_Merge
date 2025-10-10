import sys
import os
import cv2
import numpy as np
import subprocess 
import math
import time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))


from grid import load_grid, GRID_FOLDER
from interface import draw_agent_delays_on_grid, grid_visual, slider_create, slider_value, draw_agent_points, draw_paths,draw_home_positions
from config import grid_row, grid_col, cell_size,cell_size_cm, camera_cfg, MQTT_TOPIC_COMMANDS_, NORTH_TAG_ID, CORRECTION_COEF
from vision.visionsystem import VisionSystem 
from vision.camera import camera_open, Undistorter 
from cbs.pathfinder import PathFinder, Agent
from RobotController import RobotController
from recieve_message import set_tag_info_provider    
from manual_mode import ManualPathSystem  # ← 수동모드 
from ScenarioManager import ScenarioManager
from TestMode import TestMode
from RandomMode import RandomMode

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
    4: (0, 0),
    2: (2, 0),
    3: (0, 5), 
}

MODE_FACTORY = {
    "test": lambda: TestMode(),
    "random": lambda: RandomMode(
        home_provider=lambda rid: ROBOT_HOME_POSITIONS.get(rid)
    ),
}
_mode_keys = list(MODE_FACTORY.keys())
_mode_idx = [0]  # 가변 캡쳐용(리스트)

# 브로커 정보
# main.py 상단에 USE_MQTT 정의
USE_MQTT = 1  # 0: 비사용, 1: 사용

if USE_MQTT:
    from recieve_message import init_mqtt_client
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
    done_topic="robot/done",
    north_tag_id=NORTH_TAG_ID,
    direction_corr_threshold_deg=3.0,
    alignment_delay_sec=0.8,
    alignment_angle=1.0,
    alignment_dist=1.0,
)

if USE_MQTT:
    def _on_msg(c, u, m):
        try:
            controller.on_mqtt_message(m.topic, m.payload)
        except Exception as e:
            print(f"[on_message error] {e}")

    client.on_message = _on_msg

    try:
        client.subscribe(controller.done_topic)
        # client.loop_start()  # init_mqtt_client 안에서 이미 실행 중이면 생략
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
tag_info = {}
set_tag_info_provider(lambda: tag_info)

# 비전 시스템 초기화
# video_path = r"C:/img/test2.mp4"
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
    get_tag_info=lambda: tag_info,
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
    global agents, paths, visualize, tag_info, grid_array, selected_robot_id

    # 그리드 불러오기(비전 결과로 대체되기 전까지 0으로 시작)
    grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)

    # 슬라이더 생성
    slider_create()
    detect_params = slider_value()

    cv2.namedWindow("Video_display", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Video_display", vision.mouse_callback)
    cv2.namedWindow("CBS Grid", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("CBS Grid", unified_mouse)  # ← 수동 모드 대응

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
        draw_home_positions(vis, ROBOT_HOME_POSITIONS)

        # 2) 새 프레임 기반으로 화면/태그 정보 먼저 갱신
        frame = visionOutput["frame"]
        tag_info = visionOutput["tag_info"]
        controller.set_tag_info_provider(lambda: tag_info)

        # 3) 새 tag_info로 PRESET_IDS q갱신
        _prev = PRESET_IDS[:]
        new_ids = compute_visible_robot_ids(tag_info)
        PRESET_IDS[:] = new_ids
        
        scenario.tick()
        if any("grid_position" in data for data in visionOutput["tag_info"].values()):
            update_agents_from_tags(visionOutput["tag_info"])

        # UI 시각화 화면
        draw_paths(vis, paths)
        draw_agent_points(vis, agents)
        
        from interface import draw_agent_delays_on_grid
        draw_agent_delays_on_grid(vis, agents, home_positions=ROBOT_HOME_POSITIONS)
        
        manual.draw_overlay(vis)  # ← 수동 경로 오버레이

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
                'controller'.pause([str(r) for r in targets])
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
        # 수동 모드 
        elif key == ord('z'):
            manual.toggle_mode()  
        # 랜덤 모드    
        elif key == ord('m'): 
            _mode_idx[0] = (_mode_idx[0] + 1) % len(_mode_keys)
            name = _mode_keys[_mode_idx[0]]
            scenario.set_mode(MODE_FACTORY[name]())
            print(f"[Scenario] mode ← {name} (실행상태는 유지)")

        elif key == 32:  # Spacebar
            scenario.toggle_enabled()

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
    

    #print("▶ 'q' 입력 감지. command_transfer.py 종료 시도...")
    cts_process.terminate()


    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":

    main()