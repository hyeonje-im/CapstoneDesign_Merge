import sys
import os
import cv2
import numpy as np
import subprocess 
import math
import time
import threading
from queue import Queue, Empty

# ======================================
# UI 연동 관련
# ======================================
SHOW_CV_WINDOWS = bool(int(os.environ.get("SHOW_CV_WINDOWS", "1")))

_KEYQ: "Queue[int]" = Queue()

def push_keycode(code: int):
    """외부(Kivy)에서 보낸 가상 키코드를 백엔드에 전달"""
    _KEYQ.put(code)

# ======================================
# Path & Import
# ======================================
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

# ======================================
# 글로벌 상태
# ======================================
SELECTED_RIDS = set()
GOAL_ALIGN_MODE = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CTS_SCRIPT = os.path.join(SCRIPT_DIR, "command_transfer.py")

# command_transfer.py 별도 콘솔 실행
proc = subprocess.Popen([sys.executable, CTS_SCRIPT], creationflags=subprocess.CREATE_NEW_CONSOLE)
print(f"▶ command_transfer.py 별도 콘솔에서 실행: {CTS_SCRIPT}")

# ======================================
# MQTT
# ======================================
USE_MQTT = 0  # 0: 비사용, 1: 사용

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

# ======================================
# 보정 패널
# ======================================
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

# ======================================
# 안전 제어 (CollisionGuard, ReleaseManager)
# ======================================
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

rm = ReleaseManager(
    guard=guard,
    controller=controller,
    client=client,
    policy=ReleasePolicy(arming_delay_s=3.0, manage_window_s=1.25),
)

# ======================================
# 전역 변수
# ======================================
grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)
agents = []
paths = []
pathfinder = None
tag_info = {}
PRESET_IDS = []
selected_robot_id = None

# ======================================
# 비전 시스템
# ======================================
video_path = r"C:/img/test2.mp4"
cap, fps = camera_open(source=0)

undistorter = Undistorter(
    camera_cfg['type'],
    camera_cfg['matrix'],
    camera_cfg['dist'],
    camera_cfg['size']
)
vision = VisionSystem(undistorter=undistorter, visualize=True)
vision.correction_coef_getter = lambda: correction_coef_value

# ======================================
# 유틸 함수
# ======================================
def compute_visible_robot_ids(tag_info: dict) -> list[int]:
    visible = []
    for tid, data in tag_info.items():
        if isinstance(tid, int) and data.get("status") == "On" and tid != NORTH_TAG_ID:
            visible.append(tid)
    visible.sort()
    return visible

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
            cmds.append({'command': 'T185'})
        else:
            cmds.append({'command': 'L90'})
        hd = desired
    return cmds

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
    direction_angles = [90, 0, 270, 180]
    diffs = [abs(((yaw_deg - a + 180) % 360) - 180) for a in direction_angles]
    min_idx = diffs.index(min(diffs))
    hd = [0, 3, 2, 1][min_idx]
    return hd

# ======================================
# CBS
# ======================================
def compute_cbs():
    global paths, pathfinder, grid_array
    ready_agents = [a for a in agents if a.start and a.goal]
    waiters      = [a for a in agents if a.start and not a.goal]
    if not ready_agents:
        print("⚠️ start·goal 지정된 에이전트 없음")
        return
    aug_grid = grid_array.copy()
    for w in waiters:
        try:
            r, c = w.start
            if 0 <= r < grid_row and 0 <= c < grid_col:
                aug_grid[r, c] = 1
        except Exception:
            pass
    pathfinder_local = PathFinder(aug_grid)
    solved_agents = pathfinder_local.compute_paths(ready_agents)
    valid_agents = [a for a in solved_agents if a.get_final_path()]
    if not valid_agents:
        print("No solution found.")
        return
    paths.clear()
    paths.extend([a.get_final_path() for a in valid_agents])
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

# ======================================
# Main Loop
# ======================================
def main():
    global agents, paths, grid_array, tag_info, selected_robot_id, PRESET_IDS

    grid_array = np.zeros((grid_row, grid_col), dtype=np.uint8)
    slider_create()
    detect_params = slider_value()

    if SHOW_CV_WINDOWS:
        cv2.namedWindow("Video_display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Video_display", vision.mouse_callback)
        cv2.namedWindow("CBS Grid")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 획득 실패")
            continue
        visionOutput = vision.process_frame(frame, detect_params)
        if visionOutput is None:
            continue
        ob_grid = vision.get_obstacle_grid()
        if ob_grid is not None:
            grid_array = ob_grid.copy()
        vis = grid_visual(grid_array.copy())
        frame = visionOutput["frame"]
        tag_info = visionOutput["tag_info"]
        controller.set_tag_info_provider(lambda: tag_info)
        PRESET_IDS[:] = compute_visible_robot_ids(tag_info)
        update_agents_from_tags(tag_info)

        # UI 연동
        FrameBus.set_video(frame)
        FrameBus.set_grid(vis)

        # 안전 제어 tick
        guard.tick(tag_info, PRESET_IDS)
        rm.tick(tag_info, PRESET_IDS)

        # UI 명령 처리
        cmd, kwargs = get_cmd_nowait()
        if cmd == "select_robot":
            rid = int(kwargs["rid"])
            SELECTED_RIDS.clear()
            SELECTED_RIDS.add(rid)
            print(f"[UI] 선택 로봇 동기화 → {sorted(SELECTED_RIDS)}")
        elif cmd == "compute_cbs":
            compute_cbs()
        elif cmd == "lock_board":
            vision.lock_board()
            print("[UI] 보드 고정됨")
        elif cmd == "unlock_board":
            vision.reset_board()
            print("[UI] 보드 고정 해제")
        elif cmd == "toggle_visualization":
            vision.toggle_visualization()
            print(f"[UI] 시각화 모드: {'ON' if vision.visualize else 'OFF'}")

        # 화면 출력
        draw_paths(vis, paths)
        draw_agent_points(vis, agents)

        if SHOW_CV_WINDOWS:
            cv2.imshow("CBS Grid", vis)
            cv2.imshow("Video_display", frame)
            key = cv2.waitKey(1)
        else:
            try:
                key = _KEYQ.get_nowait()
            except Empty:
                key = -1

        if key == ord('q'):
            break

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
