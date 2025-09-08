import sys
import os

import cv2
import numpy as np
import json
import time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))

# code에서 필요한 모듈 임포트
from grid import load_grid
from interface import grid_visual, slider_create, slider_value, draw_agent_points, draw_paths
from config import grid_row, grid_col, cell_size, camera_cfg
from vision.visionsystem import VisionSystem
from vision.camera import camera_open, Undistorter
from cbs.pathfinder import PathFinder
from cbs.agent import Agent
from commandSendTest3 import CommandSet, FeedbackSet
from DirectionCheck import compute_and_publish_errors

# 전역 변수
agents = []
paths = []
manager = None
pathfinder = None
grid_array = None
visualize = True

# 비전 시스템 초기화
video_path = r"C:/img/test2.mp4"
cap, fps = camera_open(source=None) # 특정 카메라나 영상을 쓰고 싶을 시 source=0(원하는 카메라 번호) 또는 source=video_path로 설정, 아니면 None으로 두기

undistorter = Undistorter(
    camera_cfg['type'],
    camera_cfg['matrix'],
    camera_cfg['dist'],
    camera_cfg['size']
)

vision = VisionSystem(undistorter=undistorter, visualize=True)

# 사용할 ID 목록
PRESET_IDS = [1,2,3,4,5,6,7,8,9,10,11]  # 예시: 1~12까지의 ID 사용

# 마우스 콜백 함수
def mouse_event(event, x, y, flags, param):
    """
    좌클릭  : 출발지(start) 지정
    우클릭  : 도착지(goal)  지정
    - PRESET_IDS(예: [2, 4]) 두 개가 모두 완성되면 CBS 실행
    """
    global agents, paths, pathfinder

    row, col = y // cell_size, x // cell_size
    if not (0 <= row < grid_row and 0 <= col < grid_col):
        return

    updated = False                 # ← 변경 여부 플래그
    complete_agents = [a for a in agents if a.start and a.goal]

    # ---------- 1. 출발지 클릭 ----------
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Start set at ({row}, {col})")

        # 1‑A. 이미 완성된 agent가 한도(PRESET_IDS)만큼이면 생성 제한
        if len(complete_agents) >= len(PRESET_IDS):
            print("더 이상 agent를 생성할 수 없습니다.")
            return
        
        if event == cv2.EVENT_LBUTTONDOWN and any(a.start == (row, col) for a in agents):
            return

        # 1‑B. goal‑only agent에 start 채우기
        for agent in agents:
            if agent.start is None and agent.goal is not None:
                agent.start = (row, col)
                updated = True
                break

        # 1‑C. start‑only agent의 start 덮어쓰기
        if not updated:
            for agent in agents:
                if agent.start is not None and agent.goal is None:
                    agent.start = (row, col)
                    updated = True
                    break

        # 1‑D. 둘 다 없으면 새 agent 생성
        if not updated:
            # 사용하지 않은 ID 선택
            used_ids = {a.id for a in agents}
            avail_ids = [pid for pid in PRESET_IDS if pid not in used_ids]
            if not avail_ids:
                print("더 이상 agent를 생성할 수 없습니다.")
                return
            new_id = avail_ids[0]
            agent = Agent(id=new_id, start=(row, col), goal=None, delay=0)
            agents.append(agent)
            updated = True

    # ---------- 2. 도착지 클릭 ----------
    elif event == cv2.EVENT_RBUTTONDOWN:
        print(f"Goal set at ({row}, {col})")

        # 2‑A. 이미 완성된 agent가 한도만큼이면 생성 제한
        if len(complete_agents) >= len(PRESET_IDS):
            print("더 이상 agent를 생성할 수 없습니다.")
            return

        # 2‑B. start‑only agent에 goal 채우기
        for agent in agents:
            if agent.goal is None and agent.start is not None:
                agent.goal = (row, col)
                updated = True
                break

        # 2‑C. goal‑only agent의 goal 덮어쓰기
        if not updated:
            for agent in agents:
                if agent.goal is not None and agent.start is None:
                    agent.goal = (row, col)
                    updated = True
                    break

        # 2‑D. 둘 다 없으면 새 agent 생성 (goal‑only)
        if not updated:
            used_ids = {a.id for a in agents}
            avail_ids = [pid for pid in PRESET_IDS if pid not in used_ids]
            if not avail_ids:
                print("더 이상 agent를 생성할 수 없습니다.")
                return
            new_id = avail_ids[0]
            agent = Agent(id=new_id, start=None, goal=(row, col), delay=0)
            agents.append(agent)
            updated = True

    # ---------- 3. 공통 후처리 ----------
    if updated:
        target_ids = set(PRESET_IDS)  # ← PRESET_IDS 기반으로 변경
        ready_ids  = {a.id for a in agents if a.start and a.goal and a.id in target_ids}

        if ready_ids == target_ids:
            print(f"Agent {sorted(ready_ids)} 준비 완료. CBS 실행.")
            compute_cbs()

# 태그를 통해 에이전트 업데이트
def update_agents_from_tags(tag_info):        # cm → 셀 좌표
    for tag_id, data in tag_info.items():
        if tag_id not in PRESET_IDS:
            continue
        if data.get("status") != "On":
            continue

        start_cell = data["grid_position"]                # (row, col)

        existing = next((a for a in agents if a.id == tag_id), None)
        if existing:                                      # 이미 agent 존재
            # ② 위치가 그대로면 아무것도 하지 않고 다음 tag로
            if existing.start == start_cell:
                continue
            existing.start = start_cell                   # 새 좌표로 갱신
        else:                                             # 처음 보는 tag
            agents.append(
                Agent(id=tag_id, start=start_cell, goal=None, delay=0)
            )

#CBS 계산
def compute_cbs():
    global paths, pathfinder, grid_array

    grid_array = load_grid(grid_row, grid_col)

    if pathfinder is None:
        pathfinder = PathFinder(grid_array)

    ready_agents = [a for a in agents if a.start and a.goal]
    if not ready_agents:
        print("⚠️  start·goal이 모두 지정된 에이전트가 없습니다.")
        return

    # ✅ 2) pathfinder에 ready_agents만 전달
    if pathfinder is None:
        pathfinder = PathFinder(load_grid())

    solved_agents = pathfinder.compute_paths(ready_agents)
    new_paths = [agent.get_final_path() for agent in solved_agents]

    if not new_paths:
        print("No solution found.")
        return

    paths.clear()
    paths.extend(new_paths)

    print("Paths updated via PathFinder.")

    # 로봇 명령 전송
    command_sets = [CommandSet(str(agent.id), agent.get_final_path(), initial_dir=getattr(agent, "direction", "north"))
                    for agent in solved_agents]

# 전송할 JSON 문자열을 미리 출력
    try:
        payload = json.dumps({"commands": [cs.to_dict() for cs in command_sets]}, indent=2, ensure_ascii=False)
        print("!!!전송 예정 명령 세트:")
        print(payload)
    except Exception as e:
        print(f"명령 세트 변환 중 오류 발생: {e}")

    # 실제 전송 시도
    try:
        CommandSet.send_command_sets(command_sets)
    except Exception as e:
        print(f"명령 전송 중 오류 발생: {e}")

# 딜레이 적용
def apply_start_delays(paths, starts, delays):
    delayed_paths = []
    for i, path in enumerate(paths):
        delay = delays[i]
        hold = [starts[i]] * delay
        delayed_paths.append(hold + path)
    return delayed_paths

def main():
    # 초기 설정
    global agents, paths, manager, visualize
    
    # 피드백 세트 초기화
    feedback_mode      = False
    feedback_selection = ""


    # 그리드 불러오기
    base_grid = load_grid(grid_row, grid_col)
    grid_array = base_grid.copy()

    # 슬라이더 생성
    slider_create()
    detect_params = slider_value()  # 슬라이더에서 받아오기

    cv2.namedWindow("Video_display", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Video_display", vision.mouse_callback)
    cv2.namedWindow("CBS Grid")
    cv2.setMouseCallback("CBS Grid", mouse_event)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 획득 실패")
            continue

        visionOutput = vision.process_frame(frame, detect_params)

        if visionOutput is None:
            continue
        vis = grid_visual(grid_array.copy())

        frame = visionOutput["frame"]
        tag_info = visionOutput["tag_info"]
        warped = visionOutput.get("warped")

        if warped is not None:
            warped_display = cv2.resize(warped, (480, 480))
            cv2.imshow("Warped Live", warped_display)

        if any("grid_position" in data for data in visionOutput["tag_info"].values()):
            update_agents_from_tags(visionOutput["tag_info"])


        #UI 시각화 화면
        
        draw_paths(vis, paths)
        draw_agent_points(vis, agents)
        
        cv2.imshow("CBS Grid", vis)
        cv2.imshow("Video_display", frame)

        key = cv2.waitKey(1)
        if feedback_mode:
            if key in map(ord, '0123456789'):
                feedback_selection = chr(key)  # 덮어쓰기
                print(f"선택된 ID: {feedback_selection}")
            elif key == 8:  # 백스페이스
                feedback_selection = ""
                print("선택 취소")
            elif key == 13:  # Enter
                all_fb = FeedbackSet.from_tag_info(tag_info)
                if feedback_selection == "":
                    FeedbackSet.send_feedback_sets(all_fb)
                else:
                    rid = int(feedback_selection)
                    single = [fs for fs in all_fb if fs.robot_id == rid]
                    if single:
                        FeedbackSet.send_feedback_sets(single)
                # 상태 초기화
                feedback_mode      = False
                feedback_selection = ""
            elif key == 27:  # ESC로 취소
                feedback_mode      = False
                feedback_selection = ""
                print("피드백 모드 취소")
            continue 
        if key == ord('q'):  # 'q' 키 -> 종료 (저장 없이)
            break
        elif key == ord('r'):
            print("Reset all")
            agents.clear()
            paths.clear()
        elif key == ord('m'):
            # 현재 모드가 'tag'면 'contour'로, 아니면 'tag'로 토글
            new_mode = 'contour' if vision.board_mode == 'tag' else 'tag'
            vision.set_board_mode(new_mode)
            print(f"Board mode switched to: {new_mode}")

        elif key == ord('c'):  # 'c' 키로 CBS 재계산
            if all(a.start and a.goal for a in agents):
                compute_cbs()
            else:
                print("start 또는 goal이 비어 있는 에이전트가 있습니다.")

        elif key == ord('n'):
            vision.lock_board()
            print("보드 고정됨")

        elif key == ord('b'):
            vision.reset_board()
            print("🔄 고정된 보드를 해제")

        elif key == ord('v'):
            vision.toggle_visualization()
            print(f"시각화 모드: {'ON' if vision.visualize else 'OFF'}")

        elif key == ord('p'):
            compute_and_publish_errors(tag_info, agents)
        
        elif key == ord('s'):
            vision.start_roi_selection()
        
        elif key == ord('f'):
            feedback_mode = True
            feedback_selection = ""
            print("피드백 모드 진입: 숫자 입력 후 Enter (아무것도 입력 안 하면 전체 전송)")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
