# mode_sim_run.py
import os
import cv2
import sys
import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))

from ScenarioManager import ScenarioManager
from TestMode import TestMode
from mode_simulator import Simulator, LocalControllerStub, TagSynth
from fake_mqtt import FakeMQTTBroker
broker = FakeMQTTBroker()

GRID_SHAPE = (10, 10)
CELL = 50
COLORS = [(255,0,0),(0,255,0),(0,0,255),(255,128,0)]

# 시뮬레이터 구성
grid = np.zeros(GRID_SHAPE, dtype=np.uint8)
sim = Simulator(grid, colors=COLORS, cell_size=CELL)

# 컨트롤러/태그 합성
controller = LocalControllerStub(sim)
tags = TagSynth(sim)

# ScenarioManager 구성에 필요한 훅
agents, paths = [], []

def get_grid(): return sim.map_array
def get_tag_info(): return tags.get_tag_info()
def get_initial_hd(rid: int): return 0  # 단순화
def path_to_commands(path, init_hd):
    # 셀 간 이동을 전진(F10) 명령으로 치환 (최소 예시)
    n = max(0, len(path) - 1)
    return [{"command": "F10"} for _ in range(n)]

scenario = ScenarioManager(
    controller=controller, agents_ref=agents, paths_ref=paths,
    get_grid=get_grid, get_tag_info=get_tag_info,
    path_to_commands=path_to_commands, get_initial_hd=get_initial_hd,
    mode=TestMode(frames_per_step=5, idle_threshold_frames=8, delay_max_steps=2)
)

controller.set_sequence_completion_callback(lambda: scenario.on_sequence_complete())

# 마우스: 숫자키로 로봇 선택 → 좌클릭으로 해당 셀에 생성/이동
selected_robot = None
def on_mouse(event, x, y, flags, param):
    global selected_robot
    if event == cv2.EVENT_LBUTTONDOWN and selected_robot is not None:
        r, c = y // CELL, x // CELL
        rid = selected_robot
        if rid not in sim.robots:
            sim.add_robot(rid, broker=None, start_pos=(r, c))
        else:
            sim.robots[rid].position = (r, c)
        selected_robot = None

cv2.namedWindow("Grid")
cv2.setMouseCallback("Grid", on_mouse)
print("숫자키(1~9) 선택 후 Grid 좌클릭으로 로봇 생성/이동. q = 종료")

while True:
    # 표시
    img = sim.create_grid()
    sim.draw_home_positions(img)
    sim.draw_robots(img)
    cv2.imshow("Grid", img)

    # 시나리오 매니저 틱 + 컨트롤러 폴
    scenario.tick()
    controller.poll()

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif ord('1') <= key <= ord('9'):
        selected_robot = key - ord('0')
