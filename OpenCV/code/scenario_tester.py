import sys
import os
import random
from collections import deque

# MAPF-ICBS\code 경로를 추가
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ICBS_PATH = os.path.join(CURRENT_DIR, '..', 'MAPF-ICBS', 'code')
sys.path.append(os.path.normpath(ICBS_PATH))

import cv2
import numpy as np
from grid import load_grid
from interface import grid_visual, draw_agent_info_window
from OpenCV.code.simulator.simulator import Simulator
from OpenCV.code.simulator.fake_mqtt import FakeMQTTBroker
from OpenCV.code.simulator.commandSendTest3 import CommandSet
from cbs.pathfinder import PathFinder, Agent
from config import COLORS, grid_row, grid_col, cell_size
import json

# 전역 변수
agents = []
paths = []
sim = None
broker = FakeMQTTBroker()
pathfinder = None
grid_array = None
selected_robot_id = None # 생성할 때 선택된 로봇 ID

pending_steps = {}          # { robot_id: deque([(r,c), ...]) }
barrier_inflight = {}    # 직전에 보낸 스텝을 아직 수행 중인 로봇들
BARRIER_MODE = True         # 끄고 싶으면 False

delay_input_mode = False
delay_input_buffer = ""

random_mode_enabled = True

# 사용할 ID 목록
PRESET_IDS = [0,1,2,3,4,5,6,7,8,9]  # 예시: 1~12까지의 ID 사용

# === [NEW] 구역 태그/색상 관련 전역 ===
AREA_TAGS_MAP = {}   # {(r,c): {"zone": int, "tag": str, "role": "pickup"/"waiting", "color": (B,G,R)}}
AREA_TAGS_LIST = []  # [{"cell": (r,c), "zone": int, "tag": str, "role": ..., "color": (B,G,R)}]
CUSTOMER_CELLS = set()

ORDERS = []  # [{"cell": (r,c), "zone": int, "color": (B,G,R)} ...]
_next_order_tick = 0  # 다음 주문 생성 시각(프레임틱)

# 구역(페어)별 기본색 (원하면 더 추가)
TAG_BASE_COLORS = [
    (0, 0, 255),    # 빨강 (BGR)
    (0, 165, 255),  # 주황
    (0, 255, 255),  # 노랑
    (0, 255, 0),    # 초록
    (255, 0, 0),    # 파랑
    (255, 0, 255),  # 보라
]

SHIFT_DIGIT_MAP = {'!':'1','@':'2','#':'3','$':'4','%':'5','^':'6','&':'7','*':'8','(':'9',')':'0'}

def _zone_info_for_cell(cell):
    """아랫줄 태그에서 셀의 zone/color/role을 조회"""
    meta = AREA_TAGS_MAP.get(cell)
    if not meta:
        return None
    return {"zone": meta["zone"], "color": meta["color"], "role": meta["role"]}

def _zone_of_waiting_for_robot(robot_id):
    """로봇이 '대기칸' 위에 있으면 해당 zone/color를 반환, 아니면 None"""
    if robot_id not in sim.robots:
        return None
    pos = tuple(map(int, sim.robots[robot_id].get_position()))
    info = _zone_info_for_cell(pos)
    if info and info["role"] == "waiting":
        return info
    return None

def get_pickup_waiting_pair_by_zone(zone):
    """특정 zone의 (pickup_cell, waiting_cell, color) 반환"""
    pickup = next(i for i in AREA_TAGS_LIST if i["zone"]==zone and i["role"]=="pickup")["cell"]
    waiting = next(i for i in AREA_TAGS_LIST if i["zone"]==zone and i["role"]=="waiting")["cell"]
    color = next(i for i in AREA_TAGS_LIST if i["zone"]==zone)["color"]
    return pickup, waiting, color

def _random_order_interval_ticks():
    """주문 생성 간격(프레임) — 필요시 조절 (예: 0.6~2.0초, 100ms/프레임 가정)"""
    return random.randint(6, 20)

def _eligible_zones_with_waiting_robot():
    """대기칸에 실제 로봇이 있는 zone들만 반환"""
    zones = set()
    for rid, rb in sim.robots.items():
        info = _zone_of_waiting_for_robot(rid)
        if info:
            zones.add(info["zone"])
    return sorted(list(zones))

def spawn_random_order_if_due(tick_count):
    """랜덤 타이밍에, '대기칸에 로봇이 있는 색상들' 중 하나로 고객칸에 숫자0 생성"""
    global _next_order_tick
    if tick_count < _next_order_tick:
        return
    eligible = _eligible_zones_with_waiting_robot()
    if not eligible or not CUSTOMER_CELLS:
        _next_order_tick = tick_count + _random_order_interval_ticks()
        return
    zone = random.choice(eligible)
    pickup, waiting, color = get_pickup_waiting_pair_by_zone(zone)
    # 이미 동일 zone의 '0'이 존재하면 중복 생성하지 않음(원하면 허용 가능)
    if any(o["zone"]==zone for o in ORDERS):
        _next_order_tick = tick_count + _random_order_interval_ticks()
        return
    cell = random.choice(list(CUSTOMER_CELLS))
    ORDERS.append({"cell": cell, "zone": zone, "color": color})
    _next_order_tick = tick_count + _random_order_interval_ticks()

def draw_orders(vis_img):
    """고객칸의 숫자 '0' 시각화: 흰색 채움 + zone 테두리 + '0' 문자"""
    for o in ORDERS:
        r, c = o["cell"]
        x, y = c * cell_size, r * cell_size
        # 둥근 사각형 느낌의 원+테두리
        cx, cy = x + cell_size//2, y + cell_size//2
        rad = max(6, cell_size//3)
        cv2.circle(vis_img, (cx, cy), rad, (255,255,255), -1)       # 내부 흰색
        cv2.circle(vis_img, (cx, cy), rad, o["color"], 2)           # 테두리 = zone 색
        # 숫자 '0'
        cv2.putText(vis_img, "0", (cx - rad//3, cy + rad//3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

def _enqueue_one_step(robot_id, target_cell):
    """해당 로봇을 target_cell로 1칸 이동하도록 큐에 삽입"""
    if robot_id not in pending_steps:
        pending_steps[robot_id] = deque()
    pending_steps[robot_id].clear()
    pending_steps[robot_id].append(tuple(target_cell))

def nudge_waiting_robot_to_pickup_if_matching_order():
    """
    대기칸에 있는 로봇이 자기 zone 색상의 '0'이 있으면
    왼쪽(=수령칸)으로 한 칸 이동시킨 뒤 북쪽을 보게 함.
    """
    # zone -> active order 존재 여부
    zones_with_order = {o["zone"] for o in ORDERS}
    for rid, rb in sim.robots.items():
        info = _zone_of_waiting_for_robot(rid)
        if not info:
            continue
        if info["zone"] not in zones_with_order:
            continue
        # 왼쪽(수령칸)으로 한 칸
        _, waiting_cell, _ = get_pickup_waiting_pair_by_zone(info["zone"])
        wr, wc = waiting_cell
        pickup_cell = (wr, wc - 1)
        # 이동 가능 체크(격자 범위/장애물 회피)
        if not (0 <= pickup_cell[0] < grid_row and 0 <= pickup_cell[1] < grid_col):
            continue
        if grid_array[pickup_cell[0], pickup_cell[1]] != 0:
            continue
        # 1칸 이동 큐 + 방향 정렬
        _enqueue_one_step(rid, pickup_cell)

# === [NEW] 수령칸 인덱싱 (왼쪽부터 1..9)
def pickup_cells_left_to_right():
    cells = [i["cell"] for i in AREA_TAGS_LIST if i["role"]=="pickup"]
    cells.sort(key=lambda rc: rc[1])  # col 기준
    return cells

def zone_by_pickup_index(idx1_based):
    pcs = pickup_cells_left_to_right()
    if 1 <= idx1_based <= len(pcs):
        cell = pcs[idx1_based-1]
        meta = AREA_TAGS_MAP.get(cell)
        return meta["zone"] if meta else None
    return None

def handle_keypad_digit(digit_char):
    """
    수령칸에서 1~9 입력 시: 해당 인덱스의 '수령칸 색상'과 같은 '0' 주문으로 경로 생성.
    - 현재 '그 수령칸'에 실제로 로봇이 있어야 동작(가장 자연스러움)
    """
    if digit_char < '1' or digit_char > '9':
        return
    idx = int(digit_char)
    zone = zone_by_pickup_index(idx)
    if zone is None:
        print(f"[입력] {idx}: 해당하는 수령칸이 없습니다.")
        return

    # 해당 zone의 '0' 주문 찾기
    target_order = next((o for o in ORDERS if o["zone"] == zone), None)
    if not target_order:
        print(f"[입력] {idx}: zone {zone}의 주문(0)이 없습니다.")
        return

    # 그 수령칸에 '현재' 로봇이 있는지 확인하고 그 로봇을 목적지로 보냄
    pickup_cell, _, _ = get_pickup_waiting_pair_by_zone(zone)
    robot_on_pickup = None
    for rid, rb in sim.robots.items():
        pos = tuple(map(int, rb.get_position()))
        if pos == pickup_cell:
            robot_on_pickup = rid
            break
    if robot_on_pickup is None:
        print(f"[입력] {idx}: 수령칸에 로봇이 없습니다.")
        return

    # 에이전트 찾아 goal 지정 후 CBS
    agent = next((a for a in agents if a.id == robot_on_pickup), None)
    if not agent:
        agent = Agent(id=robot_on_pickup, start=pickup_cell, goal=None, delay=0)
        agents.append(agent)

    agent.start = pickup_cell  # 현재 위치로 start 갱신
    agent.goal = target_order["cell"]

    print(f"[주문수락] 로봇 {robot_on_pickup} -> {agent.goal} (zone={zone})")
    # 주문은 하나만 소비(원하면 여러개 허용 가능)
    ORDERS.remove(target_order)

    compute_cbs()

def get_waiting_cells_left_to_right():
    """아랫줄 '대기(waiting)' 칸들을 왼->오 순으로 정렬해 반환"""
    waiting = [item["cell"] for item in AREA_TAGS_LIST if item["role"] == "waiting"]
    waiting.sort(key=lambda rc: rc[1])  # col 기준 정렬
    return waiting

def auto_spawn_waiting_robots():
    """
    아랫줄 '대기' 칸마다 로봇/에이전트를 자동 생성.
    - ID는 왼쪽부터 1,2,3,... 순서
    - 이미 로봇/에이전트가 있으면 중복 생성 안 함
    - PRESET_IDS도 동적으로 재설정
    """
    global PRESET_IDS, agents, sim

    waiting_cells = get_waiting_cells_left_to_right()
    PRESET_IDS = list(range(1, len(waiting_cells) + 1))  # 동적 IDs

    for idx, cell in enumerate(waiting_cells, start=1):
        # 로봇이 없으면 추가
        if idx not in sim.robots:
            sim.add_robot(idx, broker, start_pos=cell, direction="west")
        else:
            # 있으면 위치만 맞춰 둠
            rbt = sim.robots[idx]
            rbt.position = cell
            rbt.start_pos = cell
            rbt.target_pos = cell
            rbt.direction = "west"

        # 에이전트가 없으면 start-only로 추가 (CBS 제외 상태)
        existing = next((a for a in agents if a.id == idx), None)
        if existing:
            existing.start = cell
            # goal은 그대로 둠(None이면 대기)
        else:
            agents.append(Agent(id=idx, start=cell, goal=None, delay=0))

        # UI 정보 업데이트
        sim.robot_info[idx]['start'] = cell
        sim.robot_info[idx]['goal'] = None
        sim.robot_info[idx]['path'] = []

def init_customer_cells():
    """
    장애물(=1) 셀의 8-이웃 중 빈 칸(=0)을 Customer 지점으로 등록
    """
    global CUSTOMER_CELLS, grid_array
    CUSTOMER_CELLS.clear()

    R, C = grid_array.shape
    nbrs = [(-1,0),(0,-1),(0,1),(1,0)]
    for r in range(R):
        for c in range(C):
            if grid_array[r, c] == 1:  # 장애물
                for dr, dc in nbrs:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < R and 0 <= nc < C and grid_array[nr, nc] == 0:
                        CUSTOMER_CELLS.add((nr, nc))

def _is_waiting_cell(cell):
    m = AREA_TAGS_MAP.get(cell)
    return bool(m and m.get("role") == "waiting")

def _waiting_cells():
    return [i["cell"] for i in AREA_TAGS_LIST if i["role"] == "waiting"]

def _occupied_cells():
    """현재 점유 + target_pos(정지 로봇 포함)까지 모두 점유로 간주"""
    occ = set()
    for rb in sim.robots.values():
        occ.add(_to_cell(rb.get_position()))
        # target_pos도 항상 포함 (정지/대기 로봇의 고정 위치 보장)
        if hasattr(rb, "target_pos") and rb.target_pos is not None:
            occ.add(_to_cell(rb.target_pos))
    return occ

def _is_home_cell(cell):
    """수령/대기 칸 여부"""
    m = AREA_TAGS_MAP.get(cell)
    return bool(m and m.get("role") in ("waiting", "pickup"))

def _home_cells():
    """수령+대기 모든 칸"""
    return [i["cell"] for i in AREA_TAGS_LIST if i["role"] in ("waiting", "pickup")]

def _reserved_cells():
    """이번 배리어로 이미 보낸 스텝, 다음 예정 스텝을 예약 점유로 간주"""
    rs = set()
    rs |= { _to_cell(t) for t in barrier_inflight.values() }  # 이미 보낸 한 스텝의 타깃
    for dq in pending_steps.values():
        if dq: rs.add(_to_cell(dq[0]))                        # 다음 예정 스텝
    return rs

# 좌표를 격자 셀 인덱스로 안정적으로 스냅(부동소수 오차 방지)
def _to_cell(pos_tuple):
    r, c = pos_tuple
    return (int(round(r)), int(round(c)))

def _zones_home_pairs():
    """[(zone, pickup_cell, waiting_cell), ...]"""
    zones = {}
    for item in AREA_TAGS_LIST:
        z = item["zone"]
        zones.setdefault(z, {"pickup": None, "waiting": None})
        zones[z][item["role"]] = item["cell"]
    out = []
    for z, ps in zones.items():
        if ps["pickup"] is not None and ps["waiting"] is not None:
            out.append((z, ps["pickup"], ps["waiting"]))
    return out

def nearest_waiting_in_free_zone(from_pos):
    """
    '둘 다 비어있는' ZONE들만 후보로 삼아, from_pos에서 가장 가까운 ZONE을 고른 뒤
    그 ZONE의 '대기칸'을 반환. (없으면 None)
    """
    occ_like = _occupied_cells() | _reserved_cells()

    candidates = []
    for z, pc, wc in _zones_home_pairs():
        if (pc not in occ_like) and (wc not in occ_like):
            # ZONE 거리 = min( pickup까지, waiting까지 ) 맨해튼
            d = min(abs(pc[0]-from_pos[0]) + abs(pc[1]-from_pos[1]),
                    abs(wc[0]-from_pos[0]) + abs(wc[1]-from_pos[1]))
            candidates.append((d, z, pc, wc))
    if not candidates:
        return None

    candidates.sort(key=lambda t: t[0])
    _, z, pc, wc = candidates[0]
    return wc  # 귀환은 일관되게 '대기칸'으로


def nearest_free_home_cell(from_pos):
    """from_pos에서 가장 가까운 '빈' 수령/대기 칸 (현재 점유 + 예약 점유 제외)"""
    occ_like = _occupied_cells() | _reserved_cells()
    candidates = [h for h in _home_cells() if h not in occ_like]
    if not candidates:
        return None
    ret = min(candidates, key=lambda h: abs(h[0]-from_pos[0]) + abs(h[1]-from_pos[1]))
    # 최종 가드: 혹시나 동시 프레임 갱신으로 막혔으면 차선택
    if ret in (_occupied_cells() | _reserved_cells()):
        # 다시 걸러서 하나 더 고름
        new_cands = [h for h in candidates if h not in (_occupied_cells() | _reserved_cells()) and h != ret]
        if new_cands:
            ret = min(new_cands, key=lambda h: abs(h[0]-from_pos[0]) + abs(h[1]-from_pos[1]))
        else:
            return None
    return ret

# (선택) 고객 지점 시각화가 필요하면 사용
def draw_customers(vis_img):
    """
    Customer 지점을 가볍게 표시 (작은 원)
    """
    for (r, c) in CUSTOMER_CELLS:
        x, y = c * cell_size, r * cell_size
        cx, cy = x + cell_size//2, y + cell_size//2
        cv2.circle(vis_img, (cx, cy), max(2, cell_size//6), (255, 255, 255), -1)  # 흰 점
        cv2.circle(vis_img, (cx, cy), max(2, cell_size//6)+1, (200, 200, 200), 1) # 테두리

def _lighten_bgr(bgr, alpha=0.5):
    """밝게(연하게) 만들기: white(255,255,255)와 보간"""
    b,g,r = bgr
    return (int(b + (255-b)*alpha), int(g + (255-g)*alpha), int(r + (255-r)*alpha))

def init_bottom_row_tags():
    """
    가장 아랫줄(r = grid_row-1)을 2칸씩 페어로 묶어 태그 생성.
    - 왼쪽: 수령(pickup, 진한색)
    - 오른쪽: 대기(waiting, 연한색)
    - 홀수 칸이면 맨 오른쪽 1칸은 사용하지 않음(버림)
    """
    global AREA_TAGS_MAP, AREA_TAGS_LIST

    AREA_TAGS_MAP.clear()
    AREA_TAGS_LIST.clear()

    r = grid_row - 1
    pairs = grid_col // 2   # 홀수면 맨 오른쪽 1칸 버림
    for i in range(pairs):
        left_c  = 2*i
        right_c = 2*i + 1

        base = TAG_BASE_COLORS[i % len(TAG_BASE_COLORS)]
        light = _lighten_bgr(base, alpha=0.55)  # 연하게

        left_item = {
            "cell": (r, left_c),
            "zone": i,
            "tag": f"ZONE_{i}",
            "role": "pickup",       # 수령구역(왼쪽)
            "color": base
        }
        right_item = {
            "cell": (r, right_c),
            "zone": i,
            "tag": f"ZONE_{i}",
            "role": "waiting",      # 대기구역(오른쪽)
            "color": light
        }

        AREA_TAGS_LIST.extend([left_item, right_item])
        AREA_TAGS_MAP[(r, left_c)]  = {k:v for k,v in left_item.items() if k != "cell"}
        AREA_TAGS_MAP[(r, right_c)] = {k:v for k,v in right_item.items() if k != "cell"}

def draw_bottom_row_tags(vis_img):
    """
    태그 색을 시각화 (수령=진함, 대기=연함).
    paths보다 '아래'에 깔고 싶으면 draw_paths 전에 호출,
    '위'에 보이고 싶으면 draw_paths 후에 호출.
    """
    overlay = vis_img.copy()
    for item in AREA_TAGS_LIST:
        (r, c) = item["cell"]
        color = item["color"]
        x, y = c * cell_size, r * cell_size
        cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)
    cv2.addWeighted(overlay, 0.35, vis_img, 0.65, 0, vis_img)


# 마우스 콜백 함수
def mouse_event(event, x, y, flags, param):
    """
    좌클릭  : 출발지(start) 지정
    우클릭  : 도착지(goal)  지정
    - PRESET_IDS(예: [2, 4]) 두 개가 모두 완성되면 CBS 실행
    """
    global agents, paths, pathfinder, selected_robot_id
    row, col = y // cell_size, x // cell_size
    if not (0 <= row < grid_row and 0 <= col < grid_col):
        return

    updated = False                 # ← 변경 여부 플래그
    complete_agents = [a for a in agents if a.start and a.goal]

    # ---------- 1. 출발지 클릭 & 로봇 생성 ----------
    if event == cv2.EVENT_LBUTTONDOWN:

        if selected_robot_id is None:
            return  # 아무 것도 선택 안 된 경우 무시
        
        pos = (row, col)

        # 로봇 생성 또는 위치 초기화
        if selected_robot_id in sim.robots:
            robot = sim.robots[selected_robot_id]
            robot.position = pos
            robot.start_pos = pos
            robot.target_pos = pos
            sim.robot_info[selected_robot_id]['start'] = pos
        else:
            robot = sim.add_robot(selected_robot_id, broker, start_pos=pos)

        # 에이전트 생성 + start 설정
        if all(a.id != selected_robot_id for a in agents):
            agent = Agent(id=selected_robot_id, start=pos, goal=None, delay=0)
            agents.append(agent)
        else:
            # 이미 존재하는 agent라면 start만 업데이트 (정합성 보장)
            for agent in agents:
                if agent.id == selected_robot_id:
                    agent.start = pos
                    break

        selected_robot_id = None
        return


    # ---------- 2. 도착지 클릭 ----------
    elif event == cv2.EVENT_RBUTTONDOWN:
        if selected_robot_id is None:
            return  # 아무 것도 선택 안 된 경우 무시

        print(f"Goal set at ({row}, {col})")

        # 2‑A. 이미 완성된 agent가 한도만큼이면 생성 제한
        if len(complete_agents) >= len(PRESET_IDS):
            print("더 이상 agent를 생성할 수 없습니다.")
            return

        # 2‑B. start‑only agent에 goal 채우기
        for agent in agents:
            if agent.id == selected_robot_id and agent.goal is None and agent.start is not None:
                agent.goal = (row, col)
                updated = True
                break

        # 2‑C. goal‑only agent의 goal 덮어쓰기
        if not updated:
            for agent in agents:
                if agent.id == selected_robot_id and agent.goal is not None and agent.start is None:
                    agent.goal = (row, col)
                    updated = True
                    break

        # 2‑D. 둘 다 없으면 새 agent 생성 (goal‑only)
        if not updated:
            used_ids = {a.id for a in agents}
            if selected_robot_id in used_ids:
                # ✅ 이미 존재하는 agent의 goal을 덮어쓰기 (이동 중 goal 변경용)
                for agent in agents:
                    if agent.id == selected_robot_id:
                        agent.goal = (row, col)
                        updated = True
                        print(f"Agent {agent.id}의 도착지를 ({row}, {col})로 변경")
                        break
            else:
                if selected_robot_id not in PRESET_IDS:
                    print(f"{selected_robot_id}는 허용된 ID 목록에 없습니다.")
                    return
                agent = Agent(id=selected_robot_id, start=None, goal=(row, col), delay=0)
                agents.append(agent)
                updated = True

        selected_robot_id = None
        return


    # ---------- 3. 공통 후처리 ----------
    if updated:
        target_ids = set(PRESET_IDS)  # ← PRESET_IDS 기반으로 변경
        ready_ids  = {a.id for a in agents if a.start and a.goal and a.id in target_ids}

        if ready_ids == target_ids:
            print(f"Agent {sorted(ready_ids)} 준비 완료. CBS 실행.")
            compute_cbs()

#에이전트 시작 위치를 로봇 현재 위치로 설정
def get_start_from_robot():
    for agent in agents:
        if agent.id in sim.robots:
            robot = sim.robots[agent.id]
            pos = robot.target_pos if robot.moving else robot.position  # 핵심 변경
            int_pos = tuple(map(int, pos))
            agent.start = int_pos
            sim.robot_info[agent.id]['start'] = int_pos

# 에이전트 초기 방향을 로봇의 회전 방향으로 설정
def get_direction_from_robot():
    for agent in agents:
        if agent.id in sim.robots:
            robot = sim.robots[agent.id]
            directions = ["north", "east", "south", "west"]
            idx = directions.index(robot.direction)

            if robot.rotating and robot.rotation_dir:
                delta = 1 if robot.rotation_dir == "right" else -1
                expected_dir = directions[(idx + delta) % 4]
            else:
                expected_dir = robot.direction

            agent.initial_dir = expected_dir  # CommandSet 생성 시 참조할 수 있게 저장

def _expected_dir(robot):
    directions = ["north", "east", "south", "west"]
    idx = directions.index(robot.direction)
    if robot.rotating and getattr(robot, "rotation_dir", None):
        delta = 1 if robot.rotation_dir == "right" else -1
        return directions[(idx + delta) % 4]
    return robot.direction

def send_next_step(robot_id):
    """로봇이 유휴면 다음 셀로 이동하는 '한 스텝짜리' CommandSet 전송"""
    if robot_id not in pending_steps or not pending_steps[robot_id]:
        return False
    if robot_id not in sim.robots:
        return False

    robot = sim.robots[robot_id]
    if robot.moving or robot.rotating:
        return False

    cur_pos = tuple(map(int, sim.robots[robot_id].get_position()))
    while pending_steps[robot_id] and tuple(pending_steps[robot_id][0]) == cur_pos:
        pending_steps[robot_id].popleft()
    if not pending_steps[robot_id]:
        return False

    # 한 칸만 보장(방어 로직)
    target = tuple(pending_steps[robot_id][0])
    manh = abs(target[0]-cur_pos[0]) + abs(target[1]-cur_pos[1])
    if manh > 1:
        step = (cur_pos[0] + (1 if target[0] > cur_pos[0] else -1 if target[0] < cur_pos[0] else 0),
                cur_pos[1] + (1 if target[1] > cur_pos[1] else -1 if target[1] < cur_pos[1] else 0))
    else:
        step = pending_steps[robot_id].popleft()

    cs = CommandSet(str(robot_id), [cur_pos, step], initial_dir=_expected_dir(robot))
    broker.send_command_sets([cs])

    # 🔹 이번 배리어 사이클에서 이 로봇의 목표칸을 기록
    barrier_inflight[robot_id] = step
    return True

def _all_idle(ids):
    # 모두 '대기(이동/회전 중 아님)' 상태인지 확인
    for rid in ids:
        if rid not in sim.robots:
            return False
        r = sim.robots[rid]
        if r.moving or r.rotating:
            return False
    return True

def dispatch_if_barrier_ready():
    # 1) 직전에 보낸 스텝의 '도착'만 정리 (idle이지만 아직 출발칸이면 유지)
    for rid, tgt in list(barrier_inflight.items()):
        if rid not in sim.robots:
            barrier_inflight.pop(rid, None)
            continue
        r = sim.robots[rid]
        pos = tuple(map(int, r.get_position()))
        if (not r.moving and not r.rotating) and pos == tgt:
            barrier_inflight.pop(rid, None)  # 도착 완료 → 배리어 탈퇴

    # 2) 아직 누가 이동 중이면 다음 턴 대기
    if barrier_inflight:
        return False

    # 3) 다음 스텝 후보(남은 칸 있는 로봇)
    active = [rid for rid, dq in pending_steps.items() if dq]
    if not active:
        return False

    # 4) 모두 '대기' 상태일 때에만 동시에 한 칸 보냄
    if not _all_idle(active):
        return False

    for rid in active:
        send_next_step(rid)
    return True

# ⬇️ cbs_tester.py 상단 헬퍼들 근처에 추가
def expand_to_unit_steps(path):
    """[(r,c), (r,c+3)] 같은 구간을 [(r,c+1),(r,c+2),(r,c+3)]로 펼침"""
    out = []
    for i in range(len(path) - 1):
        r1, c1 = path[i]
        r2, c2 = path[i + 1]
        dr = 0 if r2 == r1 else (1 if r2 > r1 else -1)
        dc = 0 if c2 == c1 else (1 if c2 > c1 else -1)
        # 대각선 방지(있다면 경로 생성 단계 문제)
        if dr != 0 and dc != 0:
            raise ValueError(f"Diagonal segment in path: {path[i]}->{path[i+1]}")
        rr, cc = r1, c1
        while (rr, cc) != (r2, c2):
            rr += dr
            cc += dc
            out.append((rr, cc))
    return out

#CBS 계산
#CBS 계산
def compute_cbs():
    """
    - 출발/도착이 '둘 다 있는' agent만 CBS에 전달
    - 출발만 있는 agent는 CBS 대상에서 제외 → 명령 미생성(가만히 있게)
    - 도착만 있는 agent는 무시
    - 빈 입력 혹은 경로 0개면 paths/pending_steps 정리하고 종료
    """
    global paths, pathfinder, grid_array, pending_steps, barrier_inflight, agents, sim

    # 최신 그리드 로드 & 로봇 현재 위치로 start 갱신
    grid_array = load_grid(grid_row, grid_col)
    get_start_from_robot()

    # 1) CBS에 보낼 대상만 필터링
    cbs_input_agents = [a for a in agents if a.start is not None and a.goal is not None]

    # 2) 아무도 없으면 깨끗이 정리하고 종료
    if not cbs_input_agents:
        print("[CBS] 유효한 (start+goal) agent가 없습니다. 명령을 생성하지 않습니다.")
        paths.clear()
        pending_steps.clear()
        # 화면 로봇 정보도 비우기(선택 사항)
        if sim:
            for rid in list(sim.robot_info.keys()):
                sim.robot_info[rid]['path'] = []
                sim.robot_info[rid]['goal'] = None
        return

    # 3) PathFinder 준비
    if pathfinder is None:
        pathfinder = PathFinder(grid_array)

    # 4) 안전 가드: 예외 발생 시 조용히 정리
    try:
        new_agents = pathfinder.compute_paths(cbs_input_agents)
    except Exception as e:
        print(f"[CBS] 경로 계산 중 오류로 중단: {e}")
        paths.clear()
        pending_steps.clear()
        return

    # 5) 경로 수집
    new_paths = [agent.get_final_path() for agent in new_agents if agent.get_final_path()]
    if not new_paths:
        print("[CBS] 생성된 경로가 없습니다.")
        paths.clear()
        pending_steps.clear()
        return

    # 6) 그리기용 paths 갱신
    paths.clear()
    paths.extend(new_paths)
    print("[CBS] Paths updated via PathFinder.")

    # 7) 모든 agent의 지연 초기화
    for agent in agents:
        agent.delay = 0

    # 8) 1-스텝 명령 큐 재구성: CBS에 사용된 agent만 대상
    pending_steps.clear()
    for agent in new_agents:
        if agent.id in sim.robots:
            fp = agent.get_final_path() or []
            unit_steps = expand_to_unit_steps(fp) if len(fp) > 1 else []
            pending_steps[agent.id] = deque(unit_steps)

    # 9) 시뮬레이터 표시 갱신 (CBS 대상만)
    if sim:
        for agent in new_agents:
            if agent.id in sim.robots:
                sim.robot_info[agent.id]['path'] = agent.get_final_path()
                sim.robot_info[agent.id]['goal'] = agent.goal

    # 10) CBS에서 제외된(=출발만 있는) 로봇들은 path/goal 표시를 비워 명령이 안 가도록 유지(선택)
    if sim:
        excluded_ids = {a.id for a in agents if (a.start is not None and a.goal is None)}
        for rid in excluded_ids:
            if rid in sim.robot_info:
                sim.robot_info[rid]['path'] = []
                # goal은 UI 용으로 남겨도 무방하나, 확실히 “가만히”를 원하면 아래도 비워주세요.
                sim.robot_info[rid]['goal'] = None

#경로 색칠용 코드
# === [REPLACE] 경로 색칠: 로봇 색 팔레트에 맞춤 ===
def draw_paths(vis_img, _paths_ignored=None):
    """
    sim.robot_info에 들어있는 각 로봇의 path를 그 로봇 색으로 반투명 칠한다.
    (Simulator.colors와 동일 규칙: robot_id % len(colors))
    """
    if sim is None or not hasattr(sim, "robot_info"):
        return

    # Simulator 쪽 팔레트 사용
    palette = getattr(sim, "colors", None)
    if not palette:
        return

    for rid, info in sim.robot_info.items():
        p = info.get('path') or []
        if not p:
            continue

        color = palette[rid % len(palette)]  # ← draw_robots와 동일 규칙 사용
        overlay = vis_img.copy()

        # path 전체를 한 번의 overlay로 칠함(성능/투명도 일관성)
        for (r, c) in p:
            x, y = c * cell_size, r * cell_size
            cv2.rectangle(overlay, (x, y), (x + cell_size, y + cell_size), color, -1)

        # 투명도는 원하신 대로 적당히(기본 0.28)
        cv2.addWeighted(overlay, 0.28, vis_img, 0.72, 0, vis_img)


def _rebuild_paths_from_robot_info():
    """sim.robot_info에 남아있는 경로만 모아 전역 paths를 재구성"""
    global paths
    new_paths = []
    for rid, info in sim.robot_info.items():
        p = info.get('path', [])
        if p:
            new_paths.append(p)
    paths.clear()
    paths.extend(new_paths)

# 로봇 도착 시 재계산
def on_robot_arrival(robot_id, pos):
    """
    목적지 도착 시:
      - 대기칸이면: 방향을 west로 두고, goal/대기 큐를 모두 비운 뒤 종료
      - 수령칸이면: 종료(그대로 대기)
      - 그 외라면: 가장 가까운 빈 수령/대기 칸으로 귀환
    """
    global agents, pending_steps, barrier_inflight

    pos = tuple(map(int, pos))

    # 1) 대기칸 처리: 방향 west + goal/pending 정리
    if _is_waiting_cell(pos):
        # 방향을 명령 없이 "그냥" 바꿈
        if robot_id in sim.robots:
            sim.robots[robot_id].direction = "west"

        # 이 로봇의 goal/pending/path UI 정리
        ag = next((a for a in agents if a.id == robot_id), None)
        if ag:
            ag.goal = None
        pending_steps.pop(robot_id, None)
        barrier_inflight.pop(robot_id, None)
        if robot_id in sim.robot_info:
            sim.robot_info[robot_id]['goal'] = None
            sim.robot_info[robot_id]['path'] = []
        _rebuild_paths_from_robot_info()
        return

    # 2) 수령칸은 그냥 대기
    if _is_home_cell(pos):  # pickup or waiting
        return

    # 3) 그 외: 귀환 목표 지정
    ret = nearest_waiting_in_free_zone(pos)
    if ret is None:
        print(f"[귀환] '둘 다 비어있는' ZONE이 없습니다. (robot {robot_id})")
        return

    agent = next((a for a in agents if a.id == robot_id), None)
    if not agent:
        agent = Agent(id=robot_id, start=pos, goal=None, delay=0)
        agents.append(agent)

    agent.start = pos
    agent.goal  = ret

    print(f"[귀환] 로봇 {robot_id}: {pos} → {ret} (가까운 수령/대기칸)")
    compute_cbs()

def main():
    global agents, paths, grid_array, selected_robot_id, sim, delay_input_buffer, delay_input_mode, random_mode_enabled
    grid_array = load_grid(grid_row, grid_col)
    init_bottom_row_tags()
    cv2.namedWindow("CBS Grid")
    cv2.setMouseCallback("CBS Grid", mouse_event)

    sim = Simulator(grid_array.astype(bool), colors=COLORS)
    sim.register_arrival_callback(on_robot_arrival)
    sim.random_mode_enabled = True
    auto_spawn_waiting_robots()
    init_customer_cells()
    tick = 0

    while True:
        vis = grid_visual(grid_array.copy())
        draw_customers(vis)
        draw_bottom_row_tags(vis)

        draw_orders(vis)

        draw_paths(vis, paths)
        
        # --- Start 마커: 로봇 색으로
        for agent in agents:
            if agent.id in sim.robots:
                pos = sim.robots[agent.id].get_position()
                x, y = int(pos[1] * cell_size), int(pos[0] * cell_size)
                color = sim.colors[agent.id % len(sim.colors)]
                cv2.circle(vis, (x + cell_size//2, y + cell_size//2), 5, color, -1)
                cv2.putText(vis, f"S{agent.id}", (x + 2, y + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # --- Goal 마커: 로봇 색으로
        for agent in agents:
            if agent.goal:
                x, y = agent.goal[1] * cell_size, agent.goal[0] * cell_size
                color = sim.colors[agent.id % len(sim.colors)]
                cv2.circle(vis, (x + cell_size//2, y + cell_size//2), 5, color, -1)
                cv2.putText(vis, f"G{agent.id}", (x + 2, y + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        agent_info_img = draw_agent_info_window(
            agents,
            preset_ids=PRESET_IDS,
            total_height=grid_array.shape[0] * cell_size,
            selected_robot_id=selected_robot_id,
            delay_input_mode=delay_input_mode,
            delay_input_buffer=delay_input_buffer,
            cell_size=cell_size
        )

        combined = cv2.hconcat([vis, agent_info_img])
        cv2.imshow("CBS Grid", combined)
        
        sim.run_once()
        dispatch_if_barrier_ready()

        spawn_random_order_if_due(tick)
        nudge_waiting_robot_to_pickup_if_matching_order()
        tick += 1
        
        # 키보드 입력 처리
        key = cv2.waitKey(100)
        if key != -1:
            key_char = chr(key & 0xFF)
            if delay_input_mode:
                if key_char.isdigit():
                    delay_input_buffer += key_char
                elif key == 8:  # Backspace
                    delay_input_buffer = delay_input_buffer[:-1]
                elif key == 13 or key == 10:  # Enter
                    if selected_robot_id is not None and delay_input_buffer.isdigit():
                        delay_val = int(delay_input_buffer)
                        existing = next((a for a in agents if a.id == selected_robot_id), None)
                        if existing:
                            existing.delay = delay_val
                        else:
                            agent = Agent(id=selected_robot_id, start=None, goal=None, delay=delay_val)
                            agents.append(agent)
                    delay_input_mode = False
                    delay_input_buffer = ""

            else:
                if key_char in SHIFT_DIGIT_MAP:
                    digit_str = SHIFT_DIGIT_MAP[key_char]
                    if digit_str != '0':          # 0은 안 쓸 거라면 건너뛰기
                        handle_keypad_digit(digit_str)
                    continue

                # 2) 숫자만: 기존 로봇 선택
                if key_char.isdigit():
                    selected_robot_id = int(key_char)
                    if selected_robot_id in PRESET_IDS:
                        print(f"[로봇선택] ID {selected_robot_id} 선택됨.")
                    continue

                elif key == ord('d') and selected_robot_id in PRESET_IDS:
                    print(f"Delay 입력 모드 진입 (ID {selected_robot_id})")
                    delay_input_mode = True
                    delay_input_buffer = ""

        if key == ord('q'):
            break
        elif key == ord('z'):
            print("Reset all")
            agents.clear()
            paths.clear()

        elif key == ord(' '):  # ✅ Spacebar 눌러서 일시정지
            sim.paused = not sim.paused
            print("Paused" if sim.paused else "Resumed")
        
        elif key == ord('c'):  # 'c' 키로 CBS 재계산
            compute_cbs()

        elif key == ord('x'):
            selected_robot_id = None
            delay_input_mode = False
            delay_input_buffer = ""

        elif key == ord('r'):
            random_mode_enabled = not random_mode_enabled
            sim.random_mode_enabled = random_mode_enabled

            
            
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
