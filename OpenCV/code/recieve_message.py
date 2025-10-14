
import time
import json
import paho.mqtt.client as mqtt
from OpenCV.code.align import send_center_align, send_north_align 
from OpenCV.code.config import MQTT_TOPIC_COMMANDS_, MQTT_PORT, IP_address_ ,NORTH_TAG_ID
import threading


# ===== Barrier-step sync (모든 로봇 동시 한 스텝씩) =====
current_step = 0
max_steps = 0
step_inflight = set()    # 이번 스텝 전송 대상(완료 대기)
step_done = set()        # 이번 스텝 완료한 로봇

# 직진 전 보정 임계값
DIRECTION_CORR_THRESHOLD_DEG = 3.0   # 방향오차 임계(도)

# 정렬 명령 간 딜레이 (카메라 프레임 처리 보장용) DEBUG on_message
alignment_delay_sec = 0.8

paused = False                 # 전체 시퀀스 일시정지 여부
inflight = {}                  # rid -> bool (현재 1개 명령 수행중인지)

DONE_TOPIC = "robot/done"


# 이번 스텝에서 '방향 보정 + 직진(2단계)'을 보낸 로봇들 → MOVE 완료까지 기다리기
step_wait_for_move = set()

robot_command_map = {}     # 전체 명령
robot_indices = {}         # 현재 인덱스
last_heading = {}          # 로봇별 기준 방향 (0=북,1=동,2=남,3=서)
tag_info_provider = None   # 최신 yaw 값을 가져오는 콜백
active = False             # 실행 중 여부

#스탭 동기화 관련 변수
current_step = 0           # 현재 스텝 인덱스(0-based)
max_steps = 0              # 전체 스텝 수 (로봇별 명령 길이 중 최대)
step_inflight = set()      # 이번 스텝 명령을 보낸 로봇들(완료 대기 대상)
step_done = set()          # 이번 스텝 완료한 로봇들

# 정렬 반복 관련
alignment_pending = {}
alignment_angle= 1  # 임계각도 = 1도
alignment_dist = 1  # 임계거리 = 3cm

paused_robots = set()

def pause_robots(targets):
    """특정 로봇만 '현재 명령 완료 후' 정지."""
    for rid in targets:
        rid = str(rid)
        paused_robots.add(rid)
        client.publish(f"robot/{rid}/cmd", "S")
        print(f"🛑 [Robot_{rid}] 정지 예약(S)")

def resume_robots(targets):
    """특정 로봇만 재개."""
    for rid in targets:
        rid = str(rid)
        if rid in paused_robots:
            paused_robots.remove(rid)
        client.publish(f"robot/{rid}/cmd", "RE")
        print(f"▶ [Robot_{rid}] 재개(RE)")


def reset_all_indices():
    for rid in robot_command_map:
        robot_indices[rid] = 0
    # 수행 상태 초기화
    global inflight, paused
    inflight = {rid: False for rid in robot_command_map}
    paused = False

def set_alignment_pending(robot_id, mode):
    alignment_pending[robot_id] = {
        "mode": mode,
        "in_progress": False
    }
    print(f"▶ pending: {mode} <- Robot_{robot_id}")

def clear_alignment_pending(robot_id):
    if robot_id in alignment_pending:
        del alignment_pending[robot_id]

def check_center_alignment_ok(robot_id, dist_thresh=alignment_dist):
    tag_info = tag_info_provider()
    data = tag_info.get(int(robot_id))
    if not data or data.get("status") != "On":
        print(f"⚠️ Robot_{robot_id} 정렬용 태그 정보 없음 또는 비활성")
        return False

    dist = data.get("dist_cm", 0)
    print(f"[중앙정렬 거리 확인] Robot_{robot_id}: dist={dist:.2f} cm (기준: {dist_thresh} cm)")
    return abs(dist) <= dist_thresh

def check_north_alignment_ok(robot_id):
    if tag_info_provider is None:
        return False

    tag_info = tag_info_provider()
    tag = tag_info.get(int(robot_id))
    north = tag_info.get(NORTH_TAG_ID)

    if not tag or tag.get("status") != "On":
        print(f"⚠️ Robot_{robot_id} 태그 상태 비정상")
        return False
    if not north or north.get("status") != "On":
        print(f"⚠️ NORTH_TAG_ID={NORTH_TAG_ID} 상태 비정상")
        return False

    cur_yaw = tag.get("yaw_front_deg", None)
    north_yaw = north.get("yaw_front_deg", None)

    if cur_yaw is None or north_yaw is None:
        print(f"⚠️ yaw_front_deg 정보 없음: Robot_{robot_id} 또는 NORTH_TAG")
        return False

    # ✅ Δ 계산 (북쪽 기준 회전 오차)
    delta = ((cur_yaw - north_yaw + 180) % 360) - 180
    print(f"▶ Robot_{robot_id} Δ={delta:.2f}°, 기준: {alignment_angle:.1f}°")
    return abs(delta) < alignment_angle

def check_direction_alignment_ok(robot_id):
    """VisionSystem이 저장한 heading_offset_deg(기준각 대비 오차)를 사용해 OK 여부 판단"""
    if tag_info_provider is None:
        return False
    tag_info = tag_info_provider()
    tag = tag_info.get(int(robot_id))
    if not tag or tag.get("status") != "On":
        print(f"⚠️ Robot_{robot_id} 태그 상태 비정상")
        return False

    delta = tag.get("heading_offset_deg", None)
    if delta is None:
        print(f"⚠️ Robot_{robot_id} heading_offset_deg 없음")
        return False

    print(f"▶ Robot_{robot_id} 방향정렬 Δ={delta:.2f}°, 기준: {alignment_angle:.1f}°")
    return abs(delta) < alignment_angle


class CommandSet:
    def __init__(self, robot_id, commands):
        self.robot_id = robot_id
        self.commands = commands

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "command_count": len(self.commands),
            "command_set": [{"command": c} for c in self.commands]
        }

def set_tag_info_provider(func):
    global tag_info_provider
    tag_info_provider = func

def send_next_command(robot_id):
    idx = robot_indices[robot_id]
    commands = robot_command_map[robot_id]

    if idx < len(commands):
        cmd = commands[idx]

        # 실시간 회전 보정
        if cmd.startswith(("R", "L")) and tag_info_provider:
            tag_info = tag_info_provider()
            tag = tag_info.get(int(robot_id))
            hd = last_heading.get(robot_id, 0)
            
            if tag and "yaw_front_deg" in tag:
                delta = tag.get("heading_offset_deg", 0)

                base_angle = 90
                if (delta > 0 and cmd.startswith("R")) or (delta < 0 and cmd.startswith("L")):
                    corrected_angle = base_angle - abs(delta)
                else:
                    corrected_angle = base_angle + abs(delta)

                corrected_angle = max(0, round(corrected_angle, 1))
                corrected_cmd = f"{cmd[0]}{corrected_angle}"
                cmd = corrected_cmd

                if cmd.startswith("R"):
                    hd = (hd + 1) % 4
                elif cmd.startswith("L"):
                    hd = (hd - 1) % 4
                last_heading[robot_id] = hd

        elif cmd.startswith("T"):
            hd = last_heading.get(robot_id, 0)
            hd = (hd + 2) % 4
            last_heading[robot_id] = hd

        cs = CommandSet(robot_id, [cmd])
        payload = json.dumps({"commands": [cs.to_dict()]})
        print(f"📤 [Robot_{robot_id}] → {cmd}")
        client.publish(MQTT_TOPIC_COMMANDS_, payload)
        inflight[robot_id] = True
        robot_indices[robot_id] += 1
    else:
        print(f"✅ [Robot_{robot_id}] 모든 명령 완료")

def _send_step_commands():
    global step_inflight, step_done, active

    step_inflight = set()
    step_done = set()

    participants = [rid for rid, cmds in robot_command_map.items() if current_step < len(cmds)]
    if not participants:
        print("\n✅ [모든 명령 전송 완료] (no participants)")
        active = False
        return

    actual_targets = [rid for rid in participants if rid not in paused_robots]
    if not actual_targets:
        print(f"⏸ 모든 대상이 일시정지 → Step {current_step+1}/{max_steps} 대기")
        return

    tag_info = tag_info_provider() if tag_info_provider else {}
    north = tag_info.get(NORTH_TAG_ID, {}) if tag_info else {}

    for rid in actual_targets:
        cmd_raw = robot_command_map[rid][current_step]
        cmd = cmd_raw

        # 🔹 Stay: 전송 없이 즉시 완료 처리
        if cmd == "Stay":
            print(f"⏸ [Step {current_step+1}/{max_steps}] [Robot_{rid}] → Stay (즉시 완료)")
            inflight[rid] = False
            robot_indices[rid] = current_step + 1
            step_inflight.add(rid)   # 이번 스텝의 '대상'에는 포함
            step_done.add(rid)       # 곧바로 완료 처리
            if rid in step_wait_for_move:
                step_wait_for_move.discard(rid)
            continue

        # (아래 기존 F 전 보정/일반 전송 로직은 그대로 유지)
        dist = tag_info.get(int(rid), {}).get("dist_cm", None) if tag_info else None
        delta = tag_info.get(int(rid), {}).get("heading_offset_deg", None) if tag_info else None

        command_set = [{"command": cmd}]
        two_stage = False
        try:
            if cmd.startswith("F") and tag_info and delta is not None and abs(float(delta)) >= DIRECTION_CORR_THRESHOLD_DEG:
                angle = round(abs(float(delta)), 1)
                pre_cmd = f"{'L' if float(delta) > 0 else 'R'}{angle}_modeOnly"
                command_set = [{"command": pre_cmd}, {"command": cmd}]
                two_stage = True
        except Exception:
            command_set = [{"command": cmd}]
            two_stage = False

        payload = json.dumps({
            "commands": [{
                "robot_id": rid,
                "command_count": len(command_set),
                "command_set": command_set,
            }]
        })

        if two_stage:
            print(f"📤 [Step {current_step+1}/{max_steps}] [Robot_{rid}] → (dir-fix + {cmd})")
            step_wait_for_move.add(rid)
        else:
            print(f"📤 [Step {current_step+1}/{max_steps}] [Robot_{rid}] → {cmd}")
            if rid in step_wait_for_move:
                step_wait_for_move.discard(rid)

        client.publish(MQTT_TOPIC_COMMANDS_, payload)
        inflight[rid] = True
        robot_indices[rid] = current_step + 1
        step_inflight.add(rid)

    if step_inflight:
        print(f"▶ Step {current_step+1}/{max_steps} 전송 대상: {sorted(list(step_inflight))}")

    # 🔹 이번 스텝이 전부 'Stay'였거나, Stay만 남은 경우 즉시 다음 스텝으로
    if step_inflight and step_done >= step_inflight:
        _advance_step_if_ready()



def _advance_step_if_ready():
    """이번 스텝 대상 전원이 완료되면 다음 스텝으로."""
    global current_step, active
    if not active:
        return
    if step_inflight and step_done >= step_inflight:
        print(f"\n✅ Step {current_step+1}/{max_steps} 전체 완료 → 다음 스텝")
        current_step += 1
        if current_step >= max_steps:
            print("\n✅ [모든 명령 전송 완료] (max steps reached)")
            active = False
            return
        _send_step_commands()



def check_all_completed():
    # 배리어 모드 기준: 모든 스텝을 마치고 더 이상 전송할 것이 없는 경우
    if not active:
        return True
    if current_step >= max_steps and not step_inflight:
        return True
    return False

def get_active_robot_ids():
    """현재 시퀀스에 포함된 로봇 ID 목록(str)"""
    return list(robot_command_map.keys())

def pause_sequences():
    """전체 정지"""
    global paused
    paused = True
    targets = get_active_robot_ids()
    if not targets:
        print("⏸ 일시정지: 활성 시퀀스 없음")
        return
    print("⏸ 일시정지 예약: 현재 명령 완료 후 정지 (S 전송)")
    for rid in targets:
        client.publish(f"robot/{rid}/cmd", "S")
        print(f"🛑 [Robot_{rid}] S 전송")

def resume_sequences():
    """전체 재개"""
    global paused
    paused = False
    targets = get_active_robot_ids()
    if not targets:
        print("▶ 재개: 활성 시퀀스 없음")
        return
    print("▶ 재개: RE 전송 & 남은 명령 이어서 진행")
    for rid in targets:
        client.publish(f"robot/{rid}/cmd", "RE")

def on_message(client, userdata, msg):
    global active, inflight, paused_robots  # paused 대신 paused_robots 사용

    payload = msg.payload.decode("utf-8", "ignore")

    # DONE 토픽만 처리
    if msg.topic != DONE_TOPIC:
        return

    # --------------------------------------------------------------------------------
    # 1) DONE 메시지 파싱
    # --------------------------------------------------------------------------------
    if payload.startswith("DONE;Robot_"):
        parts = payload.split(";")
        try:
            # 예: ["DONE","Robot_1","cmd=MOVE","mode=straight"]
            robot_id = parts[1].split("_", 1)[1]  # "1"
            cmd_info = parts[2] + (f";{parts[3]}" if len(parts) > 3 else "")
        except Exception as e:
            print(f"[DONE 파싱 오류] {payload} / {e}")
            return

        print(f"✅ [Robot_{robot_id}] 명령 ({cmd_info}) 완료")

        # 1-1) 현재 명령 완료 → inflight 해제
        if inflight is not None:
            inflight[robot_id] = False

        # --------------------------------------------------------------------------------
        # 2) 정렬 반복 처리 (modeOnly: north / direction)
        # --------------------------------------------------------------------------------
        if "mode=modeOnly" in payload and robot_id in alignment_pending:
            info = alignment_pending[robot_id]
            mode = info["mode"]
            in_progress = info.get("in_progress", False)

            # 북쪽 정렬 반복
            if mode == "north":
                if check_north_alignment_ok(robot_id):
                    print(f"✅ 북쪽 정렬 완료: Robot_{robot_id}")
                    clear_alignment_pending(robot_id)
                    return

                if in_progress:
                    print(f"⚠️ 이미 반복 중 → 건너뜀: Robot_{robot_id}")
                    return

                alignment_pending[robot_id]["in_progress"] = True

                def repeat_wrapper():
                    time.sleep(alignment_delay_sec)
                    if check_north_alignment_ok(robot_id):
                        print(f"✅ 북쪽 정렬 완료 (지연 후 재확인): Robot_{robot_id}")
                        clear_alignment_pending(robot_id)
                        if all(info["mode"] != "north" for info in alignment_pending.values()):
                            print("✅ 모든 로봇 북쪽정렬 완료")
                        return

                    tag_info = tag_info_provider()
                    send_north_align(
                        client, tag_info, MQTT_TOPIC_COMMANDS_, NORTH_TAG_ID,
                        targets=[int(robot_id)],
                        alignment_pending=alignment_pending
                    )
                    if robot_id in alignment_pending:
                        alignment_pending[robot_id]["in_progress"] = False

                threading.Thread(target=repeat_wrapper, daemon=True).start()

            # 방향 정렬 반복
            elif mode == "direction":
                if check_direction_alignment_ok(robot_id):
                    print(f"✅ 방향정렬 완료: Robot_{robot_id}")
                    clear_alignment_pending(robot_id)
                    return

                if in_progress:
                    print(f"⚠️ 이미 방향정렬 반복 중 → 건너뜀: Robot_{robot_id}")
                    return

                alignment_pending[robot_id]["in_progress"] = True

                def repeat_wrapper_dir():
                    time.sleep(alignment_delay_sec)
                    if check_direction_alignment_ok(robot_id):
                        print(f"✅ 방향정렬 완료 (지연 후 재확인): Robot_{robot_id}")
                        clear_alignment_pending(robot_id)
                        return

                    tag_info = tag_info_provider()
                    from align import send_direction_align
                    send_direction_align(
                        client, tag_info, MQTT_TOPIC_COMMANDS_,
                        targets=[int(robot_id)],
                        alignment_pending=alignment_pending
                    )
                    if robot_id in alignment_pending:
                        alignment_pending[robot_id]["in_progress"] = False

                threading.Thread(target=repeat_wrapper_dir, daemon=True).start()
                
        # 3) 중앙 정렬 처리: 반복만, 후속 명령 전송 없음
        if robot_id in alignment_pending:
            info = alignment_pending[robot_id]
            if info["mode"] == "center":
                in_progress = info.get("in_progress", False)

                # 완료 조건
                if check_center_alignment_ok(robot_id):
                    print(f"✅ 중앙정렬 완료: Robot_{robot_id}")
                    clear_alignment_pending(robot_id)
                    # 모두 끝났는지 안내 (옵션)
                    if all(i["mode"] != "center" for i in alignment_pending.values()):
                        print("✅ 모든 로봇 중앙정렬 완료")
                    return

                # 이미 반복 중이면 스킵
                if in_progress:
                    print(f"⚠️ 이미 반복 중 → 건너뜀: Robot_{robot_id}")
                    return

                # 재시도 스케줄
                alignment_pending[robot_id]["in_progress"] = True
                def repeat_wrapper_center():
                    time.sleep(alignment_delay_sec)
                    if check_center_alignment_ok(robot_id):
                        print(f"✅ 중앙정렬 완료 (지연 후 재확인): Robot_{robot_id}")
                        clear_alignment_pending(robot_id)
                        return
                    tag_info = tag_info_provider()
                    send_center_align(
                        client, tag_info, MQTT_TOPIC_COMMANDS_,
                        targets=[int(robot_id)],
                        alignment_pending=alignment_pending
                    )
                    if robot_id in alignment_pending:
                        alignment_pending[robot_id]["in_progress"] = False

                threading.Thread(target=repeat_wrapper_center, daemon=True).start()
                return


        # --------------------------------------------------------------------------------
        # 4) 로봇별 일시정지 상태면 다음 전송 보류
        # --------------------------------------------------------------------------------
        if robot_id in paused_robots:
            total = len(robot_command_map.get(robot_id, []))
            sent = robot_indices.get(robot_id, 0)
            print(f"⏸ [Robot_{robot_id}] 개별 일시정지 상태 → 다음 전송 보류 (완료={sent}/{total})")

        # --------------------------------------------------------------------------------
        # 5) 일반 명령 진행
        # --------------------------------------------------------------------------------
        if active and robot_id in step_inflight:
            # (회전 modeOnly → 직진) 두 단계 전송한 경우: MOVE DONE에서만 카운트
            if robot_id in step_wait_for_move:
                if "cmd=MOVE" in payload:
                    step_done.add(robot_id)
                    inflight[robot_id] = False
                    step_wait_for_move.discard(robot_id)
                    print(f"🟢 [Step {current_step+1}/{max_steps}] 완료 수신(MOVE 최종): {sorted(step_done)} / {sorted(step_inflight)}")
                    _advance_step_if_ready()

            # 일반 케이스: DONE 즉시 카운트
            else:
                step_done.add(robot_id)
                inflight[robot_id] = False
                print(f"🟢 [Step {current_step+1}/{max_steps}] 완료 수신: {sorted(step_done)} / {sorted(step_inflight)}")
                _advance_step_if_ready()



client = None  # 전역


def init_mqtt_client():
    """단일 MQTT 클라이언트 생성 + 콜백 바인딩 + 구독"""
    global client
    if client is not None:
        return client

    import paho.mqtt.client as mqtt
    c = mqtt.Client()

    # ★ 콜백 바인딩
    c.on_connect = on_connect
    c.on_message = on_message

    # 접속 및 루프 시작
    c.connect(IP_address_, MQTT_PORT, 60)
    c.loop_start()

    # on_connect에서 subscribe하지만, 안전하게 한 번 더
    c.subscribe(DONE_TOPIC)

    client = c
    print(f"[MQTT] connected and subscribed: {IP_address_}:{MQTT_PORT}, topic='{DONE_TOPIC}'")
    return client

def on_connect(client, userdata, flags, rc):
    client.subscribe(DONE_TOPIC)
    
def start_sequence(cmd_map):
    global robot_command_map, active, last_heading, inflight, paused, paused_robots
    global current_step, max_steps, step_inflight, step_done, robot_indices

    robot_command_map = cmd_map

    # 인덱스/상태 초기화
    robot_indices = {rid: 0 for rid in robot_command_map}
    last_heading = {rid: 0 for rid in robot_command_map}
    paused_robots.clear()
    inflight = {rid: False for rid in robot_command_map}
    paused = False
    active = True

    # 배리어 스텝 수 = 로봇별 명령 길이의 최대
    max_steps = max((len(v) for v in robot_command_map.values()), default=0)
    current_step = 0
    step_inflight = set()
    step_done = set()
    step_wait_for_move.clear()

    if max_steps == 0:
        print("⚠️ 전송할 명령이 없습니다.")
        active = False
        return

    print(f"▶ 배리어 모드 시작: 총 스텝 {max_steps}, 대상 {sorted(list(robot_command_map.keys()))}")
    _send_step_commands()




def start_auto_sequence(client, tag_info, PRESET_IDS, agents, MQTT_TOPIC_COMMANDS_, NORTH_TAG_ID,
                                  set_alignment_pending, alignment_pending,
                                  check_center_alignment_ok, check_direction_alignment_ok,
                                  send_center_align,
                                  compute_cbs, check_all_completed):

    import threading, time
    from align import send_direction_align  # ← 여기서 import

    def run_sequence():
        print("▶ 자동 시퀀스 시작: 중앙정렬 → 방향정렬 → 경로수행 → 재중앙정렬")

        # 1) 중앙정렬
        for rid in PRESET_IDS:
            rid_str = str(rid)
            if not check_center_alignment_ok(rid_str):
                set_alignment_pending(rid_str, "center")
                send_center_align(client, tag_info, MQTT_TOPIC_COMMANDS_,
                                  targets=[int(rid)], alignment_pending=alignment_pending)
            else:
                print(f"✅ Robot_{rid} 중앙정렬 이미 완료")

        while any(str(rid) in alignment_pending and alignment_pending[str(rid)]["mode"] == "center"
                  for rid in PRESET_IDS):
            time.sleep(0.2)
        print("✅ [1/3] 모든 로봇 중앙정렬 완료")

        # 2) 방향정렬
        for rid in PRESET_IDS:
            rid_str = str(rid)
            if not check_direction_alignment_ok(rid_str):
                set_alignment_pending(rid_str, "direction")
                send_direction_align(client, tag_info, MQTT_TOPIC_COMMANDS_,
                                     targets=[int(rid)], alignment_pending=alignment_pending)
            else:
                print(f"✅ Robot_{rid} 방향정렬 이미 완료")

        while any(str(rid) in alignment_pending and alignment_pending[str(rid)]["mode"] == "direction"
                  for rid in PRESET_IDS):
            time.sleep(0.2)
        print("✅ [2/3] 모든 로봇 방향정렬 완료")

        # 3) 경로 수행
        if all(a.start and a.goal for a in agents):
            compute_cbs()
        else:
            print("⚠️ 시작 또는 도착 지정되지 않은 에이전트 있음 → 중단")
            return
        

        print("✅ [3/3] 모든 명령 '전송' 완료")

    threading.Thread(target=run_sequence, daemon=True).start()

