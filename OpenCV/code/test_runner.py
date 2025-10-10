# test_runner.py (갱신본)

import json
import time
import paho.mqtt.client as mqtt
import keyboard  # pip install keyboard

MQTT_SERVER = "192.168.0.25"
MQTT_PORT = 1883
TRANSFER_TOPIC = "command/transfer"
DONE_TOPIC = "robot/done"

client = None

# 로봇별 전체 세트 & 완료 인덱스 & 수행중 여부
command_sets = {}        # rid -> [cmd, ...]
completed_idx = {}       # rid -> int (DONE 받을 때만 +1)
inflight = {}            # rid -> bool (현재 명령 수행중인지)
paused = False
last_space_time = 0

# active_robots = ["3", "4"]

def get_test_command_map():
    return {
        "1": ["F20_modeA","R90_modeOnly","R90_modeOnly","F20_modeA"],
        # "3": ["F20_modeA","R90_modeOnly","R90_modeOnly","F20_modeA"],
    }

def init_mqtt():
    global client
    client = mqtt.Client("TestRunner")
    client.on_message = on_message
    client.connect(MQTT_SERVER, MQTT_PORT, 60)
    client.subscribe(DONE_TOPIC)
    client.subscribe("robot/ack")
    client.loop_start()

def build_single_payload(robot_id: str, command: str) -> str:
    return json.dumps({
        "commands": [{
            "robot_id": robot_id,
            "command_count": 1,
            "command_set": [{"command": command}]
        }]
    })

def send_one(robot_id: str):
    """completed_idx 기준으로 다음 1개 전송(이미 완료한 것 이후부터)."""
    cmds = command_sets.get(robot_id, [])
    idx = completed_idx.get(robot_id, 0)
    if idx >= len(cmds):
        inflight[robot_id] = False
        print(f"✅ [Robot_{robot_id}] 남은 명령 없음 (완료={idx}/{len(cmds)})")
        return
    cmd = cmds[idx]
    payload = build_single_payload(robot_id, cmd)
    client.publish(TRANSFER_TOPIC, payload)
    inflight[robot_id] = True
    print(f"📤 [Robot_{robot_id}] → {cmd}  (idx={idx})")

def start_sequence(cmd_map: dict):
    global command_sets, completed_idx, inflight, paused
    command_sets = {str(r): list(v) for r, v in cmd_map.items()}
    completed_idx = {rid: 0 for rid in command_sets}
    inflight = {rid: False for rid in command_sets}
    paused = False
    print(f"▶ 순차 전송 시작: {command_sets}")
    # 각 로봇에 첫 1개만 전송
    for rid in command_sets:
        send_one(rid)

def on_message(client_, userdata, msg):
    global paused
    topic = msg.topic
    payload = msg.payload.decode(errors="ignore")

    if topic == DONE_TOPIC and payload.startswith("DONE;Robot_"):
        # 예: DONE;Robot_4;cmd=MOVE;mode=straight
        try:
            robot_id = payload.split(";")[1].split("_")[1]
        except Exception:
            print(f"⚠️ DONE 파싱 실패: {payload}")
            return

        print(f"✅ [Robot_{robot_id}] 완료 수신: {payload}")
        inflight[robot_id] = False
        # ★ 여기서만 완료 인덱스를 증가시킴 (보낸 시점이 아니라 완료 기준)
        completed_idx[robot_id] = completed_idx.get(robot_id, 0) + 1

        if paused:
            print(f"⏸ [Robot_{robot_id}] 일시정지 중 → 다음 전송 보류 (완료={completed_idx[robot_id]}/{len(command_sets.get(robot_id, []))})")
            return

        # 재개 상태면 즉시 다음 1개 전송
        send_one(robot_id)

    elif topic == "robot/ack":
        try:
            data = json.loads(payload)
            if data.get("type") == "ACK":
                rid = data.get("robot_id")
                rx = data.get("received")
                if rx == "S":
                    print(f"🛑 [Robot_{rid}] 정지 예약 ACK")
                elif rx == "RE":
                    print(f"▶ [Robot_{rid}] 재개 ACK")
        except json.JSONDecodeError:
            print(f"⚠️ 잘못된 JSON: {payload}")

if __name__ == "__main__":
    init_mqtt()
    print("▶ 키 안내: SPACE=시작 | T=일시정지(현재 명령 끝나면 정지) | Y=재개 | Q=종료")

    try:
        while True:
            now = time.time()

            if keyboard.is_pressed("space") and (now - last_space_time > 1.0):
                last_space_time = now
                cmd_map = get_test_command_map()
                targets = list(cmd_map.keys())

                print("▶ 명령 시작 (RE 전송 후 첫 명령)")
                # 1) 먼저 RE 전송
                for rid in targets:
                    client.publish(f"robot/{rid}/cmd", "RE")
                time.sleep(0.1)  # 수신 상태 전환 여유

                # 2) 시퀀스 시작(각 로봇에 첫 명령 1개씩 전송)
                start_sequence(cmd_map)
                time.sleep(0.2)

            elif keyboard.is_pressed("t"):
                # 현재 실행 중인 명령만 끝내고 멈추기 위해 각 로봇에 S 전송
                paused = True
                print("⏸ 일시정지 예약: 현재 명령 완료 후 정지 (S 전송)")
                targets = list(command_sets.keys())
                for rid in targets:
                    client.publish(f"robot/{rid}/cmd", "S")
                time.sleep(0.25)

            elif keyboard.is_pressed("y"):
                # 재개: RE 전송 후, inflight=False 인 로봇들부터 다음 1개 재전송
                paused = False
                print("▶ 재개: RE 전송 & 남은 명령 이어서 진행")
                targets = list(command_sets.keys())
                for rid in targets:
                    client.publish(f"robot/{rid}/cmd", "RE")
                time.sleep(0.1)
                for rid in targets:
                    if not inflight.get(rid, False):
                        send_one(rid)
                time.sleep(0.1)


            elif keyboard.is_pressed("q"):
                print("▶ 종료")
                break

            time.sleep(0.05)
    finally:
        if client:
            client.loop_stop()
            client.disconnect()
