
import json
import time
import paho.mqtt.client as mqtt
import math
from config import  IP_address_ , MQTT_PORT

# — MQTT 브로커 설정 — (command_transfer2.py와 동일)
CLIENT_ID   = "ErrorPublisher"

last_errors = {}

# — (dx, dy, dtheta)
ERRORS = {
    "2": {"dx": -2,   "dy":  5,   "dtheta": 30},   # 예: 30도만큼 시계방향 오차
    "4": {"dx":  2.5, "dy": -1.2, "dtheta": -15},  # 예: -15도만큼 반시계방향 오차
}

def publish_error(client, robot_id, dx, dy, dtheta):
    topic   = f"robot/{robot_id}/error"
    payload = json.dumps({
        "dx": dx,
        "dy": dy,
        "dtheta": dtheta
    })
    client.publish(topic, payload)
    print(f"[Publish] {topic} → {payload}")

def main():
    client = mqtt.Client(CLIENT_ID)
    client.connect(IP_address_, MQTT_PORT, keepalive=60)
    client.loop_start()

    for robot_id, err in ERRORS.items():
        dx, dy, dtheta = err["dx"], err["dy"], err["dtheta"]
        publish_error(client, robot_id, dx, dy, dtheta)
        time.sleep(0.1)

    time.sleep(0.5)
    client.loop_stop()
    client.disconnect()
    print("모든 오차 발행 완료. 종료합니다.")

def quantize_direction(yaw):
    directions = {
        "north": 90,
        "east": 0,
        "south": -90,
        "west": 180
    }
    def angular_distance(a, b):
        d = (a - b + 180) % 360 - 180
        return abs(d)
    return min(directions.items(), key=lambda kv: angular_distance(yaw, kv[1]))[0]

def compute_and_publish_errors(tag_info, agents):
    global last_errors
    last_errors.clear() 
    try:
        client = mqtt.Client(CLIENT_ID)
        client.connect(IP_address_, MQTT_PORT, keepalive=60)
        client.loop_start()
        mqtt_connected = True
    except Exception as e:
        print(f"[MQTT 연결 실패] MQTT 전송 없이 print로 대체합니다. 원인: {e}")
        client = None
        mqtt_connected = False

    payload_summary = {}

    for agent in agents:
        if agent.id not in tag_info:
            print(f"[Skip] Agent {agent.id} not in tag_info")
            continue

        data = tag_info[agent.id]
        smoothed_pos = data.get("smoothed_coordinates")
        if not smoothed_pos:
            continue

        yaw = data.get("rotation", [0, 0, 0])[2]
        target_dir = quantize_direction(yaw)
        agent.direction = target_dir

        # 정규화된 기준 각도 (4방위 중 가장 가까운 각도로 맞춤)
        canonical_angles = {
            "north": 90,
            "east": 0,
            "south": -90,
            "west": 180
        }
        desired_angle = canonical_angles[target_dir]
        dtheta = (desired_angle - yaw + 180) % 360 - 180  # -180~180 정규화

        payload = {
            "dx": 0,
            "dy": 0,
            "dtheta": round(dtheta, 1)
        }

        topic = f"robot/{agent.id}/error"
        payload_summary[str(agent.id)] = payload

        if mqtt_connected:
            publish_error(client, agent.id, **payload)
            time.sleep(0.05)
        else:
            print(f"[Simulated Publish] {topic} → {json.dumps(payload)}")

    if mqtt_connected:
        client.loop_stop()
        client.disconnect()

    print("!!!전송 또는 시뮬레이션된 방향 보정 payload:")
    print(json.dumps({"errors": payload_summary}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

