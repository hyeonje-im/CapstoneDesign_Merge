import time
import threading
import json
import queue
import paho.mqtt.client as mqtt
import math  
from config import  MQTT_TOPIC_COMMANDS_ , MQTT_PORT ,IP_address_

class CommandSender:
    def __init__(self, robot_id, mqtt_client):
        self.robot_id      = robot_id
        self.mqtt_client   = mqtt_client
        self.cmd_topic     = f"robot/{robot_id}/cmd"
        self.command_queue = queue.Queue()
        self.lock          = threading.RLock()

    def flush_commands(self):
        with self.lock:
            while not self.command_queue.empty():
                self.command_queue.get_nowait()

    def add_command(self, cmd):
        self.command_queue.put(cmd)
        print(f"[Robot {self.robot_id}] 큐에 추가: {cmd}")

    def add_commands(self, cmds):
        for cmd in cmds:
            self.add_command(cmd)

    def send_all_commands(self):
        with self.lock:
            while not self.command_queue.empty():
                cmd = self.command_queue.get_nowait()
                if cmd.startswith("wait="):
                    try:
                        wt = float(cmd.split("=",1)[1])
                    except:
                        wt = 0
                    time.sleep(wt)
                    continue
                print(f"[{self.robot_id}] ▶ {cmd}")
                self.mqtt_client.publish(self.cmd_topic, cmd)
                time.sleep(0.05)

def on_connect(client, userdata, flags, rc):
    print("MQTT 접속 결과:", rc)
    client.subscribe(MQTT_TOPIC_COMMANDS_)
    client.subscribe("robot/+/error")
    

def on_message(client, userdata, msg):
    topic = msg.topic
    data = msg.payload.decode()

    # 1) CBS로부터 일반 명령 세트 수신
    if topic == MQTT_TOPIC_COMMANDS_:
               # ── 긴급정지 처리 ──
        if data == "S":
            print("!! Emergency Stop Received !!")
            # 모든 로봇의 대기 큐 비우고 즉시 정지 명령 전송
            for rid, ctrl in userdata['robots'].items():
                ctrl.flush_commands()
                # 필요하다면 아래 명령을 실제 정지용 프로토콜로 바꿔주세요
                ctrl.add_command("EMERGENCY_STOP")  
                ctrl.send_all_commands()
            return
        
        
        try:
            js = json.loads(data)
            for o in js.get("commands", []):
                rid = o.get("robot_id")
                cs  = o.get("command_set")
                if rid in userdata['robots'] and cs:
                    ctrl = userdata['robots'][rid]
                    ctrl.flush_commands()
                    ctrl.add_command("wait=0.5")

                    # 각 명령(item) 파싱
                    for item in cs:
                        raw_cmd = item.get("command")
                        if isinstance(raw_cmd, dict):
                            cmd = raw_cmd.get("command", "")
                        else:
                            cmd = raw_cmd

                        if not isinstance(cmd, str):
                            continue

                        # 1) 회전 명령 처리 (L90/R90)
                        if (cmd.startswith("R") or cmd.startswith("L")):
                            ctrl.add_command(cmd)
                            continue

                        # 2) 전진 명령 처리 (F<거리>[_modeX])
                        if cmd.startswith("F"):
                            payload = cmd[1:]
                            # 거리와 suffix 분리
                            if "_" in payload:
                                dist_str, suffix = payload.split("_", 1)
                            else:
                                dist_str, suffix = payload, ""
                            try:
                                dist = float(dist_str)
                            except ValueError:
                                continue

                            # suffix에 따른 mode 설정
                            if suffix == "modeA":
                                mode = "straight"
                            elif suffix == "modeB":
                                mode = "pre_rotate_90"
                            elif suffix == "modeC":
                                mode = "pre_rotate_180"
                            else:
                                mode = "straight"

                            # MOVE 명령 전송
                            ctrl.add_command(f"MOVE;dist={dist:.1f};mode={mode}")
                            continue

                        # 3) 기타 명령 처리
                        ctrl.add_command(cmd)

                    # 명령 전송 플래그 및 실제 전송
                    userdata['pending_auto'][rid] = True
                    ctrl.send_all_commands()
        except Exception as e:
            print("command/transfer 파싱 오류:", e)
        return


def main():
    client = mqtt.Client(userdata={})
    robots = {rid: CommandSender(rid, client) for rid in ["1","2","3","4"]}
    client.user_data_set({
        'robots':       robots,
        'errors':       {rid: {'dx':0,'dy':0,'dtheta':0} for rid in robots},
        'pending_auto': {rid: False for rid in robots},
    })
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(IP_address_, MQTT_PORT, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
