import time, json
import paho.mqtt.client as mqtt
from datetime import datetime

MQTT_SERVER = "192.168.140.132"
MQTT_PORT = 1883
TRANSFER_TOPIC = "command/transfer"

# 명령세트를 체계적으로 관리하기 위한 클래스 정의
class CommandSet:
    def __init__(self, robot_id, path, initial_dir="north", buffer_time=0):
        self.robot_id = robot_id
        self.path = path
        self.initial_dir = initial_dir

        self.buffer_time = buffer_time
        self.start_ts = time.time()

        self.commands = self.path_to_commands()

        # self.commands.insert(0, f"T{self.buffer_time}")
        # ts = datetime.now().strftime("%H%M%S")
        # self.commands.insert(0, f"CT{ts}")
        # self.command_count = len(self.commands)

    def path_to_commands(self):
        if not self.path or len(self.path) < 2:
            return []

        commands = []
        current_dir = self.initial_dir
        i = 0
        N = len(self.path)

        while i < N - 1:
            pos1 = self.path[i]
            pos2 = self.path[i + 1]

            # 정지
            if pos1 == pos2:
                commands.append("D0")
                i += 1
                continue

            next_dir = self.direction_between(pos1, pos2)
            # 방향 전환 시턴 명령만 보내고, 이후 수신기에서 자동 직진
            if current_dir != next_dir:
                commands.append(self.turn_command(current_dir, next_dir))
                current_dir = next_dir
                # 첫 단계 이동은 수신기가 처리하므로 인덱스만 증가
                i += 1
                continue

            # 같은 방향으로 연속 이동: 한 칸당 D10 명령 반복
            steps = 0
            while i < N - 1 and self.direction_between(self.path[i], self.path[i + 1]) == current_dir:
                steps += 1
                i += 1
            for _ in range(steps):
                commands.append("D10")

        return commands

    def direction_between(self, pos1, pos2):
        r1, c1 = pos1
        r2, c2 = pos2
        if r1 == r2 and c1 + 1 == c2:
            return "east"
        elif r1 == r2 and c1 - 1 == c2:
            return "west"
        elif c1 == c2 and r1 + 1 == r2:
            return "south"
        elif c1 == c2 and r1 - 1 == r2:
            return "north"
        else:
            raise ValueError(f"Invalid move from {pos1} to {pos2}")

    def turn_command(self, current_dir, next_dir):
        directions = ["north", "east", "south", "west"]
        cur = directions.index(current_dir)
        nxt = directions.index(next_dir)
        delta = ((nxt - cur + 2) % 4) - 2
        angle = delta * 90
        return f"R{angle}"

    def convert_command(self, cmd):
        return cmd

    def to_dict(self):
        structured = []
        cumulative = 0.0
        for cmd in self.commands:
            # ② 누적 시간 계산
            dur = self._cmd_duration(cmd)
            cumulative += dur

            # ③ 절대시각 계산 (명령이 끝나야 할 epoch)
            at_ts = self.start_ts + cumulative
            dt = datetime.fromtimestamp(at_ts)
            hh = f"{dt.hour:02d}"
            mm = f"{dt.minute:02d}"
            ss = f"{dt.second:02d}"
            mmm = f"{int(dt.microsecond/1000):03d}"
            at_str = hh + mm + ss + mmm   # "HHMMSSmmm"

            # ④ 커맨드에 붙여서 전송
            structured.append({
                "command": f"{cmd}@{at_str}"
            })

        return {
            "robot_id": self.robot_id,
            "command_count": len(structured),
            "command_set": structured
        }
    
    def _cmd_duration(self, cmd: str) -> float:
        return CELL_TIME + self.buffer_time


    @classmethod
    def send_command_sets(cls, command_sets):
        try:
            client = mqtt.Client()
            client.connect(MQTT_SERVER, MQTT_PORT, 1)
            message = {"commands": [cs.to_dict() for cs in command_sets]}
            payload = json.dumps(message)
            print("전송 모듈 명령 세트:", payload)
            client.publish(TRANSFER_TOPIC, payload)
            client.disconnect()
        except Exception as e:
            print(f"(경고) 로봇 통신 실패: {e}")
