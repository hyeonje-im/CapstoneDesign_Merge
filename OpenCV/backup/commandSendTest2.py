import time
import json
import paho.mqtt.client as mqtt

MQTT_SERVER = "192.168.159.132"
MQTT_PORT = 1883
TRANSFER_TOPIC = "command/transfer"

"""
수정 리스트
46줄 commands.append("S")  # ✅ 정지 명령 추가        << !수정됨!
"""


# 명령세트를 체계적으로 관리하기 위한 클래스 정의
class CommandSet:
    def __init__(self, robot_id, path, initial_dir="north"):
        """
        robot_id: 제어할 로봇의 ID (문자열)
        path: 로봇의 경로 (예: [(0, 0), (0, 1), (1, 1)])
        initial_dir: 로봇의 초기 방향 (예: "north")
        """
        self.robot_id = robot_id
        # self.robot_id = "4" #테스트용으로 4로 고정
        self.path = path
        self.initial_dir = initial_dir
        self.commands = self.path_to_commands()
        self.command_count = len(self.commands)

    def path_to_commands(self):
        """
        경로 데이터를 명령 리스트로 변환합니다.
        """
        if not self.path or len(self.path) < 2:
            return []

        commands = []
        current_dir = self.initial_dir

        for i in range(len(self.path) - 1):
            pos1 = self.path[i]
            pos2 = self.path[i + 1]
            
            #!!! 일단 제자리 이동은 생략
            if pos1 == pos2:
                commands.append("S")  # ✅ 정지 명령 추가        << !수정됨!
                continue  # 제자리 이동 생략

            # 방향 계산
            next_dir = self.direction_between(pos1, pos2)

            # 방향 전환 명령 추가
            if current_dir != next_dir:
                commands.append(self.turn_command(current_dir, next_dir))
                current_dir = next_dir

            # 이동 명령 추가
            commands.append(f"D{self.distance_between(pos1, pos2)}")

        return commands

    def direction_between(self, pos1, pos2):
        """
        두 좌표 간의 방향을 계산합니다.
        """
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
        """
        현재 방향에서 다음 방향으로 전환하기 위한 명령을 반환합니다.
        """
        directions = ["north", "east", "south", "west"]
        current_idx = directions.index(current_dir)
        next_idx = directions.index(next_dir)

        if (current_idx + 1) % 4 == next_idx:
            return "R"  # 오른쪽 회전
        elif (current_idx - 1) % 4 == next_idx:
            return "L"  # 왼쪽 회전
        else:
            return "R R"  # 180도 회전

    def distance_between(self, pos1, pos2):
        """
        두 좌표 간의 거리를 계산합니다.
        """
        r1, c1 = pos1
        r2, c2 = pos2
        return abs(r1 - r2) + abs(c1 - c2)

    def convert_command(self, cmd):
        """
        간결한 형식(D숫자)을 "distance=숫자" 형식으로 변환합니다.
        """
        if cmd.startswith("D"):
            try:
                value = float(cmd[1:])
                if value.is_integer():
                    value = int(value)
                return f"distance={value}"
            except Exception:
                return cmd
        return cmd

    def to_dict(self):
        """
        전체 명령세트를 딕셔너리 형태로 반환합니다.
        """
        structured_commands = []
        for cmd in self.commands:
            converted_cmd = self.convert_command(cmd)
            structured_commands.append({"command": converted_cmd})
        return {
            "robot_id": self.robot_id,
            "command_count": self.command_count,
            "command_set": structured_commands
        }

    @classmethod
    def send_command_sets(cls, command_sets):
        """
        CommandSet 객체 리스트를 받아 MQTT를 통해 전송합니다.
        """
        try:
            client = mqtt.Client()
            client.connect(cls.MQTT_SERVER, cls.MQTT_PORT, 60)

            message = {"commands": [cs.to_dict() for cs in command_sets]}
            payload = json.dumps(message)

            print("전송 모듈 명령 세트:", payload)
            client.publish(cls.TRANSFER_TOPIC, payload)
            time.sleep(1)
            client.disconnect()

        except Exception as e:
            print(f"로봇 통신 실패: {e}")







# # 로봇 2와 로봇 4의 명령세트 생성
# command_set_robot2 = CommandSet("2", ["D30", "R", "D30", "R", "D30", "R", "D30"])
# command_set_robot4 = CommandSet("4", ["D30", "R", "D30", "R", "D30", "R", "D30"])

# # 여러 로봇의 명령세트를 하나의 메시지로 구성
# message = {
#     "commands": [
#         command_set_robot2.to_dict(),
#         command_set_robot4.to_dict()
#     ]
# }

# client = mqtt.Client()
# client.connect(MQTT_SERVER, MQTT_PORT, 60)

# payload = json.dumps(message)
# print("전송 모듈 명령 세트:", payload)
# client.publish(TRANSFER_TOPIC, payload)
# time.sleep(1)
# client.disconnect()
