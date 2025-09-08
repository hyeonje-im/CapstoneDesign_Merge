import paho.mqtt.client as mqtt
import time

# 컴퓨터2(브로커)가 실행 중인 IP 주소
broker_address = "10.50.33.54"  # 컴퓨터2의 실제 IP 주소로 변경하세요
topic = "robot/move"

# MQTT 클라이언트 생성 (클라이언트 ID 지정 가능)
client = mqtt.Client("CommandPublisher")

# 브로커 연결
client.connect(broker_address, 1883, 60)

def send_command(command):
    print("Sending command:", command)
    client.publish(topic, command)
    time.sleep(1)  # 명령 간에 약간의 간격을 줍니다.

# 원하는 명령을 순서대로 발행 (예시)
send_command("forward")
send_command("left")
send_command("right")
send_command("backward")
send_command("stop")

client.disconnect()
