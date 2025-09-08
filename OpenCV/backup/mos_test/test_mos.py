import paho.mqtt.client as mqtt

# 브로커 연결 시 호출되는 콜백 함수
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("my/topic")  # 구독할 토픽

# 메시지 수신 시 호출되는 콜백 함수
def on_message(client, userdata, msg):
    print(f"Received message: {msg.payload.decode()} on topic {msg.topic}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# 브로커에 연결 (브로커 IP 주소로 변경)
client.connect("10.50.33.54", 1883, 60)

# 메시지 루프 시작 (백그라운드로 지속)
client.loop_start()

# 메시지 발행 예시
client.publish("my/topic", "Hello, from Python!")

# 프로그램이 바로 종료되지 않도록 잠시 대기
import time
time.sleep(4)
client.loop_stop()
client.disconnect()
