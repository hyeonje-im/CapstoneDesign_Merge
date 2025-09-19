# fake_mqtt.py
class FakeMQTTBroker:
    def __init__(self):
        self.subscribers = dict()

    def subscribe(self, topic, callback):
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        self.subscribers[topic].append(callback)

    def publish(self, topic, message):
        if topic in self.subscribers:
            for callback in self.subscribers[topic]:
                callback(message)

    def send_command_sets(self, command_sets):
        for cs in command_sets:
            topic = f"robot/{cs.robot_id}/move"
            commands = [cmd["command"] for cmd in cs.to_dict()["command_set"]]
            self.publish(topic, commands)  # 리스트 그대로 publish



