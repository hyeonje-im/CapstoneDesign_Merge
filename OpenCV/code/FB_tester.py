import cv2
import numpy as np

import paho.mqtt.client as mqtt
import time
import json

from vision.apriltag import AprilTagDetector, tag_pose
from vision.camera import camera_open, frame_process
from config import camera_matrix, dist_coeffs, object_points

MQTT_BROKER = "192.168.0.25"
MQTT_PORT = 1883
ROBOT_ID = "1"

try:
    client = mqtt.Client("TagController")
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()
    mqtt_connected = True
except Exception as e:
    print(f"[MQTT 연결 실패] 메시지는 print만 수행합니다. 이유: {e}")
    client = None
    mqtt_connected = False


def publish_motion(dx, dy, dtheta):
    topic = f"robot/{ROBOT_ID}/error"
    payload = json.dumps({"dx": dx, "dy": dy, "dtheta": dtheta})
    if mqtt_connected:
        client.publish(topic, payload)
    print(f"[Publish] {topic} → {payload}")


def compute_tag_offset_xyz(tag, frame_shape, camera_matrix, dist_coeffs, object_points):
    rvec, tvec = tag_pose(tag, object_points, camera_matrix)  # tvec: (3,1)
    rmat, _ = cv2.Rodrigues(rvec)

    # 월드 좌표계에서 카메라 기준 태그 위치 (Z가 깊이, X/Y는 좌우/상하)
    x_cm = float(tvec[0][0]) * 100
    y_cm = float(tvec[1][0]) * 100
    z_cm = float(tvec[2][0]) * 100

    _, _, _, _, _, _, angles = cv2.decomposeProjectionMatrix(np.hstack((rmat, tvec)))
    yaw, pitch, roll = angles.flatten()

    return x_cm, y_cm, z_cm, yaw, pitch, roll


def estimate_tag_offset_cm(tag, frame_shape, tag_size_cm=3.2):
    # 1. 영상 중심과 태그 중심 간 픽셀 거리 계산
    img_h, img_w = frame_shape[:2]
    frame_center = np.array([img_w / 2, img_h / 2])
    tag_center = np.array(tag.center)
    offset_px = tag_center - frame_center

    # 2. 태그 한 변 길이 측정 → 픽셀당 cm 계산
    corners = tag.corners
    edge_lengths = [
        np.linalg.norm(corners[0] - corners[1]),
        np.linalg.norm(corners[1] - corners[2]),
        np.linalg.norm(corners[2] - corners[3]),
        np.linalg.norm(corners[3] - corners[0])
    ]
    avg_edge_px = np.mean(edge_lengths)
    cm_per_px = tag_size_cm / avg_edge_px

    # 3. 중심 오차를 cm 단위로 환산
    offset_cm = offset_px * cm_per_px  # [dx, dy] in cm
    distance_cm = np.linalg.norm(offset_cm)
    return offset_cm, distance_cm

def compute_relative_yaw(tag):
    pt0 = tag.corners[0]  # top-left
    pt1 = tag.corners[1]  # top-right

    dx = pt1[0] - pt0[0]
    dy = pt1[1] - pt0[1]

    # 영상 좌표계 기준 (origin = top-left, y+ is downward)
    angle_rad = np.arctan2(-dy, dx)  # y축 반전 보정
    angle_deg = np.rad2deg(angle_rad)

    return angle_deg

def compute_center_yaw(tag, frame_shape):
    frame_center = np.array([frame_shape[1] / 2, frame_shape[0] / 2])
    tag_center = np.array(tag.center)

    dx, dy = frame_center - tag_center
    angle_rad = np.arctan2(-dy, dx) 
    angle_deg = np.rad2deg(angle_rad)
    return angle_deg

def compute_relative_yaw_difference(yaw_tag, center_yaw):
    # 각도 차이를 -180~180 범위로 정규화
    diff = (center_yaw - yaw_tag + 180) % 360 - 180
    return diff


def draw_tag_direction_arrow(frame, tag, length=40, color=(0, 0, 255), thickness=2):
    corners = tag.corners
    pt0 = corners[0]  # top-left
    pt1 = corners[1]  # top-right
    dir_vec = pt1 - pt0
    dir_vec = dir_vec / np.linalg.norm(dir_vec) * length
    center = tag.center
    endpoint = center + dir_vec
    cv2.arrowedLine(frame, tuple(np.int32(center)), tuple(np.int32(endpoint)), color, thickness)



def main():
    cap, fps = camera_open()
    detector = AprilTagDetector()

    while True:
        frame, gray = frame_process(cap, camera_matrix, dist_coeffs)
        if frame is None:
            break

        tags = detector.tag_detect(gray)
        for tag in tags:
            x_cm, y_cm, z_cm, yaw, pitch, roll = compute_tag_offset_xyz(
            tag, frame.shape, camera_matrix, dist_coeffs, object_points
        )
            
            
            offset_cm_vec, offset_cm_mag = estimate_tag_offset_cm(tag, frame.shape, tag_size_cm=3.2)
            visual_yaw = compute_relative_yaw(tag, camera_matrix, dist_coeffs, object_points)
            center_yaw = compute_center_yaw(tag, frame.shape)
            relative_yaw = compute_relative_yaw_difference(visual_yaw, center_yaw)

            draw_tag_direction_arrow(frame, tag)
            pts = np.int32(tag.corners).reshape((-1, 1, 2))
            cx, cy = map(int, tag.center)

            cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
            cv2.line(frame, (int(frame.shape[1] // 2), int(frame.shape[0] // 2)), (cx, cy), (255, 0, 0), 2)
            cv2.putText(frame, f"[x={offset_cm_vec[0]:+.1f} cm, y={offset_cm_vec[1]:+.1f} cm] from center", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Distance: {offset_cm_mag:.1f} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Yaw: {visual_yaw:.1f}°", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Center Yaw: {center_yaw:.1f}°", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 50), 2)
            cv2.putText(frame, f"Relative Yaw: {relative_yaw:+.1f}°", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 150, 255), 2)

            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # 단일 태그만 표시 (다중 시 제외 가능)
            break

        cv2.imshow("Offset & Angle Viewer", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('1'):
            publish_motion(0, 0, round(relative_yaw, 1))  # 회전
        elif key == ord('2'):
            publish_motion(round(offset_cm_vec[0], 1), round(offset_cm_vec[1], 1), 0)  # 직진
        elif key == ord('3'):
            publish_motion(round(offset_cm_vec[0], 1), round(offset_cm_vec[1], 1), round(relative_yaw, 1))  # 회전, 직진
        elif key == ord('4'):
            dtheta = (90 - visual_yaw + 180) % 360 - 180
            publish_motion(0, 0, round(dtheta, 1))
        elif key == ord('q'):
            print("종료합니다...")
            break
        
        
    if mqtt_connected:
        client.loop_stop()
        client.disconnect()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
