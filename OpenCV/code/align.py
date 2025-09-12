import json
import numpy as np
from .config import  IP_address_, MQTT_TOPIC_COMMANDS_ , MQTT_PORT , NORTH_TAG_ID

def _normalize_delta_deg(delta):
    """정규화: –180° ~ +180°"""
    return ((delta + 180) % 360) - 180

def send_center_align(client, tag_info, MQTT_TOPIC_COMMANDS_, targets=None,alignment_pending=None):
    """
    중앙 정렬 명령 전송 (회전 + 직진)
    → alignment_pending에 있는 로봇만 대상
    """
    if targets is None:
        targets = list(tag_info.keys())

    for tag_id in targets:
        rid_str = str(tag_id)

        # ✅ pending에 등록된 로봇만 처리
        if rid_str not in alignment_pending:
            print(f"⏩ Robot_{rid_str} 는 중앙정렬 대상 아님 → 건너뜀")
            continue

        data = tag_info.get(tag_id)
        if data is None or data.get('status') != 'On':
            print(f"   ✗ Robot_{rid_str} 상태 비정상 → 건너뜀")
            continue

        # 거리(cm), 상대각도(°)
        d = data.get('dist_cm', 0.0)
        ry = data.get('relative_angle_deg', 0.0)

        # 명령 생성
        rot_cmd = f"{'L' if ry < 0 else 'R'}{abs(ry):.1f}_modeOnly"
        mov_cmd = f"F{d:.1f}_modeC"

        payload = {
            "commands": [{
                "robot_id": rid_str,
                "command_count": 2,
                "command_set": [
                    {"command": rot_cmd},
                    {"command": mov_cmd}
                ]
            }]
        }

        print(f"▶ 중앙정렬 명령 전송: Robot_{rid_str} → {rot_cmd} + {mov_cmd}")
        client.publish(MQTT_TOPIC_COMMANDS_, json.dumps(payload, ensure_ascii=False))

#북쪽정렬
def send_north_align(client, tag_info, MQTT_TOPIC_COMMANDS_, *, targets=None, alignment_pending=None):
    """
    북쪽 태그 없이 '보드 좌표계 North(=90°)' 로만 정렬 (회전만, modeOnly)
    VisionSystem이 보드 lock된 상태에서 각 태그의 yaw_front_deg가 보드 좌표계 기준임을 전제.
    """
    if targets is None:
        targets = list(tag_info.keys())

    for tag_id in targets:
        rid_str = str(tag_id)

        # pending만 처리 (옵션)
        if alignment_pending is not None and rid_str not in alignment_pending:
            print(f"⏩ Robot_{rid_str} 는 북쪽정렬 대상 아님 → 건너뜀")
            continue

        data = tag_info.get(tag_id)
        if data is None or data.get('status') != 'On':
            print(f"   ✗ Robot_{rid_str} 상태 비정상 → 건너뜀")
            continue

        cur_yaw = data.get('yaw_front_deg', None)
        if cur_yaw is None:
            print(f"   ✗ Robot_{rid_str} yaw_front_deg 없음")
            continue

        # 보드 North = 90°
        base_angle = 90.0
        delta = _normalize_delta_deg(cur_yaw - base_angle)

        rot_deg = round(abs(delta), 1)
        cmd_letter = 'L' if delta > 0 else 'R'
        cmd = f"{cmd_letter}{rot_deg}_modeOnly"

        payload = {
            "commands": [{
                "robot_id": rid_str,
                "command_count": 1,
                "command_set": [{"command": cmd}]
            }]
        }
        print(f"▶ 보드-북쪽정렬: Robot_{rid_str} → target=90°, Δ={delta:.1f}° → {cmd}")
        client.publish(MQTT_TOPIC_COMMANDS_, json.dumps(payload, ensure_ascii=False))


# 방향 정렬(기존 북쪽 정렬 대체)
def send_direction_align(client, tag_info, MQTT_TOPIC_COMMANDS_, targets=None, alignment_pending=None):
    """
    가장 가까운 동/서/남/북(base_angle)에 맞춰 회전만 수행 (modeOnly)
    VisionSystem에서 쓰는 동일한 로직으로 base_angle과 delta를 계산한다.
    """
    if targets is None:
        targets = list(tag_info.keys())

    for tag_id in targets:
        rid_str = str(tag_id)

        # pending 대상만 처리 (옵션)
        if alignment_pending is not None and rid_str not in alignment_pending:
            print(f"⏩ Robot_{rid_str} 는 방향정렬 대상 아님 → 건너뜀")
            continue

        data = tag_info.get(tag_id)
        if data is None or data.get('status') != 'On':
            print(f"   ✗ Robot_{rid_str} 상태 비정상 → 건너뜀")
            continue

        yaw_front = data.get("yaw_front_deg", None)
        if yaw_front is None:
            print(f"   ✗ Robot_{rid_str} yaw_front_deg 없음")
            continue

        # 0~360 정규화
        yaw_deg = (yaw_front + 360) % 360

        # VisionSystem과 동일한 기준 (주의: 여기서는 E=0, N=90, S=270, W=180)
        direction_angles = [90, 0, 270, 180]   # N, E, S, W (이름은 필요 없음)
        diffs = [abs(((yaw_deg - a + 180) % 360) - 180) for a in direction_angles]
        base_angle = direction_angles[diffs.index(min(diffs))]

        # 기준 각도 대비 오차 (–180~+180 → 실제로는 ±45 이내)
        delta = ((yaw_deg - base_angle + 180) % 360) - 180

        rot_deg = round(abs(delta), 1)
        cmd_letter = 'L' if delta > 0 else 'R'   # 북정렬과 동일한 부호 처리
        cmd = f"{cmd_letter}{rot_deg}_modeOnly"

        payload = {
            "commands": [{
                "robot_id": rid_str,
                "command_count": 1,
                "command_set": [{"command": cmd}]
            }]
        }
        print(f"▶ 방향정렬 명령 전송: Robot_{rid_str} → target={base_angle}°, Δ={delta:.1f}° → {cmd}")
        client.publish(MQTT_TOPIC_COMMANDS_, json.dumps(payload, ensure_ascii=False))


