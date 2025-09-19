# collision_recovery.py
import re
import time
import threading
from typing import List

import paho.mqtt.client as mqtt

from config import IP_address_, MQTT_PORT, NORTH_TAG_ID, critical_dist
from recieve_message import (
    pause_robots, resume_robots, start_sequence,
    set_alignment_pending, alignment_pending
)
from align import send_center_align, send_direction_align


# ─────────────────────────────────────────────────────────────────────────────
# 유틸: 클러스터 검출 (임계거리 이하로 연결된 집합)
# ─────────────────────────────────────────────────────────────────────────────
def _get_tag_cm(tag_info: dict, rid: int):
    d = tag_info.get(rid, {})
    if d.get("status") == "On" and "corrected_center" in d:
        return d["corrected_center"]  # (X_cm, Y_cm)
    return None

def _pairwise_distances_cm(tag_info: dict, ids: List[int]):
    pairs = []
    for i, a in enumerate(ids):
        pa = _get_tag_cm(tag_info, a)
        if not pa:
            continue
        for b in ids[i+1:]:
            pb = _get_tag_cm(tag_info, b)
            if not pb:
                continue
            dx = pa[0] - pb[0]
            dy = pa[1] - pb[1]
            pairs.append(((a, b), (dx*dx + dy*dy) ** 0.5))
    return pairs

def clusters_under_threshold(tag_info: dict, ids: List[int], threshold_cm: float) -> List[List[int]]:
    pairs = _pairwise_distances_cm(tag_info, ids)
    adj = {rid: set() for rid in ids}
    for (a, b), dist in pairs:
        if dist <= threshold_cm:
            adj[a].add(b); adj[b].add(a)

    clusters, visited = [], set()
    for rid in ids:
        if rid in visited:
            continue
        stack, comp = [rid], []
        while stack:
            u = stack.pop()
            if u in visited:
                continue
            visited.add(u); comp.append(u)
            stack.extend(v for v in adj[u] if v not in visited)
        if len(comp) >= 2:
            clusters.append(sorted(comp))
    return clusters


# ─────────────────────────────────────────────────────────────────────────────
# DONE 대기용 미니 MQTT 클라이언트
# ─────────────────────────────────────────────────────────────────────────────
class DoneWaiter:
    """
    특정 로봇 rid의 'T180' 완료를 기다린다.
    완료 메시지 예: "DONE;Robot_3;cmd=MOVE;mode=rotate_180"
    """
    def __init__(self, rid: int, timeout_sec: float = 20.0):
        self.rid = rid
        self.timeout_sec = timeout_sec
        self.evt = threading.Event()
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc):
        client.subscribe("robot/done")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8", "ignore")
        except Exception:
            return
        # 간단한 패턴 체크
        # ex) DONE;Robot_3;cmd=MOVE;mode=rotate_180
        if f";Robot_{self.rid};" in payload and "cmd=MOVE" in payload and "mode=rotate_180" in payload:
            self.evt.set()

    def wait(self) -> bool:
        self.client.connect(IP_address_, MQTT_PORT, 60)
        # 별도 네트워크 루프 스레드
        t = threading.Thread(target=self.client.loop_forever, daemon=True)
        t.start()
        ok = self.evt.wait(self.timeout_sec)
        try:
            self.client.disconnect()
        except Exception:
            pass
        return ok


# ─────────────────────────────────────────────────────────────────────────────
# 핵심: 낮은 번호부터 순차 T180 → 모두 정렬 → 잠금유지
# ─────────────────────────────────────────────────────────────────────────────
def _send_RE(mqtt_client, rid: int):
    mqtt_client.publish(f"robot/{rid}/cmd", "RE")

def _send_S(mqtt_client, rid: int):
    mqtt_client.publish(f"robot/{rid}/cmd", "S")

def _realign_all(mqtt_client, tag_info: dict, rids: List[int], topic_commands: str):
    # 모든 대상: 중앙정렬 + 방향정렬 (제자리 정렬)
    for rid in rids:
        set_alignment_pending(str(rid), "center")
        send_center_align(mqtt_client, tag_info, topic_commands,
                          targets=[rid], alignment_pending=alignment_pending)
        set_alignment_pending(str(rid), "direction")
        send_direction_align(mqtt_client, tag_info, topic_commands,
                             targets=[rid], alignment_pending=alignment_pending)

def recover_lowest_first(mqtt_client, comp_ids: List[int], tag_info: dict, topic_commands: str, keep_locked: bool = True):
    """
    comp_ids: 충돌 클러스터(정렬된 리스트)
    규칙:
      - 가장 큰 번호 1대는 '제자리', 나머지(낮은 번호들)만 차례대로 T180 수행
      - 각 T180은 이전 로봇의 완료 후에 시작
      - 완료 후 전체(클러스터) 중앙정렬 + 방향정렬
      - keep_locked=True면 정렬 후 다시 잠금 유지(전송중단 + S)
    """
    if not comp_ids or len(comp_ids) < 2:
        return

    # 1) 우선 전송 중단으로 안전 확보
    pause_robots([str(r) for r in comp_ids])

    # 2) 이동 대상(movers): 가장 큰 번호 제외
    movers = comp_ids[:-1]  # e.g., [1,3,4] -> [1,3]

    for rid in movers:
        # (a) 이 로봇만 재개
        resume_robots([str(rid)])
        _send_RE(mqtt_client, rid)

        # (b) T180만 전송 (start_sequence로 해당 로봇 1개만)
        start_sequence({ str(rid): ["T180"] })

        # (c) 해당 로봇 완료 대기
        waiter = DoneWaiter(rid, timeout_sec=30.0)
        ok = waiter.wait()
        if not ok:
            print(f"⚠️ [복구] Robot_{rid} T180 완료 신호 타임아웃 → 다음 단계로 진행")

        # (d) 다시 전송 중단으로 잠금
        pause_robots([str(rid)])
        if keep_locked:
            _send_S(mqtt_client, rid)

    # 3) 모두 제자리 정렬
    _realign_all(mqtt_client, tag_info, comp_ids, topic_commands)

    # 4) 잠금 유지 옵션
    if keep_locked:
        pause_robots([str(r) for r in comp_ids])
        for rid in comp_ids:
            _send_S(mqtt_client, rid)
