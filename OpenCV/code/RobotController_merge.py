
# import time
# import json
# import paho.mqtt.client as mqtt
# from align import send_center_align, send_north_align 
# from config import MQTT_TOPIC_COMMANDS_, MQTT_PORT, IP_address_ ,NORTH_TAG_ID
# import threading

# RobotController.py

import json
import time
import threading
from OpenCV.code.align import send_center_align, send_north_align, send_direction_align  # :contentReference[oaicite:1]{index=1}
from typing import Optional, Callable
import math  # 추가
import numpy as np  # 추가

class RobotController:
    def __init__(
        self,
        client,
        mqtt_topic_commands: str,
        *,
        grid_col: int, # <-- [1] 추가
        done_topic: str = "robot/done",
        north_tag_id: int | None = None,
        direction_corr_threshold_deg: float = 3.0,
        alignment_delay_sec: float = 0.5,
        step_transition_delay_sec: float = 0.5,
        alignment_angle: float = 1.0,
        alignment_dist: float = 1.0,
    ):
        # 통신
        self.client = client
        self.mqtt_topic_commands = mqtt_topic_commands
        self.done_topic = done_topic
        self.north_tag_id = north_tag_id  # 필요 없으면 None
        self.sequence_completion_callback = None
        self.robot_completion_callback = None

        # 튜너블 파라미터
        self.direction_corr_threshold_deg = direction_corr_threshold_deg
        self.alignment_angle = alignment_angle
        self.alignment_dist = alignment_dist
        self.alignment_delay_sec = alignment_delay_sec
        self.step_transition_delay_sec = step_transition_delay_sec

        # 외부 데이터 공급자
        self.tag_info_provider = None  # lambda: dict
        self.board_info_provider = None # <-- [2] 추가
        

        # 런타임 상태
        self.active = False
        self.current_step = 0
        self.max_steps = 0
        self.grid_col = grid_col # <-- [3] 추가
        
        self.step_inflight: set[str] = set()
        self.step_done: set[str] = set()
        self.step_yield: set[str] = set()          # 이번 스텝에서 양보해야 하는 로봇 집합
        self._pending_moves: dict[str, dict] = {}  # {rid: {"command": "Fxx..."}}
        self._step_cell_plan = {}                  # {step_idx: {rid: {"src":(r,c), "dst":(r,c)}}}
        self.yield_block_cell = {}                 # {rid: (r,c)}  # rid가 기다려야 하는 '블로킹' 셀
        
        self.robot_command_map: dict[str, list[str]] = {}
        self.robot_indices: dict[str, int] = {}
        self.paused_robots: set[str] = set()
        self.alignment_pending: dict[str, dict] = {}  # {rid: {"mode":..., "in_progress":bool}}
        self.inflight: dict[str, bool] = {}  # (옵션) 로봇별 실행중 표시
        self._last_align_ok_ts = {}
        self._align_cb = None
        self._robot_done_cb = None
        self._last_align_ok_ts = {}
        self.defer_pause_all = False

        
    def set_sequence_completion_callback(self, callback: Callable[[], None]):
        """전체 시퀀스 완료 시 호출될 콜백 함수를 등록합니다."""
        self.sequence_completion_callback = callback
        
    def set_robot_completion_callback(self, callback: Callable[[str], None]):
        """개별 로봇이 자신의 경로를 모두 마쳤을 때 호출될 콜백 함수를 등록합니다."""
        self.robot_completion_callback = callback
        
    def stop_sequence(self):
        """진행 중인 시퀀스를 즉시 중단합니다."""
        if self.active:
            print("⏹️ [Controller] 외부 요청으로 시퀀스를 중단합니다.")
            self.active = False
            self.step_inflight.clear()
            self.step_done.clear()
    # ===== 외부 연결 =====
    def set_tag_info_provider(self, fn):
        """fn() -> 최신 tag_info(dict)"""
        self.tag_info_provider = fn
        
    def set_board_info_provider(self, fn):
        """fn() -> 최신 board_result (VisionSystem의 결과 객체)"""
        self.board_info_provider = fn

    def _normalize_delta_deg(self, delta):
        """각도 정규화: –180° ~ +180°"""
        return ((delta + 180) % 360) - 180

    # ===== 퍼블릭 API =====
    def start_sequence(self, cmd_map: dict[str, list[str]], step_cell_plan: dict[int, dict[str, dict]] | None = None) -> None:
        """경로(cmd_map)를 받아 배리어-스텝 실행 시작"""
        self.robot_command_map = cmd_map or {}
        if not self.robot_command_map:
            print("⚠️ 전송할 명령이 없습니다.")
            self.active = False
            return

        # 상태 초기화
        self.robot_indices = {rid: 0 for rid in self.robot_command_map}
        self.inflight = {rid: False for rid in self.robot_command_map}
        try:
            self._release(list(self.robot_command_map.keys()))  # 모든 대상에게 RE 전송
        except Exception:
            pass
        self.paused_robots.clear()
        self.alignment_pending.clear()

        self.step_yield.clear()
        self._pending_moves.clear()
        self._step_cell_plan = step_cell_plan or {}
        self.yield_block_cell.clear()

        self.max_steps = max((len(v) for v in self.robot_command_map.values()), default=0)
        self.current_step = 0
        self.step_inflight.clear()
        self.step_done.clear()
        self.active = True

        print(f"▶ 배리어 모드 시작: 총 스텝 {self.max_steps}, 대상 {sorted(list(self.robot_command_map.keys()))}")
        self._send_step_commands()

    # ===== 내부 구현 =====

    def _send_step_commands(self) -> None:
            """이번 스텝 명령을 중앙 명령 토픽으로 publish"""
            if not self.active:
                return

            # 스텝 집합 초기화
            self.step_inflight = set()
            self.step_done = set()
            self.step_yield = set()
            self._pending_moves = {}
            self.yield_block_cell.clear()

            participants = [
                rid for rid, cmds in self.robot_command_map.items()
                if self.current_step < len(cmds)
            ]
            if not participants:
                print("\n✅ [모든 명령 전송 완료] (no participants)")
                self.active = False
                return

            actual_targets = [rid for rid in participants if rid not in self.paused_robots]
            if not actual_targets:
                print(f"⏸ 모든 대상이 일시정지 → Step {self.current_step+1}/{self.max_steps} 대기")
                return

            tag_info = self.tag_info_provider() if self.tag_info_provider else {}
            board_result = self.board_info_provider() if self.board_info_provider else None

            plan = self._step_cell_plan.get(self.current_step, {}) if hasattr(self, "_step_cell_plan") else {}
            src_by_robot = {k: v.get("src") for k, v in plan.items()}
            dst_by_robot = {k: v.get("dst") for k, v in plan.items()}
            src_set = set(src_by_robot.values()) if src_by_robot else set()

            for rid in actual_targets:
                cmd_raw = self.robot_command_map[rid][self.current_step]
                cmd = cmd_raw

                if cmd == "Stay":
                    print(f"⏸ [Step {self.current_step+1}/{self.max_steps}] [Robot_{rid}] → Stay (즉시 완료)")
                    self.inflight[rid] = False
                    self.robot_indices[rid] = self.current_step + 1
                    self.step_inflight.add(rid)
                    self.step_done.add(rid)
                    continue
                
                command_set = [{"command": cmd}]
                two_stage_reason = "NONE"

                if (isinstance(cmd, str) and cmd.startswith("F") and
                    board_result and hasattr(board_result, 'grid_reference') and board_result.grid_reference):

                    robot_data = tag_info.get(int(rid))
                    dst_cell = dst_by_robot.get(rid)

                    if robot_data and dst_cell:
                        current_cm = robot_data.get("center_cm")
                        current_yaw = robot_data.get("yaw_front_deg")
                        cell_centers = board_result.grid_reference.get("cell_centers")

                        if current_cm and current_yaw is not None and cell_centers:
                            try:
                                r, c = dst_cell
                                idx = r * self.grid_col + c
                                target_cm = cell_centers[idx]

                                vec = np.array(target_cm) - np.array(current_cm)
                                dist_cm = np.linalg.norm(vec)

                                if dist_cm > 0.5:
                                    # 각도 계산 (기존 방식 유지: 시각계 보정 포함)
                                    vec_for_angle = vec.copy()
                                    vec_for_angle[1] = -vec_for_angle[1]
                                    target_yaw = math.degrees(math.atan2(-vec_for_angle[1], vec_for_angle[0])) + 180

                                    delta = self._normalize_delta_deg(target_yaw - current_yaw)
                                    
                                    if cmd.startswith("T"):
                                        delta = self._normalize_delta_deg(delta + 5.0)


                                    # ✅ 항상 두 단계(회전 + 직진)로 보냄
                                    rot_deg = round(abs(delta), 1)
                                    if rot_deg < 0.1:
                                        rot_deg = 0.1   # 0° 회전 방지용 최소각
                                    rot_cmd_letter = 'R' if delta > 0 else 'L'
                                    pre_cmd = f"{rot_cmd_letter}{rot_deg}_modeOnly"
                                    mov_cmd = f"F{dist_cm:.1f}_modeA"

                                    command_set = [{"command": pre_cmd}, {"command": mov_cmd}]
                                    two_stage_reason = "ALIGN_MOVE"
                                else:
                                    command_set = [{"command": "Stay"}]

                            except IndexError:
                                print(f"⚠️ [Robot_{rid}] 목표 셀 인덱스 오류: {dst_cell}")
                            except Exception as e:
                                print(f"⚠️ [Robot_{rid}] 정렬 이동 계산 오류: {e}")
                                
                elif (isinstance(cmd, str) and (cmd.startswith("R") or cmd.startswith("L") or cmd.startswith("T")) and
                    board_result and hasattr(board_result, 'grid_reference') and board_result.grid_reference):

                    robot_data = tag_info.get(int(rid))
                    dst_cell = dst_by_robot.get(rid)

                    if robot_data and dst_cell:
                        current_cm = robot_data.get("center_cm")
                        current_yaw = robot_data.get("yaw_front_deg")   # 비전에서 계산된 현재 정면각(°)
                        cell_centers = board_result.grid_reference.get("cell_centers")

                        if current_cm and current_yaw is not None and cell_centers:
                            try:
                                r, c = dst_cell
                                idx = r * self.grid_col + c
                                target_cm = cell_centers[idx]

                                # 다음 셀 중심까지의 벡터
                                vec = np.array(target_cm) - np.array(current_cm)
                                dist_cm = np.linalg.norm(vec)

                                if dist_cm > 0.5:
                                    # (비전과 동일한 방식) 각도 계산: 보라색 표기와 일치
                                    vec_for_angle = vec.copy()
                                    vec_for_angle[1] = -vec_for_angle[1]  # Y축 부호 반전
                                    target_yaw = math.degrees(math.atan2(-vec_for_angle[1], vec_for_angle[0])) + 180.0

                                    # 현재 각도 → 목표 각도까지 회전량(부호 유지, -180~+180 정규화)
                                    delta = self._normalize_delta_deg(target_yaw - current_yaw)

                                    # 회전 명령(항상 modeOnly). 명령에 적힌 숫자(R90 등)는 '무시'
                                    rot_deg = round(abs(delta), 1)
                                    if rot_deg < 0.1:
                                        rot_deg = 0.1  # 0° 회전 방지
                                    rot_cmd_letter = 'R' if delta > 0 else 'L'
                                    rot_cmd = f"{rot_cmd_letter}{rot_deg}_modeOnly"

                                    # 원래 의도 기반 속도 모드 고정: R/L → modeB, T → modeC
                                    speed_mode = "modeB" if cmd[0] in ("R", "L") else "modeC"
                                    mov_cmd = f"F{dist_cm:.1f}_{speed_mode}"

                                    command_set = [{"command": rot_cmd}, {"command": mov_cmd}]
                                    two_stage_reason = "ROT→CENTER_MOVE"
                                else:
                                    command_set = [{"command": "Stay"}]

                            except IndexError:
                                print(f"⚠️ [Robot_{rid}] 목표 셀 인덱스 오류: {dst_cell}")
                            except Exception as e:
                                print(f"⚠️ [Robot_{rid}] 회전(보정) 후 중앙이동 계산 오류: {e}")

                # if two_stage_reason != "ALIGN_MOVE":
                #     delta = tag_info.get(int(rid), {}).get("heading_offset_deg")
                #     if (isinstance(cmd, str) and cmd.startswith("F") and
                #         delta is not None and abs(delta) >= self.direction_corr_threshold_deg):
                        
                #         angle = round(abs(delta), 1)
                #         pre_cmd = f"{'L' if delta > 0 else 'R'}{angle}_modeOnly"
                #         command_set = [{"command": pre_cmd}, {"command": cmd}]
                #         two_stage_reason = "DIR_FIX"
                
                is_yield = False
                if dst_by_robot:
                    my_dst = dst_by_robot.get(rid)
                    if my_dst and my_dst in src_set:
                        is_yield = True
                        self.step_yield.add(rid)
                        self.yield_block_cell[rid] = my_dst

                if is_yield:
                    self._pending_moves[rid] = {"command_set": command_set, "two_stage_reason": two_stage_reason}
                    self.step_yield.add(rid)
                    self.yield_block_cell[rid] = my_dst
                    # ✅ 양보 중이라도 이번 스텝의 배리어 대상에 '포함'시켜 스킵을 막는다
                    self.step_inflight.add(rid)
                    print(f"⏸️ [Step {self.current_step+1}] [Robot{rid}] → YIELD 보류 (pkg={len(command_set)})")
                else:
                    payload = json.dumps({
                        "commands": [{
                            "robot_id": rid,
                            "command_count": len(command_set),
                            "command_set": command_set,
                        }]
                    })
                    cmd_str = ' + '.join(c['command'] for c in command_set)
                    print(f"📤 [Step {self.current_step+1}/{self.max_steps}] [Robot_{rid}] → {cmd_str} (reason: {two_stage_reason})")
                    
                    self.client.publish(self.mqtt_topic_commands, payload)
                    self.inflight[rid] = True
                    self.robot_indices[rid] = self.current_step + 1
                    self.step_inflight.add(rid)

            if self.step_inflight:
                print(f"▶ Step {self.current_step+1}/{self.max_steps} 전송 대상: {sorted(list(self.step_inflight))}")

            if self.step_yield:
                self._start_yield_watchdog()
            
            if self.step_inflight and self.step_done >= self.step_inflight:
                self._advance_step_if_ready()

    def _start_yield_watchdog(self) -> None:
        if getattr(self, "_yield_watchdog_on", False):
            return
        self._yield_watchdog_on = True
        step_id = self.current_step
        def _loop():
            try:
                while self.active and self.current_step == step_id and self.step_yield and not (self.step_done >= self.step_inflight):
                    self._try_release_yielders()
                    time.sleep(0.1)
            finally:
                self._yield_watchdog_on = False
        threading.Thread(target=_loop, daemon=True).start()

    # --- yield 해제 조건: '블로킹 셀'이 실제로 비었는지 확인 ---
    def _yield_release_ok(self, rid: str) -> bool:
        if rid not in self.step_yield: 
            return False
        cell = self.yield_block_cell.get(rid)
        if not cell:
            return True
        tag_info = self.tag_info_provider() if self.tag_info_provider else {}
        # grid_position이 있는 태그만 대상으로, '블로킹 셀'을 점유한 로봇이 없어야 함
        for tid, data in tag_info.items():
            gp = data.get("grid_position")
            if gp == cell and data.get("status") == "On":
                return False
        return True

    def _try_release_yielders(self) -> None:
        if not self.active or not self.step_inflight:
            return
        released = []
        for rid in sorted(self.step_yield):
            if self._yield_release_ok(rid):
                pkg = self._pending_moves.get(rid, {})
                cmdset = pkg.get("command_set")
                if cmdset:
                    payload = json.dumps({
                        "commands": [{
                            "robot_id": rid,
                            "command_count": len(cmdset),
                            "command_set": cmdset,
                        }]
                    })
                    self.client.publish(self.mqtt_topic_commands, payload)
                elif "command" in pkg:
                    self._publish(rid, [{"command": pkg["command"]}])

                self.inflight[rid] = True
                self.robot_indices[rid] = self.current_step + 1
                self.step_inflight.add(rid)   # (중복 안전)

                released.append(rid)
        if released:
            print(f"🚦 GO (YIELD 해제): {released}")
            for r in released:
                self.step_yield.discard(r)
                # 발사 후 나머지는 on_mqtt_message에서 DONE 집계
    
    def _advance_step_if_ready(self) -> None:
        """이번 스텝 대상 전원이 완료되면 다음 스텝으로"""
        if not self.active:
            return
        if self.step_inflight and self.step_done >= self.step_inflight:
            print(f"\n✅ Step {self.current_step+1}/{self.max_steps} 전체 완료 → 다음 스텝")
            self.current_step += 1
            
            if self.defer_pause_all:
                targets = list(self.robot_command_map.keys())
                self.pause(targets)                # S 송신 (다음 전송부터 보류)
                self.defer_pause_all = False
                print(f"⏸ 모든 대상 일시정지(스텝 경계) → 시퀀스 종료")
                self.active = False
                self.step_inflight.clear()
                self.step_done.clear()
                if self.sequence_completion_callback:
                    self.sequence_completion_callback()
                return
                        
            if self.current_step >= self.max_steps:
                print("\n✅ [모든 명령 전송 완료] (max steps reached)")
                self.active = False

                # ▼▼▼▼▼ [핵심] 이 두 줄이 "귀를 막는" 역할을 합니다 ▼▼▼▼▼
                self.step_inflight.clear()
                self.step_done.clear()
                # ▲▲▲▲▲ [핵심] ▲▲▲▲▲
                
                if self.sequence_completion_callback:
                    self.sequence_completion_callback()
                return
            #self._send_step_commands()
            def _defer_and_send(step_id=self.current_step):
                time.sleep(self.step_transition_delay_sec)
                # 아직 동일 스텝이고, 시퀀스가 살아있을 때만 전송
                if self.active and self.current_step == step_id:
                    self._send_step_commands()
            threading.Thread(target=_defer_and_send, daemon=True).start()

    def on_mqtt_message(self, topic: str, payload_raw: str) -> None:
        """MQTT 레이어에서 DONE 수신 시 호출해줄 콜백"""
        if topic != self.done_topic:
            return

        payload = payload_raw if isinstance(payload_raw, str) else payload_raw.decode("utf-8", "ignore")

        # DONE;Robot_1;cmd=MOVE;mode=straight;...
        if not payload.startswith("DONE;Robot_"):
            return

        parts = payload.split(";")
        try:
            robot_id = parts[1].split("_", 1)[1]  # "1"
            cmd_info = parts[2] + (f";{parts[3]}" if len(parts) > 3 else "")
        except Exception as e:
            print(f"[DONE 파싱 오류] {payload} / {e}")
            return

        print(f"✅ [Robot_{robot_id}] 명령 ({cmd_info}) 완료")
        if self.inflight is not None:
            self.inflight[robot_id] = False

        # ---------- (A) 정렬 반복: modeOnly 완료 후 지연 재시도 ----------
        if "mode=modeOnly" in payload and robot_id in self.alignment_pending:
            info = self.alignment_pending[robot_id]
            mode = info.get("mode")
            in_progress = info.get("in_progress", False)

            def _delay_then(fn):
                def _wrap():
                    time.sleep(self.alignment_delay_sec)
                    fn()
                threading.Thread(target=_wrap, daemon=True).start()

            if mode == "north":
                if self.check_north_alignment_ok(robot_id):
                    print(f"✅ 북쪽 정렬 완료: Robot_{robot_id}")
                    self.clear_alignment_pending(robot_id)
                elif not in_progress:
                    self.alignment_pending[robot_id]["in_progress"] = True
                    def after_delay():
                        if self.check_north_alignment_ok(robot_id):
                            print(f"✅ 북쪽 정렬 완료 (지연 후 재확인): Robot_{robot_id}")
                            self.clear_alignment_pending(robot_id)
                            return
                        self.run_north_align([robot_id], do_release=False)
                        if robot_id in self.alignment_pending:
                            self.alignment_pending[robot_id]["in_progress"] = False
                    _delay_then(after_delay)

            elif mode == "direction":
                if self.check_direction_alignment_ok(robot_id):
                    print(f"✅ 방향정렬 완료: Robot_{robot_id}")
                    self._last_align_ok_ts[robot_id] = time.time()
                    self.clear_alignment_pending(robot_id)
                elif not in_progress:
                    self.alignment_pending[robot_id]["in_progress"] = True
                    def after_delay():
                        if self.check_direction_alignment_ok(robot_id):
                            print(f"✅ 방향정렬 완료 (지연 후 재확인): Robot_{robot_id}")
                            self.clear_alignment_pending(robot_id)
                            return
                        # 🔁 기존: self._send_direction_align([int(robot_id)])
                        self.run_direction_align([robot_id], do_release=False)
                        if robot_id in self.alignment_pending:
                            self.alignment_pending[robot_id]["in_progress"] = False
                    _delay_then(after_delay)

            # (중앙 정렬 재시도)
            elif mode == "center":
                if self.check_center_alignment_ok(robot_id):
                    print(f"✅ 중앙정렬 완료: Robot_{robot_id}")
                    self.clear_alignment_pending(robot_id)
                elif not in_progress:
                    self.alignment_pending[robot_id]["in_progress"] = True
                    def after_delay():
                        if self.check_center_alignment_ok(robot_id):
                            print(f"✅ 중앙정렬 완료 (지연 후 재확인): Robot_{robot_id}")
                            self.clear_alignment_pending(robot_id)
                            return
                        # 🔁 기존: self._send_center_align([int(robot_id)])
                        self.run_center_align([robot_id], do_release=False)
                        if robot_id in self.alignment_pending:
                            self.alignment_pending[robot_id]["in_progress"] = False
                    _delay_then(after_delay)


        if robot_id in self.paused_robots:
            total = len(self.robot_command_map.get(robot_id, []))
            sent = self.robot_indices.get(robot_id, 0)
            print(f"⏸ [Robot_{robot_id}] 개별 일시정지 상태 → 다음 전송 보류 (완료={sent}/{total})")

        if self.active and (robot_id in self.step_inflight):
            if "mode=modeOnly" in payload:
                # dir-fix+MOVE의 첫 단계(방향 보정) 완료는 스텝 진행에 영향을 주지 않으므로
                # 여기서 아무것도 하지 않고 함수를 종료합니다.
                return
            # ▶ modeOnly(정렬) 외의 모든 DONE은 '이번 스텝 실제 명령' 완료로 집계
            self.step_done.add(robot_id)
            self.inflight[robot_id] = False
            print(f"🟢 [Step {self.current_step+1}/{self.max_steps}] 완료: {sorted(self.step_done)} / {sorted(self.step_inflight)}")
            # yield 로봇 출발 재확인 후, 스텝 전진
            # 현재 완료된 스텝 번호(self.current_step + 1)가 해당 로봇의 전체 경로 길이와 같다면,
            # 그 로봇은 방금 자신의 최종 목적지에 도착한 것입니다.
            if (self.current_step + 1) == len(self.robot_command_map.get(robot_id, [])):
                if self.robot_completion_callback:
                    # 등록된 콜백 함수(random_manager의 함수)를 호출하여 보고합니다.
                    self.robot_completion_callback(robot_id)
            self._try_release_yielders()
            if not self.step_yield:
                self._advance_step_if_ready()

    # ===== 정렬 pending 관리 =====
    def set_alignment_pending(self, robot_id: str, mode: str):
        self.alignment_pending[robot_id] = {"mode": mode, "in_progress": False}
        print(f"▶ pending: {mode} <- Robot_{robot_id}")

    def clear_alignment_pending(self, robot_id: str):
        if robot_id in self.alignment_pending:
            del self.alignment_pending[robot_id]

    # ===== 정렬 OK 판정 =====
    def check_center_alignment_ok(self, robot_id: str) -> bool:
        tag_info = self.tag_info_provider() if self.tag_info_provider else {}
        data = tag_info.get(int(robot_id))
        if not data or data.get("status") != "On":
            print(f"⚠️ Robot_{robot_id} 정렬용 태그 정보 없음 또는 비활성"); return False
        dist = data.get("dist_cm", 0)
        print(f"[중앙정렬 거리 확인] Robot_{robot_id}: dist={dist:.2f} cm (기준: {self.alignment_dist} cm)")
        return abs(dist) <= self.alignment_dist

    def check_north_alignment_ok(self, robot_id: str) -> bool:
        if not self.tag_info_provider:
            return False
        tag_info = self.tag_info_provider()
        tag = tag_info.get(int(robot_id))
        if not tag or tag.get("status") != "On":
            print(f"⚠️ Robot_{robot_id} 정렬용 태그 정보 없음 또는 비활성"); return False

        cur_yaw = tag.get("yaw_front_deg", None)
        if cur_yaw is None:
            return False

        # 보드 North = 90°
        delta = ((cur_yaw - 90.0 + 180) % 360) - 180
        print(f"▶ Robot_{robot_id} (보드-N) Δ={delta:.2f}°, 기준: {self.alignment_angle:.1f}°")
        return abs(delta) < self.alignment_angle

    def check_direction_alignment_ok(self, robot_id: str) -> bool:
        if not self.tag_info_provider:
            return False
        tag_info = self.tag_info_provider()
        tag = tag_info.get(int(robot_id))
        if not tag or tag.get("status") != "On": return False
        delta = tag.get("heading_offset_deg", None)
        if delta is None: return False
        print(f"▶ Robot_{robot_id} 방향정렬 Δ={delta:.2f}°, 기준: {self.alignment_angle:.1f}°")
        return abs(delta) < self.alignment_angle
    
    # ===== 정렬 명령 송신 헬퍼 =====
    # ---- 러너: 북쪽 정렬 (release → 미정렬 추림 → pending 등록 → 전송) ----
    def run_board_north_align(self, preset_ids: list[int | str], *, do_release: bool = True) -> None:
        if do_release:
            self._release(preset_ids)

        tag_info = self.tag_info_provider() if self.tag_info_provider else {}
        # OK 아닌 대상만 선별
        unaligned = [rid for rid in preset_ids if not self.check_north_alignment_ok(str(rid))]
        if not unaligned:
            print("✅ 북쪽 정렬 대상 없음")
            return

        self._mark_pending(unaligned, "north")
        send_north_align(
            self.client, tag_info, self.mqtt_topic_commands,
            targets=[int(r) for r in unaligned], alignment_pending=self.alignment_pending
        )


    # ---- 러너: 중앙 정렬 ----
    def run_center_align(self, preset_ids: list[int | str], *, do_release: bool = True) -> None:
        if do_release:
            self._release(preset_ids)

        tag_info = self.tag_info_provider() if self.tag_info_provider else {}
        unaligned = [rid for rid in preset_ids if not self.check_center_alignment_ok(str(rid))]
        if not unaligned:
            print("✅ 중앙 정렬 대상 없음")
            return

        self._mark_pending(unaligned, "center")
        send_center_align(
            self.client, tag_info, self.mqtt_topic_commands,
            targets=[int(r) for r in unaligned], alignment_pending=self.alignment_pending
        )  # :contentReference[oaicite:3]{index=3}

    # ---- 러너: 방향 정렬(가까운 NESW) ----
    def run_direction_align(self, preset_ids: list[int | str], *, do_release: bool = True) -> None:
        if do_release:
            self._release(preset_ids)

        tag_info = self.tag_info_provider() if self.tag_info_provider else {}
        unaligned = [rid for rid in preset_ids if not self.check_direction_alignment_ok(str(rid))]
        if not unaligned:
            print("✅ 방향 정렬 대상 없음")
            return

        self._mark_pending(unaligned, "direction")
        send_direction_align(
            self.client, tag_info, self.mqtt_topic_commands,
            targets=[int(r) for r in unaligned], alignment_pending=self.alignment_pending
        )  # :contentReference[oaicite:4]{index=4}


    # ---- 내부 유틸: 개별/일괄 release ----
    def _release(self, targets: list[int | str]) -> None:
        """대상 로봇에 RE 전송 (정렬/재개 전 준비)."""
        for rid in targets:
            rid = str(rid)
            self.client.publish(f"robot/{rid}/cmd", "RE")
            print(f"▶ [Robot_{rid}] RE 전송")

    def _mark_pending(self, targets: list[int | str], mode: str) -> None:
        """alignment_pending 등록 헬퍼"""
        for rid in targets:
            self.set_alignment_pending(str(rid), mode)


    def pause(self, targets: list[str]) -> None:
        """특정 로봇만 '현재 명령 완료 후' 정지."""
        for rid in targets:
            rid = str(rid)
            self.paused_robots.add(rid)
            self.client.publish(f"robot/{rid}/cmd", "S")
            print(f"🛑 [Robot_{rid}] 정지 예약(S)")

    def resume(self, targets: list[str]) -> None:
        """특정 로봇만 재개."""
        for rid in targets:
            rid = str(rid)
            if rid in self.paused_robots:
                self.paused_robots.remove(rid)
            self.client.publish(f"robot/{rid}/cmd", "RE")
            print(f"▶ [Robot_{rid}] 재개(RE)")

    def check_all_completed(self) -> bool:
        """
        시퀀스가 비활성 상태이고, 마지막으로 진행된 스텝의 모든 로봇이 
        'DONE' 메시지를 보냈는지까지 확인하여 더 정확한 완료 여부를 반환합니다.
        """
        # 시퀀스가 실행 중이면 당연히 미완료
        if self.active:
            return False
        
        # 시퀀스가 비활성(active=False)이더라도, 마지막 스텝에 참여한 로봇들이
        # 모두 done 처리가 되었는지 확인해야 진짜 완료입니다.
        # step_inflight는 마지막으로 명령을 받은 로봇들의 집합입니다.
        # step_done은 해당 스텝을 완료했다고 보고한 로봇들의 집합입니다.
        if self.step_inflight and self.step_done >= self.step_inflight:
            # 마지막 스텝의 모든 로봇이 완료했고, 시퀀스도 비활성이므로 진짜 완료
            return True
        elif not self.step_inflight:
            # inflight가 비어있다는 것은 시퀀스가 시작되었지만 아무 로봇도 참여하지 않았거나,
            # 모든 것이 초기화된 상태이므로 완료로 간주합니다.
            return True
        
        # 시퀀스는 비활성이지만 아직 step_done이 inflight를 만족시키지 못한
        # 과도기적 상태일 수 있으므로, 미완료로 처리합니다.
        return False
    
    def _publish(self, rid: str, command_set: list[dict]) -> None:
        payload = json.dumps({
            "commands": [{
                "robot_id": rid,
                "command_count": len(command_set),
                "command_set": command_set,
            }]
        })
        self.client.publish(self.mqtt_topic_commands, payload)
        
    def is_executing(self, rid) -> bool | None:
        """
        현재 step에서 해당 로봇이 '전송되어 완료 대기 중'이라면 True,
        명령 대기 상태라면 False, 알 수 없으면 None.
        """
        try:
            return self.inflight.get(str(rid), None)
        except Exception:
            return None

    def set_alignment_completion_callback(self, cb):
        self._align_cb = cb  # cb(robot_id: str)

    def run_align_sequence(self, preset_ids, *, do_release=False, settle_sec=0.5):
        if do_release:
            self._release(preset_ids)
        # (1) 센터 정렬: 단일 전송
        self.run_center_align(preset_ids, do_release=False)

        def _one(rid):
            rid = str(rid)

            # a) 센터 pending 끝날 때까지 대기 (내부가 delay 재확인 처리)
            while self.alignment_pending.get(rid, {}).get("mode") == "center":
                time.sleep(0.05)

            # b) 카메라 프레임 안정화 버퍼
            time.sleep(settle_sec)

            # c) 방향 정렬: 단일 전송
            self.run_direction_align([rid], do_release=False)

            # d) 방향 pending 끝날 때까지 대기
            while self.alignment_pending.get(rid, {}).get("mode") == "direction":
                time.sleep(0.05)

            # (선택) 완료 콜백
            if hasattr(self, "_align_cb") and self._align_cb:
                self._align_cb(rid)

        for rid in preset_ids:
            threading.Thread(target=_one, args=(rid,), daemon=True).start()


    def aligned_recently(self, robot_id: str | int, within_sec: float = 0.3) -> bool:
        ts = self._last_align_ok_ts.get(str(robot_id))
        return (ts is not None) and ((time.time() - ts) <= within_sec)

    def request_pause_on_step_boundary(self):
        self.defer_pause_all = True
        
    def get_current_step_info(self) -> dict:
        """
        [시각화용] 현재 스텝에서 활성화된 로봇들의 목표(dst)와 명령(cmd)을 반환.
        시퀀스가 비활성이면 빈 dict를 반환.
        """
        if not self.active:
            return {}

        info = {}
        participants = [
            rid for rid, cmds in self.robot_command_map.items()
            if self.current_step < len(cmds)
        ]

        plan = self._step_cell_plan.get(self.current_step, {})

        for rid in participants:
            if rid not in self.paused_robots:
                dst = plan.get(rid, {}).get("dst")
                cmd = self.robot_command_map[rid][self.current_step]
                if dst:
                    info[rid] = {"dst": dst, "cmd": cmd}
        return info
