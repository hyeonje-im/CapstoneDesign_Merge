
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
from align import send_center_align, send_north_align, send_direction_align  # :contentReference[oaicite:1]{index=1}


class RobotController:
    def __init__(
        self,
        client,
        mqtt_topic_commands: str,
        *,
        done_topic: str = "robot/done",
        north_tag_id: int | None = None,
        direction_corr_threshold_deg: float = 3.0,
        alignment_delay_sec: float = 0.5,
        alignment_angle: float = 1.0,
        alignment_dist: float = 1.0,
    ):
        # 통신
        self.client = client
        self.mqtt_topic_commands = mqtt_topic_commands
        self.done_topic = done_topic
        self.north_tag_id = north_tag_id  # 필요 없으면 None

        # 튜너블 파라미터
        self.direction_corr_threshold_deg = direction_corr_threshold_deg
        self.alignment_angle = alignment_angle
        self.alignment_dist = alignment_dist
        self.alignment_delay_sec = alignment_delay_sec

        # 외부 데이터 공급자
        self.tag_info_provider = None  # lambda: dict

        # 런타임 상태
        self.active = False
        self.current_step = 0
        self.max_steps = 0
        
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

    # ===== 외부 연결 =====
    def set_tag_info_provider(self, fn):
        """fn() -> 최신 tag_info(dict)"""
        self.tag_info_provider = fn

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

        # 이번 스텝에 아직 명령이 남은 로봇
        participants = [
            rid for rid, cmds in self.robot_command_map.items()
            if self.current_step < len(cmds)
        ]
        if not participants:
            print("\n✅ [모든 명령 전송 완료] (no participants)")
            self.active = False
            return

        # 일시정지된 로봇 제외
        actual_targets = [rid for rid in participants if rid not in self.paused_robots]
        if not actual_targets:
            print(f"⏸ 모든 대상이 일시정지 → Step {self.current_step+1}/{self.max_steps} 대기")
            return

        tag_info = self.tag_info_provider() if self.tag_info_provider else {}
        # north = tag_info.get(self.north_tag_id, {}) if (tag_info and self.north_tag_id is not None) else {}

        plan = self._step_cell_plan.get(self.current_step, {}) if hasattr(self, "_step_cell_plan") else {}
        src_by_robot = {k: v.get("src") for k, v in plan.items()}
        dst_by_robot = {k: v.get("dst") for k, v in plan.items()}
        src_set = set(src_by_robot.values()) if src_by_robot else set()

        for rid in actual_targets:
            cmd_raw = self.robot_command_map[rid][self.current_step]
            cmd = cmd_raw

            # Stay: 전송 없이 즉시 완료
            if cmd == "Stay":
                print(f"⏸ [Step {self.current_step+1}/{self.max_steps}] [Robot_{rid}] → Stay (즉시 완료)")
                self.inflight[rid] = False
                self.robot_indices[rid] = self.current_step + 1
                self.step_inflight.add(rid)
                self.step_done.add(rid)
                continue

            # 전진 전 방향 오차 보정(2단계) 판단
            delta = None
            try:
                delta = tag_info.get(int(rid), {}).get("heading_offset_deg", None) if tag_info else None
            except Exception:
                delta = None

            command_set = [{"command": cmd}]
            two_stage = False
            
            try:
                if (
                    isinstance(cmd, str)
                    and cmd.startswith("F")
                    and (delta is not None)
                    and abs(float(delta)) >= float(self.direction_corr_threshold_deg)
                ):
                    angle = round(abs(float(delta)), 1)
                    pre_cmd = f"{'L' if float(delta) > 0 else 'R'}{angle}_modeOnly"
                    command_set = [{"command": pre_cmd}, {"command": cmd}]
                    two_stage = True
            except Exception:
                command_set = [{"command": cmd}]
                two_stage = False

            # yield 판단
            is_yield = False
            if dst_by_robot:
                my_dst = dst_by_robot.get(rid)
                if my_dst and my_dst in src_set:
                    is_yield = True
                    self.step_yield.add(rid)
                    self.yield_block_cell[rid] = my_dst

            if is_yield:
                # 👉 yield는 '묶음' 자체를 보류(해제 시 그대로 발사)
                self._pending_moves[rid] = {"command_set": command_set, "two_stage": two_stage}
                print(f"⏸️ [Step {self.current_step+1}/{self.max_steps}] [Robot_{rid}] → YIELD 보류 (pkg={len(command_set)})")
            else:
                payload = json.dumps({
                    "commands": [{
                        "robot_id": rid,
                        "command_count": len(command_set),
                        "command_set": command_set,
                    }]
                })
                print(f"📤 [Step {self.current_step+1}/{self.max_steps}] [Robot_{rid}] → "
                      f"{'dir-fix+MOVE' if two_stage else cmd}")
                self.client.publish(self.mqtt_topic_commands, payload)
                # inflight/집계
                self.inflight[rid] = True
                self.robot_indices[rid] = self.current_step + 1
                self.step_inflight.add(rid)
                # two_stage라도 DONE은 MOVE에서만 집계됨(원본과 동일)
                continue
            
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
                    # 하위호환(혹시 남아있을 경우)
                    self._publish(rid, [{"command": pkg["command"]}])
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
            if self.current_step >= self.max_steps:
                print("\n✅ [모든 명령 전송 완료] (max steps reached)")
                self.active = False
                return
            self._send_step_commands()

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
            # ▶ modeOnly(정렬) 외의 모든 DONE은 '이번 스텝 실제 명령' 완료로 집계
            self.step_done.add(robot_id)
            self.inflight[robot_id] = False
            print(f"🟢 [Step {self.current_step+1}/{self.max_steps}] 완료: {sorted(self.step_done)} / {sorted(self.step_inflight)}")
            # yield 로봇 출발 재확인 후, 스텝 전진
            self._try_release_yielders()
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
        if not self.active:
            return True
        if self.current_step >= self.max_steps and not self.step_inflight:
            return True
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