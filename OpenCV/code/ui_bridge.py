# ========== FrameBus (완전 패치 버전) ==========
import threading
from queue import Queue, Empty
from typing import Tuple, Dict, Any, Optional


class FrameBus:
    _lock = threading.Lock()

    # 기존 영상 관련
    _video = None
    _grid = None
    _warped = None
    _orders = None
    _mode = None

    # ====== 그리드/로봇 데이터 ======
    _grid_state = None                # [[0,1,0,1...]]
    _agent_states = {}                # {id:(row,col)}
    _paths = {}                       # {id:[(r,c), ...]}
    _delays = {}                      # {id: delay}
    _home_positions = {}              # {id:(row,col)}
    _goal_positions = {}              # {id:(row,col)} (⭐ 새로 추가)
    _headings = {}                    # {id:deg} (⭐ 선택적 - 로봇 방향)
    _scenario_status = "idle"
    _selected_robot = None
    _robot_ui_state = {}      # 기존
    _order_history = []       # ⭐ 주문 이력 누적용

    # ====================
    # --- Grid (영상) ---
    # ====================
    @classmethod
    def set_video(cls, frame_bgr):
        with cls._lock:
            cls._video = frame_bgr

    @classmethod
    def get_video(cls):
        with cls._lock:
            return cls._video

    @classmethod
    def set_grid(cls, frame_bgr):
        with cls._lock:
            cls._grid = frame_bgr

    @classmethod
    def get_grid(cls):
        with cls._lock:
            return cls._grid

    # ====================
    # --- Core Grid State ---
    # ====================
    @classmethod
    def set_grid_state(cls, grid_array):
        with cls._lock:
            cls._grid_state = grid_array

    @classmethod
    def get_grid_state(cls):
        with cls._lock:
            return cls._grid_state


    # ====================
    # --- Robot States ---
    # ====================
    @classmethod
    def set_agent_states(cls, agents: dict):
        with cls._lock:
            cls._agent_states = agents.copy()

    @classmethod
    def get_agent_states(cls):
        with cls._lock:
            return cls._agent_states.copy()


    @classmethod
    def set_home_positions(cls, homes: dict):
        with cls._lock:
            cls._home_positions = homes.copy()

    @classmethod
    def get_home_positions(cls):
        with cls._lock:
            return cls._home_positions.copy()

    # ⭐ 새로 추가: Goal Positions
    @classmethod
    def set_goal_positions(cls, goals: dict):
        with cls._lock:
            cls._goal_positions = goals.copy()

    @classmethod
    def get_goal_positions(cls):
        with cls._lock:
            return cls._goal_positions.copy()


    # ⭐ Heading (optional)
    @classmethod
    def set_headings(cls, heads: dict):
        with cls._lock:
            cls._headings = heads.copy()

    @classmethod
    def get_headings(cls):
        with cls._lock:
            return cls._headings.copy()


    # ====================
    # --- Paths ---
    # ====================
    @classmethod
    def set_paths(cls, paths: dict):
        with cls._lock:
            cls._paths = paths.copy()

    @classmethod
    def get_paths(cls):
        with cls._lock:
            return cls._paths.copy()


    # ====================
    # --- Delays ---
    # ====================
    @classmethod
    def set_delays(cls, delays: dict):
        with cls._lock:
            cls._delays = delays.copy()

    @classmethod
    def get_delays(cls):
        with cls._lock:
            return cls._delays.copy()


    # ====================
    # --- Scenario Mode ---
    # ====================
    @classmethod
    def set_scenario_status(cls, status: str):
        with cls._lock:
            cls._scenario_status = status

    @classmethod
    def get_scenario_status(cls):
        with cls._lock:
            return cls._scenario_status

    # ==================== Scenario Mode ====================
    @classmethod
    def set_mode(cls, mode: str):
        """UI에서 모드를 설정할 때 사용 (single, scenario 등)"""
        with cls._lock:
            cls._mode = mode
        print(f"[FrameBus] 모드 설정됨 → {mode}")

    @classmethod
    def get_mode(cls) -> Optional[str]:
        """백엔드 or 다른 UI에서 현재 모드 읽기"""
        with cls._lock:
            return cls._mode


    # ====================
    # --- Warped Preview ---
    # ====================
    @classmethod
    def set_warped(cls, frame_bgr):
        with cls._lock:
            cls._warped = frame_bgr

    @classmethod
    def get_warped(cls):
        with cls._lock:
            return cls._warped

        # ====================
    # --- Orders (BK → UI) ---
    # ====================
    @classmethod
    def set_orders(cls, orders: dict):
        with cls._lock:
            cls._orders = orders.copy()

    @classmethod
    def get_orders(cls):
        with cls._lock:
            return cls._orders.copy() if cls._orders else None


    _selected_robot = None

    @classmethod
    def set_selected_robot(cls, rid):
        with cls._lock:
            cls._selected_robot = rid

    @classmethod
    def get_selected_robot(cls):
        with cls._lock:
            return cls._selected_robot
    
    # ====================
    # --- Robot UI State ---
    # ====================
    
    @classmethod
    def set_robot_ui_state(cls, robot_state_dict: dict):
        with cls._lock:
            cls._robot_ui_state = robot_state_dict.copy()

    @classmethod
    def get_robot_ui_state(cls):
        with cls._lock:
            return cls._robot_ui_state.copy()
        
    @classmethod
    def add_order_history(cls, record: dict):
        """
        record 예시:
        { "rid": 1, "order_id": 12, "goal": [3,4], "status": "ASSIGNED", "timestamp": 12345678 }
        """
        cls._order_history.append(record)

    @classmethod
    def get_order_history(cls):
        return cls._order_history.copy()

    @classmethod
    def clear_order_history(cls):
        cls._order_history.clear()

    _scenario_order_state = {}

    @classmethod
    def set_scenario_order_state(cls, state: dict):
        with cls._lock:
            cls._scenario_order_state = state.copy()

    @classmethod
    def get_scenario_order_state(cls):
        with cls._lock:
            return cls._scenario_order_state.copy()


    # ===================
    # UI → Backend 명령큐
    # ===================
_CMDQ: "Queue[Tuple[str, Dict[str, Any]]]" = Queue()
_DEBUG_LOG = True


def post(cmd: str, **kwargs: Any) -> None:
    if _DEBUG_LOG:
        print(f"[UI→BK] post cmd='{cmd}' kwargs={kwargs}")
    _CMDQ.put((cmd, kwargs))


def get_cmd_nowait() -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    try:
        return _CMDQ.get_nowait()
    except Empty:
        return None, None


def get_cmd(block: bool = True, timeout: Optional[float] = None):
    try:
        return _CMDQ.get(block=block, timeout=timeout)
    except Empty:
        return None, None


def clear_cmd_queue() -> int:
    cleared = 0
    try:
        while True:
            _CMDQ.get_nowait()
            cleared += 1
    except Empty:
        pass
    return cleared
