# OpenCV/code/ui_bridge.py
import threading
from queue import Queue, Empty
from typing import Tuple, Dict, Any, Optional

# ======================================================
# FrameBus : OpenCV 백엔드 ↔ Kivy UI 간 영상/그리드 프레임 공유
# ======================================================
class FrameBus:
    _lock = threading.Lock()
    _video = None   # BGR ndarray
    _grid  = None   # BGR ndarray
    _warped = None  # BGR ndarray
    _orders = None  # UI 상태 딕셔너리
    _mode = None   # UI 모드 문자열

    _grid_state = None
    _agent_states = {}
    _paths = {}
    _delays = {}
    _home_positions = {}
    _scenario_status = "idle"

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

#============================================

    @classmethod
    def set_grid_state(cls, grid_array):
        """numpy grid (0/1 map) 공유"""
        with cls._lock:
            cls._grid_state = grid_array

    @classmethod
    def get_grid_state(cls):
        with cls._lock:
            return cls._grid_state

    @classmethod
    def set_agent_states(cls, agents: dict):
        """로봇의 현재 좌표 {id:(row,col)}"""
        with cls._lock:
            cls._agent_states = agents.copy()

    @classmethod
    def get_agent_states(cls):
        with cls._lock:
            return cls._agent_states.copy()

    @classmethod
    def set_paths(cls, paths: dict):
        """로봇별 경로 {id:[(r,c),...]}"""
        with cls._lock:
            cls._paths = paths.copy()

    @classmethod
    def get_paths(cls):
        with cls._lock:
            return cls._paths.copy()

    @classmethod
    def set_delays(cls, delays: dict):
        """로봇별 대기 시간 {id:delay_step}"""
        with cls._lock:
            cls._delays = delays.copy()

    @classmethod
    def get_delays(cls):
        with cls._lock:
            return cls._delays.copy()

    @classmethod
    def set_home_positions(cls, homes: dict):
        """로봇 초기 위치 {id:(row,col)}"""
        with cls._lock:
            cls._home_positions = homes.copy()

    @classmethod
    def get_home_positions(cls):
        with cls._lock:
            return cls._home_positions.copy()

    @classmethod
    def set_scenario_status(cls, status: str):
        with cls._lock:
            cls._scenario_status = status

    @classmethod
    def get_scenario_status(cls):
        with cls._lock:
            return cls._scenario_status
        
#============================================

    @classmethod
    def set_warped(cls, frame_bgr):
        with cls._lock:
            cls._warped = frame_bgr

    @classmethod
    def get_warped(cls):
        with cls._lock:
            return cls._warped
        
    @classmethod
    def set_orders(cls, img_bgr):
        with cls._lock:
            cls._orders = img_bgr

    @classmethod
    def get_orders(cls):
        with cls._lock:
            return cls._orders
        
    # ============================
    # 🔹 시나리오 모드 공유 (추가 부분)
    # ============================
    @classmethod
    def set_mode(cls, mode: str):
        """현재 시나리오 모드(Test/Restaurant/Random)를 UI와 공유"""
        with cls._lock:
            cls._mode = mode.lower() if mode else None
        print(f"[FrameBus] 시나리오 모드 저장됨 → {cls._mode}")

    @classmethod
    def get_mode(cls) -> Optional[str]:
        """UI에서 현재 모드를 읽어올 때 사용"""
        with cls._lock:
            return cls._mode
        
_CMDQ: "Queue[Tuple[str, Dict[str, Any]]]" = Queue()
_DEBUG_LOG = True 

def post(cmd: str, **kwargs: Any) -> None:
    """
    UI Thread에서 호출:
    백엔드(main 루프)가 처리할 명령을 큐에 적재.
    """
    if _DEBUG_LOG:
        print(f"[UI→BK] post cmd='{cmd}' kwargs={kwargs}")
    _CMDQ.put((cmd, kwargs))

def get_cmd_nowait() -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    """
    백엔드(main 루프)에서 비차단(non-blocking) 폴링.
    반환: (cmd, kwargs) 또는 (None, None)
    """
    try:
        return _CMDQ.get_nowait()
    except Empty:
        return None, None

def get_cmd(block: bool = True, timeout: Optional[float] = None) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    """
    백엔드(main 루프)에서 차단/타임아웃 폴링.
    block=True일 때 timeout 지정 가능.
    """
    try:
        cmd, kwargs = _CMDQ.get(block=block, timeout=timeout)
        return cmd, kwargs
    except Empty:
        return None, None

def clear_cmd_queue() -> int:
    """
    큐 비우기(디버그/리셋용). 비운 아이템 개수 반환.
    """
    cleared = 0
    try:
        while True:
            _CMDQ.get_nowait()
            cleared += 1
    except Empty:
        pass
    if _DEBUG_LOG and cleared:
        print(f"[UI→BK] cleared {cleared} pending commands")
    return cleared