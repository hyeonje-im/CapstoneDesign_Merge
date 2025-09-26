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

    @classmethod
    def set_warped(cls, frame_bgr):
        with cls._lock:
            cls._warped = frame_bgr

    @classmethod
    def get_warped(cls):
        with cls._lock:
            return cls._warped


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
