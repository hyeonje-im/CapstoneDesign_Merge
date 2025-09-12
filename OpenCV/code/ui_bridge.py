import threading  

class FrameBus:   
    # 클래스 단위 공유 리소스
    _lock = threading.Lock()  # 여러 스레드가 동시에 접근하지 못하도록 보호
    _video = None             # 최신 비디오 프레임 (OpenCV BGR ndarray)
    _grid  = None             # 최신 그리드 프레임 (OpenCV BGR ndarray)

    @classmethod
    def set_video(cls, frame_bgr):
        
        with cls._lock:       # lock 구간: set/get이 동시에 실행되지 않도록 보장
            cls._video = frame_bgr

    @classmethod
    def get_video(cls):
       
        with cls._lock:       # lock으로 스레드 충돌 방지
            return cls._video

    @classmethod
    def set_grid(cls, frame_bgr):
        
        with cls._lock:
            cls._grid = frame_bgr

    @classmethod
    def get_grid(cls):
        
        with cls._lock:
            return cls._grid
