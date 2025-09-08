# tracking.py
import collections

class MovingAverageTracker:
    """ 개별 태그의 이동 평균 좌표 및 속도를 추적하는 클래스 """
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.positions = collections.deque()

    def update(self, x, y, t):
        """ 위치 및 시간 업데이트 (최대 window_size개의 데이터 유지) """
        self.positions.append((x, y, t))
        while len(self.positions) > self.window_size:
            self.positions.popleft()
    
    def get_smoothed_position(self):
        """ 최근 위치들의 평균을 반환 """
        n = len(self.positions)
        if n == 0:
            return 0.0, 0.0
        sum_x, sum_y = 0.0, 0.0
        for (x, y, _) in self.positions:
            sum_x += x
            sum_y += y
        return sum_x / n, sum_y / n
    
    def get_average_velocity(self):
        """ 최근 데이터의 시작과 끝을 비교해 속도를 계산 """
        n = len(self.positions)
        if n < 2:
            return 0.0, 0.0
        
        x_old, y_old, t_old = self.positions[0]
        x_new, y_new, t_new = self.positions[-1]
        dx = x_new - x_old
        dy = y_new - y_old
        dt = t_new - t_old
        if dt == 0:
            return 0.0, 0.0
        return dx / dt, dy / dt


class TrackingManager:
    """ 여러 개의 태그에 대해 tracker를 자동 관리하는 클래스 """
    def __init__(self, window_size=5):
        self.trackers = {}  # tag_id별 Tracker 저장
        self.window_size = window_size

    def update_all(self, tag_info, current_time):
        """
        모든 태그의 위치를 업데이트하고 이동 평균 좌표 & 속도를 계산한 후 tag_info에 반영
        """
        for tag_id, data in tag_info.items():
            raw_x, raw_y = data["coordinates"]

            # 해당 태그의 tracker가 없으면 새로 생성
            if tag_id not in self.trackers:
                self.trackers[tag_id] = MovingAverageTracker(window_size=self.window_size)
            
            # 트래커 업데이트
            tracker = self.trackers[tag_id]
            tracker.update(raw_x, raw_y, current_time)

            # 이동 평균 좌표 & 속도 계산 후 tag_info에 저장
            avg_x, avg_y = tracker.get_smoothed_position()
            vx, vy = tracker.get_average_velocity()

            data["smoothed_coordinates"] = (avg_x, avg_y)
            data["velocity"] = (vx, vy)

            
