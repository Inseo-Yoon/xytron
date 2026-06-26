# overtake_manager.py
from .lidar_analyzer import (
    is_front_car_detected,
    get_front_distance,
    get_left_distance,
)

class OvertakeManager:
    def __init__(self):
        self.TARGET_SPEED = 17.0
        self.SAFE_DIST = 6.0
        self.MIN_DIST = 2.0
        self.MAX_DIST = 8.0

        self.phase = 0  # 0: 대기, 1: 왼쪽 진입, 2: 추월 가속, 3: 오른쪽 복귀
        self.phase_frames = 0
        self.overtake_done = False

        # 튜닝 포인트
        self.PHASE1_ANGLE    = -30.0  # 왼쪽으로 틀기
        self.PHASE1_DURATION = 20     # 프레임 수

        self.PHASE2_ANGLE    = 0.0    # 직진
        self.PHASE2_SPEED    = 20.0   # 풀가속
        self.PHASE2_DURATION = 40     # 프레임 수

        self.PHASE3_ANGLE    = 20.0   # 오른쪽 복귀
        self.PHASE3_DURATION = 20     # 프레임 수

    def update(self, lane_angle, lane_speed, lidar_ranges):
        if lidar_ranges is None:
            return lane_angle, lane_speed, "NORMAL"

        front_dist = get_front_distance(lidar_ranges) if is_front_car_detected(lidar_ranges) else None
        left_dist  = get_left_distance(lidar_ranges)

        # ── 1. ACC ──
        current_speed = self._calc_acc_speed(front_dist, lane_speed)
        current_angle = lane_angle
        state = "NORMAL"

        # ── 2. 트리거 ──
        if (not self.overtake_done
                and self.phase == 0
                and current_speed > 0
                and left_dist < 2.5):
            self.phase = 1
            self.phase_frames = 0

        # ── 3. 페이즈 실행 ──
        if self.phase == 1:
            # 왼쪽으로 틀기
            current_angle = lane_angle + self.PHASE1_ANGLE
            current_speed = self.TARGET_SPEED
            state = "PHASE1_LEFT"
            self.phase_frames += 1
            if self.phase_frames >= self.PHASE1_DURATION:
                self.phase = 2
                self.phase_frames = 0

        elif self.phase == 2:
            # 직진 + 풀가속
            current_angle = lane_angle + self.PHASE2_ANGLE
            current_speed = self.PHASE2_SPEED
            state = "PHASE2_ACCEL"
            self.phase_frames += 1
            if self.phase_frames >= self.PHASE2_DURATION:
                self.phase = 3
                self.phase_frames = 0

        elif self.phase == 3:
            # 오른쪽 복귀
            current_angle = lane_angle + self.PHASE3_ANGLE
            current_speed = self.TARGET_SPEED
            state = "PHASE3_RIGHT"
            self.phase_frames += 1
            if self.phase_frames >= self.PHASE3_DURATION:
                self.phase = 0
                self.overtake_done = True
                state = "NORMAL"

        print(f"Front: {f'{front_dist:.2f}' if front_dist is not None else 'None'} | "
              f"Left: {f'{left_dist:.2f}' if left_dist is not None else 'None'} | "
              f"Speed: {current_speed:.2f} | Angle: {current_angle:.2f} | State: {state}")

        return current_angle, current_speed, state

    def _calc_acc_speed(self, front_dist, lane_speed):
        if front_dist is None:
            return lane_speed

        if front_dist <= self.MIN_DIST:
            return -self.TARGET_SPEED

        if front_dist >= self.MAX_DIST:
            return self.TARGET_SPEED

        if front_dist < self.SAFE_DIST:
            ratio = (front_dist - self.MIN_DIST) / (self.SAFE_DIST - self.MIN_DIST)
            return self.TARGET_SPEED * (ratio - 1.0)
        else:
            ratio = (front_dist - self.SAFE_DIST) / (self.MAX_DIST - self.SAFE_DIST)
            return self.TARGET_SPEED * ratio