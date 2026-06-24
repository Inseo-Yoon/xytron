import numpy as np
from .config import *
from . import config

class Controller:

    def __init__(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_lane = "both"
        self.cooldown_counter = 0
        self.COOLDOWN_FRAMES = 20
        self.last_track_width = 220

    def calculate_pid(self, error):
        abs_error = abs(error)

        # 초급커브 구간에서 짤짤이 제어 대신 풀 스티어링이 먹히도록 배율 상향
        if abs_error > 35:
            dynamic_kp = config.KP * 2.2  # 묵직하게 끝까지 꺾음
        elif abs_error > 20:
            dynamic_kp = config.KP * 1.5
        else:
            dynamic_kp = config.KP

        p_term = dynamic_kp * error
        self.integral += error
        self.integral = np.clip(self.integral, -50, 50)
        i_term = config.KI * self.integral

        d_term = (config.KD * 1.3) * (error - self.prev_error)
        self.prev_error = error

        return np.clip(p_term + i_term + d_term, config.MIN_STEERING, config.MAX_STEERING)

    def update(self, lane_data, img_width, traffic_action, in_school_zone=False):
        """ Controller 클래스 내부로 들여쓰기를 맞춰 정렬했습니다 """
        if traffic_action in ["STOP", "WAIT_START"]:
            return 0.0, 0.0, None

        left = lane_data.get("left")
        right = lane_data.get("right")
        center_x = img_width / 2  # 200

        if left is not None and right is not None:
            # 1. 양쪽 다 보일 때
            current_width = right - left
            if current_width > 120:
                self.last_track_width = current_width
            target = int((left + right) / 2)
            self.current_lane = "both"

        elif left is not None:
            # 세 번째 사진을 보면 left 점이 약 140~150 사이에 도달해 있습니다.
            # 이 조건이 켜지면 타겟을 숏컷 방향인 left + 30 (약 170~180) 부근으로 강제 고정합니다.
            if left > 130:
                target = left + 30  # 인코스로 더 날카롭게 파고들도록 수정
            else:
                lane_margin = int(self.last_track_width / 2) - 35
                target = left + lane_margin
            self.current_lane = "left_only"

        elif right is not None:
            # 반대편 급좌회전 대응 대칭 보정
            if right < 270:
                target = right - 30
            else:
                lane_margin = int(self.last_track_width / 2) - 35
                target = right - lane_margin
            self.current_lane = "right_only"

        else:
            # 4. 양쪽 다 놓친 비상 상황
            if self.prev_error > 0:
                target = int(center_x + 70)
            else:
                target = int(center_x - 70)
            self.current_lane = "lost"

        # 오차 및 조향 계산
        error = target - center_x
        angle = self.calculate_pid(error)
        abs_angle = abs(angle)

        if abs_angle > 15.0:
            self.cooldown_counter = self.COOLDOWN_FRAMES

        # 속도 제어 루틴 추가 (이전 업데이트본 반영)
        if self.current_lane != "both" or abs_angle > 40.0:
            speed = config.SHARP_SPEED
        elif abs_angle > 15.0 or self.cooldown_counter > 0:
            speed = config.CURVE_SPEED
            if self.cooldown_counter > 0:
                self.cooldown_counter -= 1
        else:
            speed = config.STRAIGHT_SPEED

        if traffic_action in ["RED_APPROACH", "YELLOW_APPROACH", "SIGNAL_APPROACH"]:
            speed = min(speed, config.SIGNAL_APPROACH_SPEED)
        elif traffic_action == "LEFT":
            speed = min(speed, config.LEFT_TURN_SPEED)

        # 어린이 보호구역 진입 시 속도 상한 적용
        if in_school_zone:
            speed = min(speed, config.SCHOOL_ZONE_SPEED)

        return float(angle), float(speed), target
