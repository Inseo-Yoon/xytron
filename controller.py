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

    def update(self, lane_data, img_width):
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
            # 2. [우측 차선 실종 / 급우회전 케이스 - ★핵심 수정★]
            # 우회전은 여유 공간이 없으므로, 노란선(left) 기준 마진을 대폭 줄여서 
            # 차가 중앙선을 완전히 걸치고 넘어가듯 핸들을 폭발적으로 꺾게 만듭니다.
            if left > 125: 
                # 노란선이 안쪽으로 들어오면 타겟을 left + 15~20 수준으로 바짝 붙입니다.
                # 오차가 왼쪽으로 거대하게 발생하여 조향각이 즉시 최대치(-100% 방향)로 꽂힙니다.
                target = left + 20  # (기존 35 -> 20으로 축소하여 조향 대폭 강화)
            else:
                # 완만한 우회전에서도 평소보다 더 인코스를 타도록 마진을 추가로 깎습니다.
                lane_margin = int(self.last_track_width / 2) - 45  # (기존 -35 -> -45로 인코스 심화)
                target = left + lane_margin
            self.current_lane = "left_only"
            
        elif right is not None:
            # 3. [좌측 차선 실종 / 급좌회전 케이스]
            # 좌회전은 상대적으로 회전 반경에 여유가 있으므로 기존의 안정적인 마진을 유지합니다.
            if right < 275: 
                target = right - 35  
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

        if abs_angle > 20.0:
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

        return float(angle), float(speed), target