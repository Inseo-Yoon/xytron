import cv2
import numpy as np
from .config import *
from . import config

class LaneDetector:

    def __init__(self):
        # [수정] 기존 [300, 330, 360, 390]에서 위쪽 영역(220, 260)을 추가로 포함합니다.
        # 이렇게 하면 코너가 다가올 때 멀리서 꺾이는 차선을 한발 빠르게 감지할 수 있습니다.
        self.scan_lines = [220, 270, 320, 370]
        
        # 이전 값 기억 변수 유지
        self.last_left_x = 90
        self.last_right_x = 310

    def color_mask(self, bev_image):
        """
        평면도 이미지에서 config에 정의된 임계값으로 흰색과 노란색 차선을 분리합니다.
        """
        hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)

        white_mask = cv2.inRange(hsv, config.LOWER_WHITE, config.UPPER_WHITE)
        yellow_mask = cv2.inRange(hsv, config.LOWER_YELLOW, config.UPPER_YELLOW)

        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        return white_mask, yellow_mask

    def scan_lane_x(self, mask, y_pixels, x_start=0, x_end=400, default_x=None):
        """
        지정된 Y 좌표들에서 픽셀 값이 255인 X 좌표들을 추출합니다.
        x_start, x_end를 주어 좌/우 탐색 영역을 완전히 분리할 수 있습니다.
        """
        x_indices = []
        for y in y_pixels:
            # 지정된 X 탐색 범위 안에서만 픽셀 검출
            found_x = np.where(mask[y, x_start:x_end] > 0)[0]
            if len(found_x) > 0:
                # x_start 만큼 오프셋을 더해 정확한 전체 좌표계로 복원
                x_indices.extend(found_x + x_start)
                
        # 4개의 라인 중 어디라도 걸린 픽셀이 있다면 평균값 반환
        if len(x_indices) > 0:
            return int(np.mean(x_indices))
            
        # [핵심] 점선 빈 공간이라 하나도 검출 안 되었다면 직전 프레임의 위치(메모리) 반환!
        return default_x

    def detect(self, bev_image):
        white_mask, yellow_mask = self.color_mask(bev_image)

        # [수정] yellow_mask 전체를 세지 말고, 아래쪽 절반(행 200번부터 끝까지)만 셉니다.
        # 이렇게 하면 상단의 잔디 노이즈나 멀리 있는 중앙선 때문에 뻥튀기되는 걸 막아줍니다.
        yellow_pixel_count = np.count_nonzero(yellow_mask[200:, :]) 

        # 좌우 탐색 (기존 코드 유지)
        left_x = self.scan_lane_x(yellow_mask, self.scan_lines, x_start=0, x_end=190, default_x=self.last_left_x)
        right_x = self.scan_lane_x(white_mask, self.scan_lines, x_start=210, x_end=400, default_x=self.last_right_x)

        if left_x is not None: self.last_left_x = left_x
        if right_x is not None: self.last_right_x = right_x

        return {
            "left": left_x, "right": right_x, "yellow": left_x,
            "white_mask": white_mask, "yellow_mask": yellow_mask,
            "yellow_count": yellow_pixel_count
        }