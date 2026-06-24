import cv2
import numpy as np
from . import config


class SchoolZoneDetector:
    """
    EasyOCR 없이 OpenCV만으로 어린이 보호구역을 감지합니다.

    원리:
      - BEV 하단 ROI에서 노란색 픽셀 면적을 측정
      - 보호구역 진입 시: 노란 텍스트 덩어리가 크게 등장  → ENTER_THRESHOLD 이상
      - 보호구역 해제 시: 노란 텍스트가 사라짐            → EXIT_THRESHOLD 이하
      - confirm_frames 연속으로 같은 결과일 때만 상태 전환 (노이즈 방지)

    사용 예시:
        detector = SchoolZoneDetector()
        in_zone = detector.detect(bev_image)
    """

    def __init__(self,
                 enter_threshold: int = 5500,
                 exit_threshold:  int = 4800,
                 confirm_frames:  int = 3):
        """
        Args:
            enter_threshold : 보호구역 진입 판단 기준 노란 픽셀 수 (기본 800)
            exit_threshold  : 보호구역 해제 판단 기준 노란 픽셀 수 (기본 300)
            confirm_frames  : 상태 전환 확정까지 연속 감지 횟수 (노이즈 방지)
        """
        self.enter_threshold = enter_threshold
        self.exit_threshold  = exit_threshold
        self.confirm_frames  = confirm_frames

        # 내부 상태
        self.in_school_zone = False
        self._enter_count   = 0
        self._exit_count    = 0

        print("[SchoolZoneDetector] 초기화 완료 (OpenCV 경량 모드)")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, bev_image) -> bool:
        """
        매 프레임마다 호출합니다. (무거운 연산 없으므로 매 프레임 OK)

        Returns:
            bool : True면 어린이 보호구역 내부, False면 일반 구간
        """
        if bev_image is None:
            return self.in_school_zone

        yellow_pixel_count = self._count_yellow_pixels(bev_image)
        self._update_state(yellow_pixel_count)

        return self.in_school_zone

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_roi(self, bev_image):
        """
        BEV 이미지 하단 중앙 ROI 크롭.
        텍스트는 차량 바로 앞(하단 40%) + 좌우 15% 여백 제거 영역에 등장.
        """
        h, w = bev_image.shape[:2]
        x1 = int(w * 0.15)
        x2 = int(w * 0.85)
        y1 = int(h * 0.60)   # 하단 40% 만 봄
        y2 = h
        return bev_image[y1:y2, x1:x2]

    def _count_yellow_pixels(self, bev_image) -> int:
        """
        ROI 내 노란색 픽셀 수를 반환합니다.
        config의 LOWER_YELLOW / UPPER_YELLOW 범위를 그대로 재사용합니다.
        """
        roi = self._get_roi(bev_image)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, config.LOWER_YELLOW, config.UPPER_YELLOW)

        # 모폴로지로 잡음 제거
        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN,  kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        return int(np.sum(yellow_mask > 0))

    def _update_state(self, pixel_count: int):
        print(f"[DEBUG] yellow pixel: {pixel_count}")
        """
        픽셀 수를 기준으로 진입/해제 상태를 confirm_frames 연속 감지 후 전환합니다.
        """
        # --- 진입 판단 ---
        if not self.in_school_zone and pixel_count >= self.enter_threshold:
            self._enter_count += 1
            self._exit_count   = 0
            if self._enter_count >= self.confirm_frames:
                self.in_school_zone = True
                self._enter_count   = 0
                print(f"[SchoolZoneDetector] ★ 어린이 보호구역 진입 → 감속 (pixel={pixel_count})")

        # --- 해제 판단 ---
        elif self.in_school_zone and pixel_count <= self.exit_threshold:
            self._exit_count  += 1
            self._enter_count  = 0
            if self._exit_count >= self.confirm_frames:
                self.in_school_zone = False
                self._exit_count    = 0
                print(f"[SchoolZoneDetector] ★ 보호구역 해제 → 정상 속도 복귀 (pixel={pixel_count})")

        # --- 변화 없음: 카운터만 리셋 ---
        else:
            self._enter_count = 0
            self._exit_count  = 0
