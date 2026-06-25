import cv2
import numpy as np
from .config import *
from . import config

class Camera:

    def __init__(self, cam_num=0):
        # 카메라 캡처 객체 생성 (가상환경 또는 실제 카메라)
        self.cap = cv2.VideoCapture(cam_num)

        # 기본 해상도 설정 (640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

        # [투영 변환 매트릭스 계산]
        # config에 정의한 사다리꼴(SRC)과 직사각형(DST) 좌표를 매핑하는 변환 행렬 M을 생성합니다.
        self.M = cv2.getPerspectiveTransform(config.SRC_POINTS, config.DST_POINTS)

    def read(self):
        """
        카메라 프레임을 읽어와 원본 이미지와 투영 변환(Bird's Eye View) 이미지를 동시에 반환합니다.
        """
        ret, frame = self.cap.read()

        if not ret:
            return None, None

        # [사다리꼴 ROI 잘라내기 + 투영 변환 한 번에 처리]
        # 원본 이미지(640x480)에서 지정한 사다리꼴 영역만 골라 가로/세로 400x400의 평면도로 변환합니다.
        birds_eye_view = cv2.warpPerspective(
            frame, 
            self.M, 
            (config.WARP_WIDTH, config.WARP_HEIGHT)
        )

        # 이제 원본 프레임과 사다리꼴 노이즈가 제거된 평면도(BEV) 이미지를 반환합니다.
        return frame, birds_eye_view

    def release(self):
        self.cap.release()


def show_front_camera(frame, bev_image=None, lane_data=None, target=None, angle=None, speed=None):
    """
    디버깅을 위해 마스크(이진화) 처리된 평면도 화면을 실시간으로 보여주는 함수입니다.
    """
    if frame is None or bev_image is None or lane_data is None:
        return False

    # lane_detector가 넘겨준 마스크 가져오기
    white_mask = lane_data.get("white_mask")
    yellow_mask = lane_data.get("yellow_mask")

    # 마스크 창 디스플레이용 컬러 이미지 버퍼 생성 (400x400)
    # 차선이 검출된 마스크 영역을 컬러로 합성해서 시각적으로 보기 쉽게 만듭니다.
    mask_display = bev_image.copy()

    if white_mask is not None:
        # 흰색 차선 마스크가 켜진 곳을 파란색(B)으로 강조
        mask_display[white_mask > 0] = [255, 0, 0]
    
    if yellow_mask is not None:
        # 노란색 차선 마스크가 켜진 곳을 빨간색(R)으로 강조
        mask_display[yellow_mask > 0] = [0, 0, 255]

    # [스캔 라인 및 검출 점 시각화]
    # lane_detector가 실제로 스캔하는 3개의 Y 라인을 그려줍니다.
    scan_lines = [320, 350, 380]
    for y in scan_lines:
        cv2.line(mask_display, (0, y), (config.WARP_WIDTH, y), (0, 255, 0), 1)

    # 실제로 픽셀 평균으로 구한 좌/우 대표 X 점 찍기
    left_x = lane_data.get("left")
    right_x = lane_data.get("right")
    
    if left_x is not None:
        cv2.circle(mask_display, (left_x, 350), 6, (0, 255, 255), -1) # 노란색 점
    if right_x is not None:
        cv2.circle(mask_display, (right_x, 350), 6, (255, 255, 255), -1) # 흰색 점

    # 주행 조종자가 조준하고 있는 최종 Target 조향점 표시 (있을 경우)
    if target is not None:
        cv2.circle(mask_display, (int(target), config.WARP_HEIGHT - 30), 8, (0, 165, 255), -1) # 주황색 점

    # 정보 텍스트 표기
    if angle is not None:
        cv2.putText(mask_display, f"Ang: {angle:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 모니터링 창 출력
    cv2.imshow("Lane Mask & Scan Display", mask_display) # 마스크 처리 및 검출 점 화면

    key = cv2.waitKey(1) & 0xff
    if key == ord('q'):
        return True

    return False