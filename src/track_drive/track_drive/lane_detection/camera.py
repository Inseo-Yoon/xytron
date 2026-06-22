import cv2
from . import config

class Camera:
    def __init__(self):
        self.M = cv2.getPerspectiveTransform(config.SRC_POINTS, config.DST_POINTS)

    def read(self, frame):
        """
        ROS Image 콜백에서 받은 프레임을 Bird's Eye View로 변환합니다.
        """
        if frame is None:
            return None, None

        birds_eye_view = cv2.warpPerspective(
            frame,
            self.M,
            (config.WARP_WIDTH, config.WARP_HEIGHT)
        )

        return frame, birds_eye_view

    def release(self):
        pass


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
