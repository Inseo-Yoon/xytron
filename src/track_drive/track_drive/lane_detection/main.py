import cv2
import sys
import os

# 상대 경로 import를 위한 패스 설정 (동작 오류 방지)
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from camera import Camera, show_front_camera
from config import *
import config
from school_zone_detector import SchoolZoneDetector

def main():
    # 카메라 객체 초기화
    # 가상환경 시뮬레이터 윈도우 캡처 방식에 따라 cam_num은 다를 수 있습니다 (기본 0)
    camera = Camera(cam_num=0)
    school_zone_detector = SchoolZoneDetector(check_interval=10, confirm_frames=2)

    print("=== ROI 및 투영 변환 디버깅 시작 ===")
    print("종료하려면 영상 창을 선택하고 'q' 키를 누르세요.")

    while True:
        # camera.py에서 작성한 원본 프레임과 평면도(BEV) 영상을 가져옵니다.
        frame, bev_image = camera.read()

        if frame is None:
            print("카메라 프레임을 읽을 수 없습니다.")
            break

        # 어린이 보호구역 감지
        in_school_zone = school_zone_detector.detect(bev_image)

        # [디버깅의 핵심]
        # 우리가 정의한 camera.py의 show_front_camera 기능을 이용해 두 창을 동시에 띄웁니다.
        # 아직 차선 데이터나 조향각이 없으므로 뒤쪽 인자들은 None으로 둡니다.
        is_quit = show_front_camera(
            frame=frame,
            bev_image=bev_image,
            lane_data=None,
            target=None,
            angle=None,
            speed=None
        )

        # 'q' 키가 눌렸다면 디버깅 루프 종료
        if is_quit:
            break

    camera.release()
    cv2.destroyAllWindows()
    print("=== 디버깅 종료 ===")

if __name__ == "__main__":
    main()