#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import rclpy, time, cv2, os, math
import numpy as np
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

# [새로운 파이프라인 컴포넌트들 Import]
from .lane_detection.camera import Camera, show_front_camera
from .lane_detection.lane_detector import LaneDetector
from .controller import Controller          # lane_detection → 최상위
from . import config                        # lane_detection → 최상위
from .overtake import OvertakeManager       # 추가
# 기존 두 줄 삭제하고 이걸로 교체
from .overtake.lidar_analyzer import get_front_distance, get_left_distance, find_noise_boundaries


#=============================================
# ROS2 Node 클래스 정의
#=============================================
class TrackDriverNode(Node):

    #=============================================
    # 클래스 생성 초기화 함수
    #=============================================
    def __init__(self):
        super().__init__('driver')
        self.get_logger().info('----- Xycar self-driving node started -----')
        
        # 상수값 및 초기값 설정
        self.image = None              # 카메라 토픽 원본 데이터를 저장할 변수
        self.motor_msg = XycarMotor()  # 모터토픽 메시지        
        self.lidar_ranges = None
        self.bridge = CvBridge()
        
        self.angle = 0.0
        self.speed = 0.0

        # 우리가 설계한 새로운 구조의 인스턴스 생성
        # camera의 캡처 루프는 ROS2 서브스크립션이 대체하므로, 투영 행렬 M 연산용으로만 활용합니다.
        self.cam_handler = Camera(cam_num=0) 
        self.detector = LaneDetector()
        self.controller = Controller()
        self.overtake_manager = OvertakeManager()

        # ROS2 Publisher & Subscriber 설정
        self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        
        self.sub_front = self.create_subscription(
            Image, '/usb_cam/image_raw/front', self.cam_callback, qos_profile_sensor_data)

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        
        self.get_logger().info("Track Driver Node Initialized")
              
    #=============================================
    # 카메라 토픽을 수신하는 콜백 함수
    #=============================================
    def cam_callback(self, data):
        # ROS2 이미지 메시지를 OpenCV 이미지(BGR8) 포맷으로 변환
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # [핵심 변경 포인트: 투영 변환 적용]
        # 단순 사각형 slice 대신 camera.py 내부의 원근 변환 매트릭스(M)를 사용해 평면도를 생성합니다.
        bev_image = cv2.warpPerspective(
            self.image, 
            self.cam_handler.M, 
            (config.WARP_WIDTH, config.WARP_HEIGHT)
        )

        # 펴진 평면도(400x400) 위에서 색상을 거르고 차선 데이터를 추출합니다.
        lane_data = self.detector.detect(bev_image)

        # 컨트롤러에서 차선 위치 분석 후 조향각(angle), 속도(speed), 디버깅용 target 점을 반환받습니다.
        # 기존 roi.shape[1] 대신 평면도 이미지의 너비(WARP_WIDTH = 400)를 전달합니다.
        angle, speed, target = self.controller.update(
            lane_data,
            config.WARP_WIDTH
        )

        # 실시간 모터 명령을 위해 저장
        self.angle, self.speed, _ = self.overtake_manager.update(
            lane_angle=angle,
            lane_speed=speed,
            lidar_ranges=self.lidar_ranges
        )
        # 디버깅 창 시각화 (원본 프레임과 평면도를 동시에 모니터링)
        should_quit = show_front_camera(
            frame=self.image,
            bev_image=bev_image,
            lane_data=lane_data,
            target=target,
            angle=angle,
            speed=speed
        )

        # 디버깅 창에서 'q'를 누르면 안전하게 ROS2를 셧다운합니다.
        if should_quit:
            self.get_logger().info("Quit signal received. Shutting down...")
            self.drive(0, 0)
            rclpy.shutdown()

    #=============================================
    # 라이다 토픽을 수신하는 콜백 함수
    #=============================================
    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges   
      
    #=============================================
    # 모터제어 토픽을 발행하는 Publisher 함수
    #=============================================
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    #=============================================
    # 메인 주행 루프
    #=============================================
    def main_loop(self):
        self.get_logger().info("======================================")
        self.get_logger().info("   S T A R T    D R I V I N G ...     ")
        self.get_logger().info("======================================")

        # rclpy.ok()인 동안 찰나의 타임아웃으로 이벤트를 체크하고 즉시 모터 명령을 전달합니다.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.00001)
            self.drive(angle=self.angle, speed=self.speed)
                
#=============================================
# 메인 함수
#=============================================
def main(args=None):
    rclpy.init(args=args)
    node = TrackDriverNode()
    
    try:
        node.main_loop()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 종료 시 차량을 확실하게 멈추고 윈도우 창을 파괴합니다.
        if rclpy.ok():
            node.drive(angle=0, speed=0)
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()