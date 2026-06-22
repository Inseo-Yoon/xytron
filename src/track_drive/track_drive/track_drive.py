#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge

# 패키지 경로 (구조에 따라 수정하세요)
from .lane_detection.camera import Camera
from .lane_detection.lane_detector import LaneDetector
from .lane_detection.controller import Controller
from .lane_detection import config

class TrackDriverNode(Node):
    def __init__(self):
        super().__init__('track_driver_node')

        # 1. 초기화
        self.bridge = CvBridge()
        self.cam_handler = Camera()
        self.detector = LaneDetector()
        self.controller = Controller()

        # 상태 관리
        self.current_traffic_action = "WAIT_START"
        self.last_logged_traffic_action = None
        self.lane_departure = False

        # 2. 퍼블리셔 & 서브스크라이버
        self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)

        # 이미지 콜백
        self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
        # 신호등 콜백
        self.create_subscription(String, '/traffic_action', self.traffic_callback, 10)
        # 차선 이탈 콜백 (필요 시)
        self.create_subscription(Bool, "/lane_departure", self.lane_departure_callback, 10)

    def traffic_callback(self, msg):
        self.current_traffic_action = msg.data
        if self.current_traffic_action != self.last_logged_traffic_action:
            self.get_logger().info(f"traffic_action: {self.current_traffic_action}")
            self.last_logged_traffic_action = self.current_traffic_action

    def lane_departure_callback(self, msg):
        self.lane_departure = msg.data

    def image_callback(self, msg):
        try:
            # 1. 영상 전처리 (BEV)
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, bev = self.cam_handler.read(frame)
            if bev is None:
                self.publish_drive(0, 0)
                return

            # 2. 차선 인식
            lane_data = self.detector.detect(bev)

            # 3. [핵심] Controller에 신호등 상태와 함께 전달
            # Controller 내부에서 traffic_action을 확인하여 0, 0을 반환하거나 주행 로직을 수행함
            angle, speed, _ = self.controller.update(lane_data, config.WARP_WIDTH, self.current_traffic_action)

            # 4. 차선 이탈 시 속도 보정 (Controller 외부 예외 처리)
            if self.lane_departure:
                speed = 3.0

            # 5. 최종 발행
            self.publish_drive(angle, speed)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def publish_drive(self, angle, speed):
        motor_msg = XycarMotor()
        motor_msg.angle = float(angle)
        motor_msg.speed = float(speed)
        self.motor_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrackDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전하게 정지 후 종료
        node.publish_drive(0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
