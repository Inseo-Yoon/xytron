#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool

from cv_bridge import CvBridge

from xycar_msgs.msg import XycarMotor

from track_drive.traffic_cone import TrafficCone


# ── 모드 상수 ──────────────────────────────────────────────
MODE_TRAFFIC    = "TRAFFIC"    # 신호등 구간 (차선 주행)
MODE_CONE       = "CONE"       # 라바콘 구간
MODE_LANE       = "LANE"       # 차선 인식 구간


class TrackDriverNode(Node):

    def __init__(self):
        super().__init__("driver")
        self.get_logger().info("----- Xycar self-driving node started -----")

        # ── 센서 데이터 ────────────────────────────────────
        self.image        = None
        self.lidar_ranges = None

        # ── 차선 인식 데이터 ───────────────────────────────
        self.lane_angle     = 0.0
        self.lane_departure = False

        # ── 신호등 데이터 ──────────────────────────────────
        self.traffic_action    = "WAIT_START"
        self.last_traffic_time = None
        self.start_released    = True   # ★ 신호 없이 바로 출발

        # ── 모드 ───────────────────────────────────────────
        self.mode       = MODE_TRAFFIC
        self.mode_start = None   # 모드 진입 시각

        # 라바콘 구간 지속 시간 (초) — 환경에 맞게 조정
        self.CONE_DURATION = 20.0
        # 신호등 통과 후 라바콘 진입까지 대기 시간 (초)
        self.CONE_ENTER_DELAY = 5.0

        self.bridge    = CvBridge()
        self.motor_msg = XycarMotor()
        self.cone      = TrafficCone()

        # ── Publisher ──────────────────────────────────────
        self.motor_pub = self.create_publisher(XycarMotor, "xycar_motor", 10)

        # ── Subscribers ────────────────────────────────────
        self.create_subscription(
            Image, "/usb_cam/image_raw/front",
            self.cam_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan",
            self.lidar_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32, "/lane_angle",
            self.lane_angle_callback, 10
        )
        self.create_subscription(
            Bool, "/lane_departure",
            self.lane_departure_callback, 10
        )
        self.create_subscription(
            String, "/traffic_action",
            self.traffic_action_callback, 10
        )

        self.get_logger().info("Track Driver Node Initialized")

    # ── Callbacks ──────────────────────────────────────────

    def cam_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges

    def lane_angle_callback(self, msg):
        self.lane_angle = msg.data

    def lane_departure_callback(self, msg):
        self.lane_departure = msg.data

    def traffic_action_callback(self, msg):
        self.traffic_action    = msg.data
        self.last_traffic_time = time.monotonic()
        if msg.data in ("GO", "LEFT"):
            self.start_released = True

    # ── Drive ──────────────────────────────────────────────

    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    # ── Main Loop ──────────────────────────────────────────

    def main_loop(self):
        self.get_logger().info("======================================")
        self.get_logger().info("  S T A R T    D R I V I N G ...      ")
        self.get_logger().info("======================================")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.image is None:
                continue

            # ── 라바콘 주행 ───────────────────────────────
            angle, speed = self.cone.get_control(
                self.image, self.lidar_ranges
            )
            self.drive(angle, speed)


def main(args=None):
    rclpy.init(args=args)
    node = TrackDriverNode()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(angle=0, speed=0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()