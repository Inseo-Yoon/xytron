#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from xycar_msgs.msg import XycarMotor


class TrackDriverNode(Node):
    def __init__(self):
        super().__init__("driver")
        self.get_logger().info("----- Xycar self-driving node started -----")

        self.motor_msg = XycarMotor()
        self.lane_angle = 0.0
        self.lane_departure = False
        self.lidar_ranges = None
        self.traffic_action = "WAIT_START"
        self.last_traffic_time = None
        self.start_released = False
        self.signal_approach_speed = 10.0
        self.red_approach_speed = 8.5

        self.motor_pub = self.create_publisher(XycarMotor, "xycar_motor", 10)

        self.lane_angle_sub = self.create_subscription(
            Float32,
            "/lane_angle",
            self.lane_angle_callback,
            10,
        )

        self.lane_departure_sub = self.create_subscription(
            Bool,
            "/lane_departure",
            self.lane_departure_callback,
            10,
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            qos_profile_sensor_data,
        )

        self.traffic_action_sub = self.create_subscription(
            String,
            "/traffic_action",
            self.traffic_action_callback,
            10,
        )

        self.get_logger().info("Track Driver Node Initialized")

    def lane_angle_callback(self, msg):
        self.lane_angle = msg.data

    def lane_departure_callback(self, msg):
        self.lane_departure = msg.data

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges

    def traffic_action_callback(self, msg):
        self.traffic_action = msg.data
        self.last_traffic_time = time.monotonic()
        if msg.data in ("GO", "LEFT"):
            self.start_released = True

    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    def main_loop(self):
        self.get_logger().info("======================================")
        self.get_logger().info("  S T A R T    D R I V I N G ...      ")
        self.get_logger().info("======================================")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            speed = max(8.0, 20.0 - 0.25 * abs(self.lane_angle))
            if self.lane_departure:
                speed = min(speed, 3.0)

            angle = self.lane_angle
            traffic_missing = (
                self.last_traffic_time is None
                or time.monotonic() - self.last_traffic_time > 0.5
            )

            if traffic_missing or self.traffic_action in ("WAIT_START", "STOP"):
                speed = 0.0
            elif not self.start_released:
                speed = 0.0
            elif self.traffic_action == "RED_APPROACH":
                speed = self.red_approach_speed
            elif self.traffic_action == "SIGNAL_APPROACH":
                speed = self.signal_approach_speed
            elif self.traffic_action == "SLOW":
                speed = min(speed, 2.0)
            elif self.traffic_action == "LEFT":
                speed = min(speed, 3.0)
                angle = min(angle, -25.0)

            self.drive(angle=angle, speed=speed)
            time.sleep(0.1)


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
