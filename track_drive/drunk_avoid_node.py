#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from xycar_msgs.msg import XycarMotor


class DrunkAvoidNode(Node):
    FRONT_HALF_ANGLE = math.radians(45.0)
    EMERGENCY_STOP_DISTANCE = 0.35
    STOP_ENTER_DISTANCE = 0.8
    STOP_RELEASE_DISTANCE = 1.0
    AVOID_DISTANCE = 2.0
    ACTIVE_HOLD_SEC = 1.5
    MIN_BLOB_AREA = 1200.0
    CAMERA_STOP_AREA_RATIO = 0.22
    CAMERA_AVOID_AREA_RATIO = 0.08
    CAMERA_STOP_HEIGHT_RATIO = 0.60
    CAMERA_AVOID_HEIGHT_RATIO = 0.35
    AVOID_SPEED = 5.0
    AVOID_ANGLE = 45.0
    FAST_AVOID_SPEED = 7.0
    FAST_AVOID_ANGLE = 55.0
    CLEARING_SPEED = 8.0
    CLEARING_ANGLE = 35.0
    CLEARING_SEC = 1.0
    APPROACH_DISTANCE_RATE = 0.35
    APPROACH_AREA_RATE = 0.04
    APPROACH_HEIGHT_RATE = 0.08
    ROI_TOP_RATIO = 0.45

    def __init__(self):
        super().__init__('drunk_avoid_node')

        self.bridge = CvBridge()
        self.blue_detected = False
        self.blue_center_x = None
        self.blue_area_ratio = 0.0
        self.blue_height_ratio = 0.0
        self.blue_aspect_ratio = 0.0
        self.image_width = None
        self.roi_top = 0
        self.front_min_distance = math.inf
        self.last_risk_time = None
        self.last_cmd = self.make_motor_msg(0.0, 0.0)
        self.stop_latched = False
        self.state = 'IDLE'
        self.last_log_time = 0.0
        self.prev_measure_time = None
        self.prev_front_min_distance = math.inf
        self.prev_blue_area_ratio = 0.0
        self.prev_blue_height_ratio = 0.0
        self.closing_rate = 0.0
        self.area_growth_rate = 0.0
        self.height_growth_rate = 0.0
        self.approaching = False
        self.prev_stop_required = False
        self.clearing_until = 0.0
        self.last_avoid_angle = 0.0

        self.active_pub = self.create_publisher(Bool, '/drunk_avoid_active', 10)
        self.cmd_pub = self.create_publisher(XycarMotor, '/drunk_avoid_cmd', 10)

        self.create_subscription(
            Image,
            '/usb_cam/image_raw/front',
            self.image_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.create_timer(0.1, self.publish_avoid_state)
        self.get_logger().info('Drunk Avoid Node Initialized')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert camera image: {exc}')
            return

        self.image_width = image.shape[1]
        image_height = image.shape[0]
        self.roi_top = int(image_height * self.ROI_TOP_RATIO)
        roi = image[self.roi_top:, :]
        roi_area = roi.shape[0] * roi.shape[1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 50, 30], dtype=np.uint8)
        upper_blue = np.array([135, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.clear_blue_detection()
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.MIN_BLOB_AREA:
            self.clear_blue_detection()
            return

        moments = cv2.moments(largest)
        if moments['m00'] == 0:
            self.clear_blue_detection()
            return

        _, _, width, height = cv2.boundingRect(largest)
        self.blue_detected = True
        self.blue_center_x = int(moments['m10'] / moments['m00'])
        self.blue_area_ratio = area / roi_area
        self.blue_height_ratio = height / roi.shape[0]
        self.blue_aspect_ratio = height / max(width, 1)

    def scan_callback(self, msg):
        valid_front_ranges = []
        for index, distance in enumerate(msg.ranges):
            angle = self.normalize_angle(msg.angle_min + index * msg.angle_increment)
            if angle < -self.FRONT_HALF_ANGLE or angle > self.FRONT_HALF_ANGLE:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            valid_front_ranges.append(distance)

        self.front_min_distance = min(valid_front_ranges) if valid_front_ranges else math.inf

    def publish_avoid_state(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.update_approach_estimate(now)
        front_close = self.front_min_distance < self.AVOID_DISTANCE
        emergency_stop = self.blue_detected and self.front_min_distance < self.EMERGENCY_STOP_DISTANCE
        camera_stop = (
            self.blue_detected and
            (self.blue_height_ratio >= self.CAMERA_STOP_HEIGHT_RATIO or
             (self.blue_area_ratio >= self.CAMERA_STOP_AREA_RATIO and
              self.blue_height_ratio >= self.CAMERA_AVOID_HEIGHT_RATIO))
        )
        camera_avoid = (
            self.blue_detected and
            self.blue_aspect_ratio >= 0.7 and
            (self.blue_area_ratio >= self.CAMERA_AVOID_AREA_RATIO or
             self.blue_height_ratio >= self.CAMERA_AVOID_HEIGHT_RATIO)
        )
        if self.blue_detected and self.front_min_distance < self.STOP_ENTER_DISTANCE:
            self.stop_latched = True
        elif not self.blue_detected or self.front_min_distance > self.STOP_RELEASE_DISTANCE:
            self.stop_latched = False

        stop_required = emergency_stop or self.stop_latched or camera_stop
        if self.prev_stop_required and not stop_required:
            self.clearing_until = now + self.CLEARING_SEC

        clearing_required = now < self.clearing_until
        fast_avoid_required = self.approaching and self.blue_detected
        avoid_required = (self.blue_detected and front_close) or camera_avoid

        if stop_required:
            active = True
            self.state = 'STOP'
            self.last_cmd = self.make_motor_msg(0.0, 0.0)
            self.last_risk_time = now
        elif clearing_required:
            active = True
            self.state = 'CLEARING'
            self.last_cmd = self.make_clearing_cmd()
            self.last_risk_time = now
        elif fast_avoid_required:
            active = True
            self.state = 'FAST_AVOIDING'
            self.last_cmd = self.make_avoid_cmd(self.FAST_AVOID_ANGLE, self.FAST_AVOID_SPEED)
            self.last_risk_time = now
        elif avoid_required:
            active = True
            self.state = 'AVOIDING'
            self.last_cmd = self.make_avoid_cmd()
            self.last_risk_time = now
        elif self.last_risk_time is not None and now - self.last_risk_time <= self.ACTIVE_HOLD_SEC:
            active = True
        else:
            active = False
            self.state = 'IDLE'
            self.last_cmd = self.make_motor_msg(0.0, 0.0)

        self.active_pub.publish(Bool(data=active))
        self.cmd_pub.publish(self.last_cmd)
        self.log_state(now, active)
        self.save_previous_measurements(now)
        self.prev_stop_required = stop_required

    def make_avoid_cmd(self, avoid_angle=None, avoid_speed=None):
        avoid_angle = self.AVOID_ANGLE if avoid_angle is None else avoid_angle
        avoid_speed = self.AVOID_SPEED if avoid_speed is None else avoid_speed

        if self.blue_center_x is None or self.image_width is None:
            return self.make_motor_msg(0.0, avoid_speed)

        if self.blue_center_x < self.image_width / 2:
            angle = avoid_angle
        else:
            angle = -avoid_angle
        self.last_avoid_angle = angle
        return self.make_motor_msg(angle, avoid_speed)

    def make_clearing_cmd(self):
        if self.blue_center_x is not None and self.image_width is not None:
            return self.make_avoid_cmd(self.CLEARING_ANGLE, self.CLEARING_SPEED)
        if self.last_avoid_angle > 0.0:
            angle = self.CLEARING_ANGLE
        elif self.last_avoid_angle < 0.0:
            angle = -self.CLEARING_ANGLE
        else:
            angle = 0.0
        return self.make_motor_msg(angle, self.CLEARING_SPEED)

    def update_approach_estimate(self, now):
        self.approaching = False
        if self.prev_measure_time is None:
            return

        dt = now - self.prev_measure_time
        if dt <= 0.0:
            return

        if math.isfinite(self.front_min_distance) and math.isfinite(self.prev_front_min_distance):
            self.closing_rate = (self.prev_front_min_distance - self.front_min_distance) / dt
        else:
            self.closing_rate = 0.0

        self.area_growth_rate = (self.blue_area_ratio - self.prev_blue_area_ratio) / dt
        self.height_growth_rate = (self.blue_height_ratio - self.prev_blue_height_ratio) / dt

        lidar_approaching = (
            self.front_min_distance < self.AVOID_DISTANCE and
            self.closing_rate >= self.APPROACH_DISTANCE_RATE
        )
        camera_approaching = (
            self.blue_detected and
            (self.area_growth_rate >= self.APPROACH_AREA_RATE or
             self.height_growth_rate >= self.APPROACH_HEIGHT_RATE)
        )
        self.approaching = lidar_approaching or camera_approaching

    def save_previous_measurements(self, now):
        self.prev_measure_time = now
        self.prev_front_min_distance = self.front_min_distance
        self.prev_blue_area_ratio = self.blue_area_ratio
        self.prev_blue_height_ratio = self.blue_height_ratio

    def clear_blue_detection(self):
        self.blue_detected = False
        self.blue_center_x = None
        self.blue_area_ratio = 0.0
        self.blue_height_ratio = 0.0
        self.blue_aspect_ratio = 0.0

    @staticmethod
    def make_motor_msg(angle, speed):
        msg = XycarMotor()
        msg.angle = float(angle)
        msg.speed = float(speed)
        return msg

    def log_state(self, now, active):
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now
        distance = 'inf' if math.isinf(self.front_min_distance) else f'{self.front_min_distance:.2f}'
        self.get_logger().info(
            f'state={self.state} active={active} front_min={distance} '
            f'blue={self.blue_detected} area={self.blue_area_ratio:.3f} '
            f'height={self.blue_height_ratio:.3f} aspect={self.blue_aspect_ratio:.2f} '
            f'roi_top={self.roi_top} '
            f'approaching={self.approaching} close_rate={self.closing_rate:.2f} '
            f'area_rate={self.area_growth_rate:.3f} height_rate={self.height_growth_rate:.3f} '
            f'clearing_left={max(0.0, self.clearing_until - now):.1f} '
            f'angle={self.last_cmd.angle:.1f} speed={self.last_cmd.speed:.1f}')

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = DrunkAvoidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
