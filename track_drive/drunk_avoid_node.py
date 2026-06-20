#!/usr/bin/env python3
import math
import os
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from xycar_msgs.msg import XycarMotor

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class DrunkAvoidNode(Node):
    FRONT_HALF_ANGLE = math.radians(20.0)
    STOP_DISTANCE = 0.55
    SLOW_DISTANCE = 1.2

    YOLO_MODEL_NAME = 'yolov8n.pt'
    YOLO_CONFIDENCE = 0.45
    YOLO_INFERENCE_INTERVAL = 3
    SHOW_DEBUG_WINDOW = True
    DEBUG_WINDOW_NAME = 'Drunk Avoid - YOLO Debug'
    DEBUG_DISPLAY_WIDTH = 960

    HSV_LOWER_BLUE = np.array([90, 50, 30], dtype=np.uint8)
    HSV_UPPER_BLUE = np.array([135, 255, 255], dtype=np.uint8)
    HSV_ROI_TOP_RATIO = 0.45
    MIN_BLOB_AREA = 1200.0

    DANGER_ZONE_LEFT = 0.05
    DANGER_ZONE_RIGHT = 0.95
    DANGER_ZONE_TOP = 0.45
    DANGER_ZONE_BOTTOM = 0.90
    CENTER_HISTORY_SIZE = 5
    DIRECTION_THRESHOLD = 0.01

    MIN_STOP_TIME = 1.0
    ESCAPE_DURATION = 1.2
    RECOVERY_DURATION = 0.7
    SLOW_SPEED = 3.0
    ESCAPE_SPEED = 7.0
    RECOVERY_SPEED = 4.0

    IDLE = 'IDLE'
    PEDESTRIAN_DETECTED = 'PEDESTRIAN_DETECTED'
    STOPPING = 'STOPPING'
    WAITING_CLEAR = 'WAITING_CLEAR'
    ESCAPING = 'ESCAPING'
    RECOVERING = 'RECOVERING'

    def __init__(self):
        super().__init__('drunk_avoid_node')

        self.bridge = CvBridge()
        self.image_width = None
        self.image_height = None
        self.frame_count = 0

        self.yolo_model = None
        self.yolo_enabled = False
        self.person_detected = False
        self.person_confidence = 0.0
        self.person_center_x = None
        self.person_bbox = None

        self.blue_detected = False
        self.blue_center_x = None
        self.blue_area_ratio = 0.0
        self.blue_bbox = None

        self.candidate_detected = False
        self.candidate_center_x = None
        self.center_history = deque(maxlen=self.CENTER_HISTORY_SIZE)
        self.pedestrian_direction = 'UNKNOWN'

        self.front_min_distance = math.inf
        self.front_distance_valid = False

        self.state = self.IDLE
        self.state_started_at = self.now_seconds()
        self.stop_started_at = None
        self.last_log_time = 0.0
        self.last_cmd = self.make_motor_msg(0.0, 0.0)
        self.debug_window_enabled = self.SHOW_DEBUG_WINDOW and bool(
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self.debug_window_initialized = False

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
        self.create_timer(0.1, self.control_loop)

        self.initialize_yolo()
        mode = 'YOLO + HSV' if self.yolo_enabled else 'HSV fallback'
        self.get_logger().info(f'Drunk Avoid Node Initialized ({mode})')
        if not self.debug_window_enabled:
            self.get_logger().warn(
                'Debug window disabled because DISPLAY/WAYLAND_DISPLAY is unavailable.')

    def initialize_yolo(self):
        if YOLO is None:
            self.get_logger().warn(
                'Ultralytics YOLO is unavailable; using HSV fallback.')
            return

        try:
            self.yolo_model = YOLO(self.YOLO_MODEL_NAME)
            self.yolo_enabled = True
        except Exception as exc:
            self.yolo_model = None
            self.get_logger().warn(
                f'Failed to load {self.YOLO_MODEL_NAME}: {exc}; using HSV fallback.')

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert camera image: {exc}')
            return

        self.image_height, self.image_width = image.shape[:2]
        self.frame_count += 1
        self.detect_blue_blob(image)

        if self.yolo_enabled and self.frame_count % self.YOLO_INFERENCE_INTERVAL == 0:
            self.detect_person(image)

        self.update_candidate()
        self.show_debug_image(image)

    def detect_person(self, image):
        try:
            results = self.yolo_model.predict(
                source=image,
                conf=self.YOLO_CONFIDENCE,
                classes=[0],
                verbose=False)
        except Exception as exc:
            self.get_logger().warn(
                f'YOLO inference failed: {exc}; disabling YOLO and using HSV fallback.')
            self.yolo_enabled = False
            self.yolo_model = None
            self.clear_person_detection()
            return

        largest_person = None
        largest_area = 0.0
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                if class_id != 0 or confidence < self.YOLO_CONFIDENCE:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
                area = (x2 - x1) * (y2 - y1)
                if area > largest_area:
                    largest_area = area
                    largest_person = (x1, y1, x2, y2, confidence)

        if largest_person is None:
            self.clear_person_detection()
            return

        x1, y1, x2, y2, confidence = largest_person
        self.person_detected = True
        self.person_confidence = confidence
        self.person_center_x = ((x1 + x2) * 0.5) / max(self.image_width, 1)
        self.person_bbox = tuple(map(int, (x1, y1, x2, y2)))

    def detect_blue_blob(self, image):
        image_height, image_width = image.shape[:2]
        roi_top = int(image_height * self.HSV_ROI_TOP_RATIO)
        roi = image[roi_top:, :]
        roi_area = max(roi.shape[0] * roi.shape[1], 1)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.HSV_LOWER_BLUE, self.HSV_UPPER_BLUE)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.clear_blue_detection()
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        moments = cv2.moments(largest)
        if area < self.MIN_BLOB_AREA or moments['m00'] == 0:
            self.clear_blue_detection()
            return

        center_x_pixels = moments['m10'] / moments['m00']
        x, y, width, height = cv2.boundingRect(largest)
        self.blue_detected = True
        self.blue_center_x = center_x_pixels / max(image_width, 1)
        self.blue_area_ratio = area / roi_area
        self.blue_bbox = (x, y + roi_top, x + width, y + roi_top + height)

    def update_candidate(self):
        self.candidate_detected = self.person_detected or self.blue_detected
        if self.person_detected:
            self.candidate_center_x = self.person_center_x
        elif self.blue_detected:
            self.candidate_center_x = self.blue_center_x
        else:
            self.candidate_center_x = None
            self.center_history.clear()
            self.pedestrian_direction = 'UNKNOWN'
            return

        if self.candidate_center_x is not None:
            self.center_history.append(self.candidate_center_x)
        self.update_direction()

    def update_direction(self):
        if len(self.center_history) < 2:
            self.pedestrian_direction = 'UNKNOWN'
            return

        deltas = [
            current - previous
            for previous, current in zip(
                self.center_history, list(self.center_history)[1:])
        ]
        average_delta = sum(deltas) / len(deltas)
        if average_delta > self.DIRECTION_THRESHOLD:
            self.pedestrian_direction = 'LEFT_TO_RIGHT'
        elif average_delta < -self.DIRECTION_THRESHOLD:
            self.pedestrian_direction = 'RIGHT_TO_LEFT'
        else:
            self.pedestrian_direction = 'UNKNOWN'

    def scan_callback(self, msg):
        valid_front_ranges = []
        for index, distance in enumerate(msg.ranges):
            angle = self.normalize_angle(
                msg.angle_min + index * msg.angle_increment)
            if not -self.FRONT_HALF_ANGLE <= angle <= self.FRONT_HALF_ANGLE:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            valid_front_ranges.append(distance)

        self.front_distance_valid = bool(valid_front_ranges)
        self.front_min_distance = (
            min(valid_front_ranges) if valid_front_ranges else math.inf)

    def control_loop(self):
        now = self.now_seconds()
        emergency_stop = (
            self.front_distance_valid and
            self.front_min_distance < self.STOP_DISTANCE)
        candidate_close = (
            self.candidate_detected and
            self.front_distance_valid and
            self.front_min_distance < self.SLOW_DISTANCE)
        candidate_in_danger_zone = self.is_candidate_in_danger_zone()

        # Emergency distance always wins, including during ESCAPING.
        if emergency_stop:
            self.enter_stopping(now)
        elif self.state == self.ESCAPING and candidate_in_danger_zone:
            self.enter_stopping(now)
        elif self.state == self.IDLE:
            if candidate_close:
                self.set_state(self.PEDESTRIAN_DETECTED, now)
        elif self.state == self.PEDESTRIAN_DETECTED:
            if candidate_close:
                self.enter_stopping(now)
            else:
                self.set_state(self.RECOVERING, now)
        elif self.state == self.STOPPING:
            self.set_state(self.WAITING_CLEAR, now)
        elif self.state == self.WAITING_CLEAR:
            stopped_long_enough = (
                self.stop_started_at is not None and
                now - self.stop_started_at >= self.MIN_STOP_TIME)
            lidar_clear = (
                self.front_distance_valid and
                self.front_min_distance > self.SLOW_DISTANCE)
            camera_clear = (
                not self.candidate_detected or
                not candidate_in_danger_zone)
            if stopped_long_enough and lidar_clear and camera_clear:
                self.set_state(self.ESCAPING, now)
        elif self.state == self.ESCAPING:
            if now - self.state_started_at >= self.ESCAPE_DURATION:
                self.set_state(self.RECOVERING, now)
        elif self.state == self.RECOVERING:
            if candidate_close:
                self.set_state(self.PEDESTRIAN_DETECTED, now)
            elif now - self.state_started_at >= self.RECOVERY_DURATION:
                self.set_state(self.IDLE, now)

        active, command = self.command_for_state()
        self.last_cmd = command
        self.active_pub.publish(Bool(data=active))
        self.cmd_pub.publish(command)
        self.log_state(now, active)

    def enter_stopping(self, now):
        if self.state not in (self.STOPPING, self.WAITING_CLEAR):
            self.stop_started_at = now
        elif self.stop_started_at is None:
            self.stop_started_at = now
        self.set_state(self.STOPPING, now)

    def set_state(self, new_state, now):
        if self.state == new_state:
            return
        self.state = new_state
        self.state_started_at = now

    def command_for_state(self):
        if self.state == self.IDLE:
            return False, self.make_motor_msg(0.0, 0.0)
        if self.state == self.PEDESTRIAN_DETECTED:
            return True, self.make_motor_msg(0.0, self.SLOW_SPEED)
        if self.state in (self.STOPPING, self.WAITING_CLEAR):
            return True, self.make_motor_msg(0.0, 0.0)
        if self.state == self.ESCAPING:
            return True, self.make_motor_msg(0.0, self.ESCAPE_SPEED)
        return True, self.make_motor_msg(0.0, self.RECOVERY_SPEED)

    def is_candidate_in_danger_zone(self):
        person_in_danger = (
            self.person_detected and
            self.bbox_anchor_in_danger_zone(self.person_bbox))
        blue_in_danger = (
            self.blue_detected and
            self.bbox_anchor_in_danger_zone(self.blue_bbox))
        return person_in_danger or blue_in_danger

    def bbox_anchor_in_danger_zone(self, bbox):
        if bbox is None or not self.image_width or not self.image_height:
            return False

        x1, _, x2, y2 = bbox
        center_x = ((x1 + x2) * 0.5) / self.image_width
        bottom_y = y2 / self.image_height
        return (
            self.DANGER_ZONE_LEFT <= center_x <= self.DANGER_ZONE_RIGHT and
            self.DANGER_ZONE_TOP <= bottom_y <= self.DANGER_ZONE_BOTTOM)

    def show_debug_image(self, image):
        if not self.debug_window_enabled:
            return

        debug_image = image.copy()
        image_height, image_width = debug_image.shape[:2]
        danger_left = int(image_width * self.DANGER_ZONE_LEFT)
        danger_right = int(image_width * self.DANGER_ZONE_RIGHT)
        danger_top = int(image_height * self.DANGER_ZONE_TOP)
        danger_bottom = int(image_height * self.DANGER_ZONE_BOTTOM)

        overlay = debug_image.copy()
        cv2.rectangle(
            overlay, (danger_left, danger_top), (danger_right, danger_bottom),
            (0, 0, 255), -1)
        debug_image = cv2.addWeighted(overlay, 0.12, debug_image, 0.88, 0.0)
        cv2.rectangle(
            debug_image, (danger_left, danger_top),
            (danger_right, danger_bottom), (0, 0, 255), 2)

        if self.person_bbox is not None and self.person_detected:
            x1, y1, x2, y2 = self.person_bbox
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(
                debug_image, f'person {self.person_confidence:.2f}',
                (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 0), 2, cv2.LINE_AA)

        if self.blue_bbox is not None and self.blue_detected:
            x1, y1, x2, y2 = self.blue_bbox
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(
                debug_image, 'blue HSV', (x1, min(image_height - 8, y2 + 24)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0),
                2, cv2.LINE_AA)

        distance = (
            f'{self.front_min_distance:.2f} m'
            if self.front_distance_valid else 'invalid')
        status_lines = [
            f'STATE: {self.state}',
            f'YOLO: {self.person_detected}  HSV: {self.blue_detected}',
            f'FRONT: {distance}  DIR: {self.pedestrian_direction}',
        ]
        for index, text in enumerate(status_lines):
            y = 30 + index * 30
            cv2.putText(
                debug_image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (20, 20, 20), 4, cv2.LINE_AA)
            cv2.putText(
                debug_image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 2, cv2.LINE_AA)

        if image_width > self.DEBUG_DISPLAY_WIDTH:
            scale = self.DEBUG_DISPLAY_WIDTH / image_width
            debug_image = cv2.resize(
                debug_image, None, fx=scale, fy=scale,
                interpolation=cv2.INTER_AREA)

        try:
            if not self.debug_window_initialized:
                cv2.namedWindow(self.DEBUG_WINDOW_NAME, cv2.WINDOW_NORMAL)
                self.debug_window_initialized = True
            cv2.imshow(self.DEBUG_WINDOW_NAME, debug_image)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.debug_window_enabled = False
            self.get_logger().warn(f'Disabling debug window: {exc}')

    def clear_person_detection(self):
        self.person_detected = False
        self.person_confidence = 0.0
        self.person_center_x = None
        self.person_bbox = None

    def clear_blue_detection(self):
        self.blue_detected = False
        self.blue_center_x = None
        self.blue_area_ratio = 0.0
        self.blue_bbox = None

    def log_state(self, now, active):
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now
        distance = (
            f'{self.front_min_distance:.2f}'
            if self.front_distance_valid else 'invalid')
        center = (
            f'{self.candidate_center_x:.3f}'
            if self.candidate_center_x is not None else 'none')
        self.get_logger().info(
            f'state={self.state} yolo={self.person_detected} '
            f'conf={self.person_confidence:.2f} blue={self.blue_detected} '
            f'center_x={center} direction={self.pedestrian_direction} '
            f'front_min={distance} active={active} '
            f'angle={self.last_cmd.angle:.1f} speed={self.last_cmd.speed:.1f}')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def make_motor_msg(angle, speed):
        msg = XycarMotor()
        msg.angle = float(angle)
        msg.speed = float(speed)
        return msg

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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
