#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class TrafficLightNode(Node):
    """Traffic light recognition node using a YOLO model."""

    LABEL_TO_STATE = {
        "start_red": "START_RED",
        "start_yellow": "START_YELLOW",
        "start_green": "START_GREEN",
        "red_only": "RED",
        "green_straight": "GREEN",
        "yellow_only": "YELLOW",
        "red_and_left": "LEFT",
    }

    def __init__(self):
        super().__init__("traffic_light")

        self.bridge = CvBridge()

        model_path = "/home/inseo/xycar_ws/runs/detect/train/weights/best.pt"
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded: {model_path}")

        self.phase = "START"
        self.start_depart_votes = 0
        self.action_votes = []
        self.last_action = "WAIT_START"
        self.stop_hold_until = 0.0

        self.publisher_action = self.create_publisher(String, "/traffic_action", 10)
        self.subscriber_image = self.create_subscription(
            Image,
            "/usb_cam/image_raw/front",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("TrafficLightNode started (YOLO mode)")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        stop_line = self.detect_stop_line(frame)
        light_state = self.detect_traffic_light(frame)
        action = self.decide_action(light_state, stop_line)

        if action in ("WAIT_START", "STOP"):
            self.action_votes.clear()
            self.last_action = action
        else:
            action = self.stabilize_action(action)

        self.publisher_action.publish(String(data=action))
        self.get_logger().info(
            f"[TRAFFIC] phase={self.phase}, light={light_state}, "
            f"stop_line={stop_line}, action={action}"
        )

    def detect_traffic_light(self, frame) -> str:
        results = self.model(frame, verbose=False)[0]

        if results.boxes is None or len(results.boxes) == 0:
            return "UNKNOWN"

        best_idx = int(results.boxes.conf.argmax())
        best_cls = int(results.boxes.cls[best_idx])
        best_conf = float(results.boxes.conf[best_idx])

        if best_conf < 0.5:
            return "UNKNOWN"

        label = self.model.names[best_cls]
        state = self.LABEL_TO_STATE.get(label, "UNKNOWN")

        self.get_logger().info(f"[YOLO] {label} conf={best_conf:.2f} -> {state}")
        return state

    def decide_action(self, light_state: str, stop_line: bool) -> str:
        now = time.monotonic()

        if now < self.stop_hold_until:
            return "STOP"

        if self.phase == "START":
            if light_state == "START_YELLOW":
                self.start_depart_votes = 0
                return "STOP"

            if light_state == "START_GREEN":
                self.start_depart_votes += 1
            else:
                self.start_depart_votes = 0

            if self.start_depart_votes >= 5:
                self.get_logger().info("[TRAFFIC] Departing! Phase -> COURSE")
                self.phase = "COURSE"
                self.action_votes.clear()
                self.last_action = "GO"
                return "GO"

            return "WAIT_START"

        if light_state == "GREEN":
            return "GO"

        if light_state == "LEFT":
            if stop_line:
                return "LEFT"
            return "SIGNAL_APPROACH"

        if light_state == "RED":
            if stop_line:
                self.stop_hold_until = now + 2.0
                return "STOP"
            return "RED_APPROACH"

        if light_state == "YELLOW":
            if stop_line:
                self.stop_hold_until = now + 2.0
                return "STOP"
            return "SIGNAL_APPROACH"

        return "GO"

    def stabilize_action(self, action: str) -> str:
        self.action_votes.append(action)
        if len(self.action_votes) > 3:
            self.action_votes.pop(0)

        values, counts = np.unique(self.action_votes, return_counts=True)
        best = str(values[int(np.argmax(counts))])

        if counts.max() >= 2:
            self.last_action = best
        return self.last_action

    def detect_stop_line(self, frame) -> bool:
        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 190])
        upper_white = np.array([180, 65, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        y1 = int(height * 0.70)
        y2 = int(height * 0.92)
        x1 = int(width * 0.12)
        x2 = int(width * 0.88)
        roi = mask[y1:y2, x1:x2]

        kernel = np.ones((5, 15), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)

            wide_enough = w > width * 0.25
            line_like = w > max(3 * h, 1)
            visible_enough = area > width * height * 0.003
            near_bottom = y > roi.shape[0] * 0.25

            if wide_enough and line_like and visible_enough and near_bottom:
                return True

        return False


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
