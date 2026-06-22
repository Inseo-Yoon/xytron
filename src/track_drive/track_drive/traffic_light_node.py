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

        model_path = "/home/wlwnstn2396/xycar_ws/src/track_drive/models/best.pt"
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded: {model_path}")

        self.phase = "START"
        self.start_depart_votes = 0
        self.action_votes = []
        self.last_action = "WAIT_START"
        self.stop_hold_until = 0.0
        self.pending_stop = False
        self.holding_for_signal = False

        self.publisher_action = self.create_publisher(String, "/traffic_action", 10)
        self.subscriber_image = self.create_subscription(
            Image,
            "/usb_cam/image_raw/front",
            self.image_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        stop_line = self.detect_stop_line(frame)
        light_state = self.detect_traffic_light(frame)
        action = self.decide_action(light_state, stop_line)

        # 안정화(Stabilization) 적용
        if action in ("WAIT_START", "STOP"):
            self.action_votes.clear()
            self.last_action = action
        else:
            action = self.stabilize_action(action)

        self.publisher_action.publish(String(data=action))

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
        return self.LABEL_TO_STATE.get(label, "UNKNOWN")

    def detect_stop_line(self, frame) -> bool:
        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 정지선 인식용 HSV 범위
        lower_white = np.array([0, 0, 190])
        upper_white = np.array([180, 65, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        y1, y2 = int(height * 0.70), int(height * 0.92)
        x1, x2 = int(width * 0.12), int(width * 0.88)
        roi = mask[y1:y2, x1:x2]

        kernel = np.ones((5, 15), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if w > width * 0.25 and area > width * height * 0.003:
                return True
        return False

    def decide_action(self, light_state: str, stop_line: bool) -> str:
        now = time.monotonic()
        if now < self.stop_hold_until:
            return "STOP"

        if self.holding_for_signal:
            if light_state in ("GREEN", "LEFT", "START_GREEN"):
                self.holding_for_signal = False
                self.pending_stop = False
                return "GO" if light_state != "LEFT" else "LEFT"
            return "STOP"

        if self.phase == "START":
            if light_state == "START_GREEN":
                self.start_depart_votes += 1
            else:
                self.start_depart_votes = 0

            if self.start_depart_votes >= 5:
                self.phase = "COURSE"
                self.pending_stop = False
                self.holding_for_signal = False
                return "GO"
            return "WAIT_START"

        # 주행 단계 로직
        if light_state == "GREEN":
            self.pending_stop = False
            return "GO"

        if light_state == "LEFT":
            self.pending_stop = False
            return "LEFT" if stop_line else "SIGNAL_APPROACH"

        if light_state in ("RED", "YELLOW"):
            self.pending_stop = True

        if self.pending_stop:
            if stop_line:
                self.stop_hold_until = now + 2.0
                self.pending_stop = False
                self.holding_for_signal = True
                return "STOP"
            return "YELLOW_APPROACH" if light_state == "YELLOW" else "RED_APPROACH"

        return "GO"

    def stabilize_action(self, action: str) -> str:
        self.action_votes.append(action)
        if len(self.action_votes) > 3: self.action_votes.pop(0)
        values, counts = np.unique(self.action_votes, return_counts=True)
        best = str(values[int(np.argmax(counts))])
        if counts.max() >= 2: self.last_action = best
        return self.last_action

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
