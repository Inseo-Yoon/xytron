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
    """
    신호등 인식 노드 (YOLOv8 버전)

    [출발 신호등 3등 - START 페이즈]
      start_red / start_yellow / start_green
      - start_red: 대기 (WAIT_START)
      - start_yellow 또는 start_green: 출발 (GO) → COURSE 페이즈

    [주행 신호등 4등 - COURSE 페이즈]
      red_only / green_straight / yellow_only / red_and_left
      - green_straight: GO
      - red_and_left:   LEFT
      - red_only:       정지선 감지 시 STOP, 아니면 SLOW
      - yellow_only:    정지선 감지 시 STOP, 아니면 SLOW
    """

    # YOLO 클래스 → 내부 상태 매핑
    LABEL_TO_STATE = {
        "start_red":      "START_RED",
        "start_yellow":   "START_YELLOW",
        "start_green":    "START_GREEN",
        "red_only":       "RED",
        "green_straight": "GREEN",
        "yellow_only":    "YELLOW",
        "red_and_left":   "LEFT",
    }

    def __init__(self):
        super().__init__("traffic_light")

        self.bridge = CvBridge()

        # YOLO 모델 로드
        model_path = "/home/xytron/xycar_ws/src/track_drive/best.pt"
        self.model  = YOLO(model_path)
        self.get_logger().info(f"YOLO 모델 로드 완료: {model_path}")

        # 페이즈: "START" → "COURSE"
        self.phase = "START"

        # 출발 신호등: 주황/초록 연속 감지 투표 카운터
        self.start_depart_votes = 0

        # 액션 안정화용 투표 버퍼
        self.action_votes = []
        self.last_action  = "WAIT_START"

        # 정지 유지 타이머
        self.stop_hold_until = 0.0

        self.publisher_action = self.create_publisher(String, "/traffic_action", 10)
        self.subscriber_image = self.create_subscription(
            Image,
            "/usb_cam/image_raw/front",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("TrafficLightNode started (YOLO mode)")

    # ------------------------------------------------------------------ #
    #  메인 콜백
    # ------------------------------------------------------------------ #

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        stop_line   = self.detect_stop_line(frame)
        light_state = self.detect_traffic_light(frame)
        action      = self.decide_action(light_state, stop_line)

        # STOP / WAIT 계열은 즉시 확정, 나머지는 다수결 안정화
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

    # ------------------------------------------------------------------ #
    #  YOLO 신호등 감지
    # ------------------------------------------------------------------ #

    def detect_traffic_light(self, frame) -> str:
        import tempfile, os
        tmp = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        cv2.imwrite(tmp.name, frame)
        tmp.close()
        results = self.model(tmp.name, verbose=False)[0]
        os.unlink(tmp.name)

        if results.boxes is None or len(results.boxes) == 0:
            return "UNKNOWN"

        # confidence 가장 높은 박스 선택
        best_idx  = int(results.boxes.conf.argmax())
        best_cls  = int(results.boxes.cls[best_idx])
        best_conf = float(results.boxes.conf[best_idx])

        if best_conf < 0.5:
            return "UNKNOWN"

        label = self.model.names[best_cls]
        state = self.LABEL_TO_STATE.get(label, "UNKNOWN")

        self.get_logger().info(f"[YOLO] {label} conf={best_conf:.2f} → {state}")
        return state

    # ------------------------------------------------------------------ #
    #  액션 결정
    # ------------------------------------------------------------------ #

    def decide_action(self, light_state: str, stop_line: bool) -> str:
        now = time.monotonic()

        # 정지 유지 중
        if now < self.stop_hold_until:
            return "STOP"

        # ── 출발 페이즈 ─────────────────────────────────────────────────
        if self.phase == "START":
            if light_state in ("START_YELLOW", "START_GREEN"):
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

        # ── 주행 페이즈 ─────────────────────────────────────────────────
        if light_state == "GREEN":
            return "GO"

        if light_state == "LEFT":
            return "LEFT"

        if light_state in ("RED", "YELLOW"):
            if stop_line:
                self.stop_hold_until = now + 2.0
                return "STOP"
            else:
                return "SLOW"

        # UNKNOWN
        return "GO"

    # ------------------------------------------------------------------ #
    #  액션 안정화 (다수결, 윈도우=3)
    # ------------------------------------------------------------------ #

    def stabilize_action(self, action: str) -> str:
        self.action_votes.append(action)
        if len(self.action_votes) > 3:
            self.action_votes.pop(0)

        values, counts = np.unique(self.action_votes, return_counts=True)
        best = str(values[int(np.argmax(counts))])

        if counts.max() >= 2:
            self.last_action = best
        return self.last_action

    # ------------------------------------------------------------------ #
    #  정지선 감지 (HSV 흰색)
    # ------------------------------------------------------------------ #

    def detect_stop_line(self, frame) -> bool:
        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0,   0, 190])
        upper_white = np.array([180, 65, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        y1 = int(height * 0.70)
        y2 = int(height * 0.92)
        x1 = int(width  * 0.12)
        x2 = int(width  * 0.88)
        roi = mask[y1:y2, x1:x2]

        kernel = np.ones((5, 15), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)

            wide_enough    = w > width * 0.25
            line_like      = w > max(3 * h, 1)
            visible_enough = area > width * height * 0.003
            near_bottom    = y > roi.shape[0] * 0.25

            if wide_enough and line_like and visible_enough and near_bottom:
                return True

        return False


# ---------------------------------------------------------------------- #

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
