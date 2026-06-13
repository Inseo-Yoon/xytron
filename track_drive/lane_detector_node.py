#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__("lane_detector")

        self.bridge = CvBridge()
        self.prev_angle = 0.0

        self.angle_scale = 70.0
        self.alpha = 0.7
        self.base_speed = 5.0
        self.debug_view = False

        self.publisher_angle = self.create_publisher(Float32, "/lane_angle", 10)
        self.publisher_departure = self.create_publisher(Bool, "/lane_departure", 10)
        self.subscriber_image = self.create_subscription(
            Image,
            "/usb_cam/image_raw/front",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("LaneDetectorNode started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        target_points, visible_lane_count, debug_image = self.process_lane_image(frame)
        steering_error = self.calculate_steering_error(target_points, frame.shape[1])

        angle = steering_error * self.angle_scale
        angle = self.alpha * angle + (1.0 - self.alpha) * self.prev_angle
        angle = float(np.clip(angle, -50.0, 50.0))
        self.prev_angle = angle

        lane_departure = visible_lane_count == 0 or not target_points

        self.publisher_angle.publish(Float32(data=angle))
        self.publisher_departure.publish(Bool(data=lane_departure))

        self.get_logger().info(
            f"[LANE] angle: {angle:.2f}, error: {steering_error:.2f}, "
            f"visible_lanes: {visible_lane_count}, departure: {lane_departure}"
        )

        if self.debug_view and debug_image is not None:
            cv2.imshow("Masked Lane Image", debug_image)
            cv2.waitKey(1)

    def process_lane_image(self, image):
        if image is None:
            return [], 0, None

        height, width = image.shape[:2]

        roi_points = np.array(
            [
                [
                    (int(width * 0.02), height),
                    (int(width * 0.35), int(height * 0.52)),
                    (int(width * 0.65), int(height * 0.52)),
                    (int(width * 0.98), height),
                ]
            ],
            dtype=np.int32,
        )

        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, roi_points, 255)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([179, 60, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        yellow_lane_mask = cv2.bitwise_and(yellow_mask, roi_mask)
        white_lane_mask = cv2.bitwise_and(white_mask, roi_mask)

        yellow_kernel = np.ones((5, 15), np.uint8)
        yellow_lane_mask = cv2.morphologyEx(
            yellow_lane_mask,
            cv2.MORPH_CLOSE,
            yellow_kernel,
        )

        left_white_mask, right_white_mask = self.split_white_components_by_yellow(
            white_lane_mask,
            yellow_lane_mask,
            min_area=80,
        )

        yellow_points = self.get_center_points(
            yellow_lane_mask,
            min_pixels=5,
            step=20,
            max_x_jump=160,
        )
        left_white_points = self.get_center_points(
            left_white_mask,
            min_pixels=20,
            step=20,
            max_x_jump=80,
        )
        right_white_points = self.get_center_points(
            right_white_mask,
            min_pixels=20,
            step=20,
            max_x_jump=80,
        )

        lane_half_width = int(width * 0.18)
        target_points = self.make_driving_target_points(
            yellow_points,
            left_white_points,
            right_white_points,
            lane_half_width,
        )

        yellow_visible = len(yellow_points) >= 3
        left_white_visible = len(left_white_points) >= 3
        right_white_visible = len(right_white_points) >= 3
        visible_lane_count = sum(
            [yellow_visible, left_white_visible, right_white_visible]
        )

        debug_image = self.make_debug_image(
            image,
            yellow_lane_mask,
            left_white_mask,
            right_white_mask,
            yellow_points,
            left_white_points,
            right_white_points,
            target_points,
            visible_lane_count,
        )

        return target_points, visible_lane_count, debug_image

    def calculate_steering_error(self, target_points, width):
        if not target_points:
            return 0.0

        image_center = width / 2.0
        points = sorted(target_points, key=lambda point: point[1], reverse=True)
        selected_points = points[: min(5, len(points))]

        weights = np.linspace(2.0, 1.0, len(selected_points))
        target_xs = [point[0] for point in selected_points]
        target_x = float(np.average(target_xs, weights=weights))

        error = (target_x - image_center) / image_center
        return float(np.clip(error, -1.0, 1.0))

    def make_driving_target_points(
        self,
        yellow_points,
        left_white_points,
        right_white_points,
        lane_half_width,
    ):
        if len(yellow_points) >= 3 and len(right_white_points) >= 3:
            return self.make_target_points(yellow_points, right_white_points)

        if len(left_white_points) >= 3 and len(yellow_points) >= 3:
            return self.make_target_points(left_white_points, yellow_points)

        if len(left_white_points) >= 3 and len(right_white_points) >= 3:
            return self.make_target_points(left_white_points, right_white_points)

        if len(right_white_points) >= 3:
            return self.shift_points(right_white_points, -lane_half_width)

        if len(yellow_points) >= 3:
            return self.shift_points(yellow_points, lane_half_width)

        if len(left_white_points) >= 3:
            return self.shift_points(left_white_points, lane_half_width)

        return []

    def get_center_points(self, mask, min_pixels=20, step=20, max_x_jump=80):
        height, _ = mask.shape[:2]
        center_points = []
        last_x = None

        for y in range(height - 1, 0, -step):
            y_start = max(y - step, 0)
            y_end = y

            band = mask[y_start:y_end, :]
            _, xs = np.where(band > 0)

            if len(xs) <= min_pixels:
                continue

            center_x = int(np.mean(xs))
            center_y = int((y_start + y_end) / 2)

            if last_x is not None and abs(center_x - last_x) > max_x_jump:
                continue

            center_points.append((center_x, center_y))
            last_x = center_x

        return center_points

    def draw_points_and_lines(self, image, points, color):
        for point in points:
            cv2.circle(image, point, 4, color, -1)

        for index in range(len(points) - 1):
            cv2.line(image, points[index], points[index + 1], color, 3)

    def points_to_dict(self, points):
        return {y: x for x, y in points}

    def make_target_points(self, left_points, right_points):
        left_dict = self.points_to_dict(left_points)
        right_dict = self.points_to_dict(right_points)

        target_points = []
        common_ys = sorted(set(left_dict.keys()) & set(right_dict.keys()), reverse=True)

        for y in common_ys:
            center_x = int((left_dict[y] + right_dict[y]) / 2)
            target_points.append((center_x, y))

        return target_points

    def shift_points(self, points, x_offset):
        return [(int(x + x_offset), y) for x, y in points]

    def split_white_components_by_yellow(
        self,
        white_lane_mask,
        yellow_lane_mask,
        min_area=80,
    ):
        height, width = white_lane_mask.shape[:2]

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            white_lane_mask,
            connectivity=8,
        )

        _, yellow_xs = np.where(yellow_lane_mask > 0)

        if len(yellow_xs) > 10:
            reference_x = int(np.mean(yellow_xs))
        else:
            reference_x = width // 2

        left_white_mask = np.zeros((height, width), dtype=np.uint8)
        right_white_mask = np.zeros((height, width), dtype=np.uint8)

        for label in range(1, num_labels):
            area = stats[label, cv2.CC_STAT_AREA]

            if area < min_area:
                continue

            cx = centroids[label][0]
            component_mask = np.zeros((height, width), dtype=np.uint8)
            component_mask[labels == label] = 255

            if cx < reference_x:
                left_white_mask = cv2.bitwise_or(left_white_mask, component_mask)
            else:
                right_white_mask = cv2.bitwise_or(right_white_mask, component_mask)

        return left_white_mask, right_white_mask

    def split_white_by_yellow(self, white_lane_mask, yellow_lane_mask, step=20):
        height, width = white_lane_mask.shape[:2]

        left_white_mask = np.zeros((height, width), dtype=np.uint8)
        right_white_mask = np.zeros((height, width), dtype=np.uint8)
        last_reference_x = width // 2

        for y in range(height - 1, 0, -step):
            y_start = max(y - step, 0)
            y_end = y

            yellow_band = yellow_lane_mask[y_start:y_end, :]
            white_band = white_lane_mask[y_start:y_end, :]

            _, yellow_xs = np.where(yellow_band > 0)
            white_ys, white_xs = np.where(white_band > 0)

            if len(white_xs) == 0:
                continue

            if len(yellow_xs) > 5:
                reference_x = int(np.mean(yellow_xs))
                last_reference_x = reference_x
            else:
                reference_x = last_reference_x

            for wy, wx in zip(white_ys, white_xs):
                real_y = y_start + wy

                if wx < reference_x:
                    left_white_mask[real_y, wx] = 255
                else:
                    right_white_mask[real_y, wx] = 255

        return left_white_mask, right_white_mask

    def make_debug_image(
        self,
        image,
        yellow_lane_mask,
        left_white_mask,
        right_white_mask,
        yellow_points,
        left_white_points,
        right_white_points,
        target_points,
        visible_lane_count,
    ):
        masked_image = np.zeros_like(image)
        masked_image[yellow_lane_mask > 0] = (0, 255, 255)
        masked_image[left_white_mask > 0] = (255, 255, 255)
        masked_image[right_white_mask > 0] = (255, 255, 255)

        self.draw_points_and_lines(masked_image, yellow_points, (0, 0, 255))
        self.draw_points_and_lines(masked_image, left_white_points, (255, 0, 0))
        self.draw_points_and_lines(masked_image, right_white_points, (0, 255, 0))
        self.draw_points_and_lines(masked_image, target_points, (255, 0, 255))

        cv2.putText(
            masked_image,
            f"lanes: {visible_lane_count}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )

        return masked_image


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
