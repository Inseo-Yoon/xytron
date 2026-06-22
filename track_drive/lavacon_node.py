#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class LavaconNode(Node):

    def __init__(self):
        super().__init__("lavacon_node")

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.angle_pub = self.create_publisher(
            Float32,
            "/lavacon_angle",
            10
        )

        self.prev_angle = 0.0
        self.alpha = 0.6  # 조향 필터링 계수

        self.get_logger().info("Lavacon Node Started (Xycar 90-deg Front Fix)")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        # [변경 포인트 1] 90도 정면 기준 각도 슬라이싱
        # 정면(90도) 기준 좌측(105~155도), 우측(25~75도) 범위 지정
        left_cones = self.get_cone_centers(ranges, start_idx=105, end_idx=155)
        right_cones = self.get_cone_centers(ranges, start_idx=25, end_idx=75)

        if not left_cones or not right_cones:
            # 한쪽이라도 유실되면 직전 조향 안전 유지
            self.publish_angle(self.prev_angle)
            return

        # 최적의 전방 중앙 타겟 검색
        target_x, target_y = self.find_best_target(left_cones, right_cones)

        if target_x is None or target_x <= 0.1:
            self.publish_angle(self.prev_angle)
            return

        # Pure Pursuit 조향각 계산
        # target_x가 전방 거리(양수), target_y가 좌우 편차
        lookahead_dist = np.hypot(target_x, target_y)
        
        # 차량 좌표계 기준 사잇각 계산
        raw_angle = math.degrees(math.atan2(target_y, target_x))

        # 조향 게인 조절 (가까우면 기민하게, 멀면 부드럽게)
        steering_gain = 1.2 / max(lookahead_dist, 1.0)
        angle = raw_angle * steering_gain

        # 자이카 조향각 범위 제한 (-50 ~ 50)
        angle = np.clip(angle, -50.0, 50.0)

        # 직전 조향 매칭 (Smoothing 버그 수정 완료본)
        angle = (self.alpha * angle) + ((1.0 - self.alpha) * self.prev_angle)
        self.prev_angle = angle

        self.publish_angle(angle)
        self.get_logger().info(
            f"L={len(left_cones)} R={len(right_cones)} "
            f"Target=({target_x:.2f}, {target_y:.2f}) Angle={angle:.2f}"
        )

    def find_best_target(self, left_cones, right_cones):
        """ 좌우 라바콘들의 쌍 중 차량 전방 중앙에 가장 잘 위치한 점을 반환 """
        center_points = []
        for lc in left_cones:
            for rc in right_cones:
                cx = (lc[0] + rc[0]) / 2.0
                cy = (lc[1] + rc[1]) / 2.0
                dist = np.hypot(cx, cy)
                
                # 차량 전방 0.5m ~ 3.5m 사이의 유효 타겟 필터링
                if 0.5 <= dist <= 3.5:
                    center_points.append((cx, cy, dist))

        if not center_points:
            # 유효 범위에 없으면 가장 가까운 쌍의 중점 선택
            return (left_cones[0][0] + right_cones[0][0]) / 2.0, (left_cones[0][1] + right_cones[0][1]) / 2.0

        # 중간 거리에 있는 안정적인 타겟 선택
        center_points.sort(key=lambda p: p[2])
        mid_idx = len(center_points) // 2
        return center_points[mid_idx][0], center_points[mid_idx][1]

    def get_cone_centers(self, ranges, start_idx, end_idx):
        points = []

        for deg in range(start_idx, end_idx):
            if deg >= len(ranges):
                break
            dist = ranges[deg]

            if not np.isfinite(dist) or dist < 0.1 or dist > 5.0:
                continue

            # [변경 포인트 2] 90도가 정면(X축)인 차량 좌표계 변환
            # 일반 변환: x = dist * cos(rad), y = dist * sin(rad) 이지만,
            # 90도가 정면이므로 정면 기준 오차각(deg - 90)으로 변환해야 차량 중심 좌표계가 됩니다.
            rad = math.radians(deg - 90)
            x = dist * math.cos(rad)  # 정면 방향 거리 (+X)
            y = dist * math.sin(rad)  # 좌측(+Y), 우측(-Y) 방향 거리

            points.append((x, y))

        if len(points) < 3:
            return []

        points = np.array(points)
        clusters = []
        current = [points[0]]

        for i in range(1, len(points)):
            gap = np.linalg.norm(points[i] - points[i - 1])
            if gap < 0.35:  # 시뮬레이터 스케일에 맞춘 군집 임계값
                current.append(points[i])
            else:
                if len(current) >= 3:
                    clusters.append(current)
                current = [points[i]]
        if len(current) >= 3:
            clusters.append(current)

        centers = []
        for cluster in clusters:
            center = np.mean(cluster, axis=0)
            centers.append((float(center[0]), float(center[1])))

        # 차량(원점)에서 가까운 순 정렬
        centers.sort(key=lambda p: np.hypot(p[0], p[1]))
        return centers

    def publish_angle(self, angle):
        self.angle_pub.publish(Float32(data=float(angle)))

def main(args=None):
    rclpy.init(args=args)
    node = LavaconNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()