import cv2
import numpy as np


class TrafficCone:

    def __init__(self):

        self.prev_error = 0.0
        self.prev_angle = 0.0

        self.KP = 1.8
        self.KD = 0.4

        self.MAX_ANGLE = 70
        self.MIN_ANGLE = 3.0

        self.BASE_SPEED = 5
        self.steer_alpha = 0.7
        self.turn_hold = 0
        # 라이다 파라미터
        self.CONE_MIN_DIST = 0.3   # 최소 감지 거리 (m)
        self.CONE_MAX_DIST = 5.0   # 최대 감지 거리 줄임 (8→5m, 먼 노이즈 제거)
        self.CLUSTER_ANGLE = 8     # 클러스터 묶음 각도 범위 (deg)
        self.SCAN_HALF = 60        # 전방 스캔 범위 ±60도로 줄임 (±90→±60)

        # 카메라 파라미터 (카메라는 좌/우 구분 보조만, 양쪽 다 잡힐 때만 사용)
        self.CAM_WEIGHT = 0.0      # 일단 카메라 비중 끔 (라이다만 사용)

        cv2.namedWindow("BEV", cv2.WINDOW_NORMAL)
        cv2.namedWindow("MASK", cv2.WINDOW_NORMAL)
        cv2.namedWindow("LIDAR", cv2.WINDOW_NORMAL)

# =====================================================
# LiDAR 콘 클러스터 감지
# 반환: [(angle_deg, dist), ...] 가까운 클러스터 중심들
# angle: -180~180, 음수=오른쪽, 양수=왼쪽
# =====================================================

    def detect_lidar_cones(self, lidar_ranges):

        # array('f', ...) 또는 list 모두 처리
        ranges = np.frombuffer(lidar_ranges, dtype=np.float32) if hasattr(lidar_ranges, 'buffer_info') else np.array(list(lidar_ranges), dtype=np.float32)
        n = len(ranges)  # 360

        # inf/nan 제거
        ranges = np.where(np.isfinite(ranges), ranges, 999.0)

        # 전방 ±SCAN_HALF도 인덱스 추출
        # 인덱스 0=정면, 반시계: 1~180=왼쪽, 181~359=오른쪽(-179~-1)
        indices = []
        for i in range(self.SCAN_HALF + 1):
            indices.append(i)           # 왼쪽 0~90
        for i in range(n - self.SCAN_HALF, n):
            indices.append(i)           # 오른쪽 270~359

        # 유효 포인트 수집 (거리 필터)
        points = []
        for idx in indices:
            d = ranges[idx]
            if self.CONE_MIN_DIST < d < self.CONE_MAX_DIST:
                # 각도 변환: 반시계 인덱스 → signed angle
                # 0=0도, 1~180=+1~+180, 181~359=-179~-1
                if idx <= 180:
                    angle = float(idx)
                else:
                    angle = float(idx - 360)
                points.append((angle, d))

        if not points:
            return []

        # 단순 클러스터링: 각도 기준 그룹화
        points.sort(key=lambda p: p[0])
        clusters = []
        current = [points[0]]

        for p in points[1:]:
            if abs(p[0] - current[-1][0]) <= self.CLUSTER_ANGLE:
                current.append(p)
            else:
                clusters.append(current)
                current = [p]
        clusters.append(current)

        # 각 클러스터의 중심 각도와 최솟값 거리
        cone_list = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            angles = [p[0] for p in cluster]
            dists  = [p[1] for p in cluster]
            center_angle = np.mean(angles)
            min_dist     = np.min(dists)
            cone_list.append((center_angle, min_dist))

        return cone_list  # [(angle, dist), ...]

# =====================================================
# Bird Eye View
# =====================================================

    def bird_eye_view(self, image):

        h, w = image.shape[:2]

        src = np.float32([
            [w * 0.05, h * 0.95],
            [w * 0.95, h * 0.95],
            [w * 0.80, h * 0.50],
            [w * 0.20, h * 0.50]
        ])

        dst = np.float32([
            [0, h],
            [w, h],
            [w, 0],
            [0, 0]
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(image, M, (w, h))

# =====================================================
# 카메라 콘 감지 (보조)
# 반환: left_cones, right_cones (픽셀 좌표 리스트)
# =====================================================

    def detect_camera_cones(self, bev):

        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)

        lower = np.array([5,  100, 150])
        upper = np.array([20, 200, 255])

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((15, 15), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h, w = bev.shape[:2]
        center_x = w // 2
        left_cones  = []
        right_cones = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            if ch < 10:
                continue
            cx = x + cw // 2
            cy = y + ch // 2
            if cy < h * 0.3:
                continue
            if cx < center_x:
                left_cones.append((cx, cy))
            else:
                right_cones.append((cx, cy))

        return left_cones, right_cones, mask

# =====================================================
# 라이다 시각화
# =====================================================

    def draw_lidar_debug(self, cone_list, target_angle):

        size = 400
        img = np.zeros((size, size, 3), dtype=np.uint8)
        cx, cy = size // 2, size // 2
        scale = 20  # 1m = 20px

        # 축
        cv2.line(img, (cx, cy), (cx, cy - 150), (80, 80, 80), 1)  # 전방
        cv2.circle(img, (cx, cy), 2, (255, 255, 255), -1)

        # 스캔 범위 호
        for r_m in [2, 4, 6, 8]:
            cv2.circle(img, (cx, cy), r_m * scale, (40, 40, 40), 1)
            cv2.putText(img, f"{r_m}m", (cx + r_m * scale + 2, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (60, 60, 60), 1)

        # 콘 클러스터
        for angle, dist in cone_list:
            rad = np.radians(-angle + 90)  # 시각화용 변환 (위=전방)
            px = int(cx + dist * scale * np.cos(rad))
            py = int(cy - dist * scale * np.sin(rad))
            color = (0, 140, 255) if angle > 0 else (255, 100, 0)
            cv2.circle(img, (px, py), 7, color, -1)
            cv2.putText(img, f"{angle:.0f}d {dist:.1f}m",
                        (px + 5, py), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                        (200, 200, 200), 1)

        # 목표 방향
        if target_angle is not None:
            rad_t = np.radians(-target_angle + 90)
            tx = int(cx + 120 * np.cos(rad_t))
            ty = int(cy - 120 * np.sin(rad_t))
            cv2.arrowedLine(img, (cx, cy), (tx, ty), (0, 255, 0), 2)

        cv2.putText(img, "LIDAR TOP VIEW", (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.imshow("LIDAR", img)

# =====================================================
# Main
# =====================================================

    def get_control(self, image, lidar_ranges):

        if image is None or lidar_ranges is None:
            return 0.0, 0.0

        # ── 라이다 콘 감지 ──────────────────────────
        cone_list = self.detect_lidar_cones(lidar_ranges)

        # 왼쪽/오른쪽 분리
        # angle > 0 = 오른쪽(시계방향), angle < 0 = 왼쪽(반시계방향)
        # 인덱스 1~60 = 반시계(왼쪽) → angle양수, 300~359 = 시계(오른쪽) → angle음수
        # 실제 차량 기준: 왼쪽 콘은 왼쪽에, 오른쪽 콘은 오른쪽에 있어야 함
        # 라이다 반시계=왼쪽=양수각도이므로 양수=왼쪽콘, 음수=오른쪽콘
        left_cones_lidar  = [(a, d) for a, d in cone_list if a > 0]   # 반시계=왼쪽
        right_cones_lidar = [(a, d) for a, d in cone_list if a <= 0]  # 시계=오른쪽

        # ── 카메라 보조 ─────────────────────────────
        bev = self.bird_eye_view(image)
        left_cones_cam, right_cones_cam, mask = self.detect_camera_cones(bev)

        # ── 목표 각도 계산 (라이다 주) ──────────────
        lidar_angle = None

        if left_cones_lidar and right_cones_lidar:
            best_left  = min(left_cones_lidar,  key=lambda x: x[1])
            best_right = min(right_cones_lidar, key=lambda x: x[1])
            lidar_angle = (best_left[0] + best_right[0]) / 2.0
            if self.turn_hold != 0:
                lidar_angle += self.turn_hold

                if abs(lidar_angle) < 20:
                    self.turn_hold = 0
            

        elif left_cones_lidar:
            # 왼쪽 콘만 보임 → 차가 너무 오른쪽에 있거나 왼쪽 벽 따라감
            # 왼쪽 콘 각도(양수)에서 오프셋 빼서 오른쪽으로 치우치게
            best_left = min(left_cones_lidar, key=lambda x: x[1])
            lidar_angle = min(best_left[0] + 30, 60)
            self.turn_hold = 15

        elif right_cones_lidar:
            # 오른쪽 콘만 보임 → 왼쪽으로 이동
            best_right = min(right_cones_lidar, key=lambda x: x[1])
            lidar_angle = max(best_right[0] - 30, -60)

            self.turn_hold = -15 # 왼쪽으로 이동

        # ── 카메라 보조 각도 ─────────────────────────
        cam_angle = None
        bev_h, bev_w = bev.shape[:2]
        bev_cx = bev_w // 2

        if left_cones_cam and right_cones_cam:
            lx = np.mean([x for x, y in left_cones_cam])
            rx = np.mean([x for x, y in right_cones_cam])
            mid_x = (lx + rx) / 2.0
            cam_angle = np.degrees(np.arctan2(mid_x - bev_cx, 120)) * 2.0

        elif left_cones_cam:
            lx = np.mean([x for x, y in left_cones_cam])
            cam_angle = np.degrees(np.arctan2(lx + 150 - bev_cx, 120)) * 2.0

        elif right_cones_cam:
            rx = np.mean([x for x, y in right_cones_cam])
            cam_angle = np.degrees(np.arctan2(rx - 150 - bev_cx, 120)) * 2.0

        # ── 최종 목표 각도 융합 ──────────────────────
        if lidar_angle is not None and cam_angle is not None:
            target_angle = (1.0 - self.CAM_WEIGHT) * lidar_angle + self.CAM_WEIGHT * cam_angle
        elif lidar_angle is not None:
            target_angle = lidar_angle
        elif cam_angle is not None:
            target_angle = cam_angle
        else:
            # 아무것도 감지 안 됨 → 이전 각도 유지
            angle = self.prev_angle * 0.8
            speed = 3
            self.draw_lidar_debug(cone_list, None)
            cv2.imshow("BEV", bev)
            cv2.imshow("MASK", mask)
            cv2.waitKey(1)
            print("NO CONE DETECTED")
            return float(angle), float(speed)

        # ── PD 제어 ──────────────────────────────────
        # target_angle: 양수 = 콘 중심이 왼쪽 → 차가 왼쪽으로 가야 함
        # 차량 조향 부호 확인: 이전 코드에서 양수=왼쪽 동작 확인됨
        error = target_angle
        d_error = error - self.prev_error
        angle = self.KP * error + self.KD * d_error
        self.prev_error = error

        # 데드존
        if abs(angle) < self.MIN_ANGLE:
            angle = 0.0

        angle = np.clip(angle, -self.MAX_ANGLE, self.MAX_ANGLE)

        # 스무딩
        angle = self.steer_alpha * angle + (1.0 - self.steer_alpha) * self.prev_angle
        self.prev_angle = angle

        # 속도
        if abs(angle) < 10:
            speed = 5
        elif abs(angle) < 25:
            speed = 4
        else:
            speed = 3

        # ── 디버그 출력 ──────────────────────────────
        debug = bev.copy()
        for x, y in left_cones_cam:
            cv2.circle(debug, (x, y), 8, (255, 0, 0), -1)
        for x, y in right_cones_cam:
            cv2.circle(debug, (x, y), 8, (0, 0, 255), -1)

        self.draw_lidar_debug(cone_list, target_angle)
        cv2.imshow("BEV",  debug)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        lidar_a_str = f"{lidar_angle:.1f}" if lidar_angle is not None else "N/A"
        print(
            f"L_lidar={len(left_cones_lidar)} R_lidar={len(right_cones_lidar)} "
            f"L_cam={len(left_cones_cam)} R_cam={len(right_cones_cam)} "
            f"lidar_a={lidar_a_str} "
            f"target={target_angle:.1f} angle={angle:.1f}"
        )

        return float(angle), float(speed)