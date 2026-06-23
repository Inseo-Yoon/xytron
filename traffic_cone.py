import cv2
import numpy as np

class TrafficCone:


    def __init__(self):

        self.prev_error = 0.0
        self.prev_angle = 0.0

        self.KP = 0.15
        self.KD = 0.05

        self.MAX_ANGLE = 70

        self.TRACK_OFFSET = 200

        self.BASE_SPEED = 5

        self.steer_alpha = 0.9

        cv2.namedWindow("BEV", cv2.WINDOW_NORMAL)
   

        cv2.namedWindow("MASK", cv2.WINDOW_NORMAL)
        

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
# Cone Detection
# =====================================================

    def detect_cones(self, image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([0, 120, 120])
        upper = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((15, 15), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((3,3), np.uint8)
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        cones = []

        for cnt in contours:

            area = cv2.contourArea(cnt)

            if area < 250:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if h < 10:
                continue

            cx = x + w // 2
            cy = y + h // 2

            if cy < image.shape[0] * 0.3:
                continue

            cones.append((cx, cy))

        cones.sort(key=lambda p: -p[1])
        merged = []

        for cx, cy in cones:

            found = False

            for i, (mx, my) in enumerate(merged):

                if np.hypot(cx - mx, cy - my) < 70:

                    merged[i] = (
                        int((mx + cx) / 2),
                        int((my + cy) / 2)
                    )

                    found = True
                    break

            if not found:
                merged.append((cx, cy))

        cones = merged
        
        
        print(
            "left =", len([c for c in cones if c[0] < image.shape[1]//2]),
            "right =", len([c for c in cones if c[0] >= image.shape[1]//2]),
            cones
        )
        return cones, mask

# =====================================================
# Main
# =====================================================

    def get_control(self, image, lidar_ranges):

        if image is None:
            return 0.0, 0.0

        bev = self.bird_eye_view(image)

        cones, mask = self.detect_cones(bev)

        debug = bev.copy()

        h, w = bev.shape[:2]

        center_x = w // 2

        left_cones = []
        right_cones = []

        for cx, cy in cones:

            if cx < center_x:
                left_cones.append((cx, cy))
            else:
                right_cones.append((cx, cy))

        left_cones.sort(key=lambda p: -p[1])
        right_cones.sort(key=lambda p: -p[1])

    # =========================
        # 양쪽 콘 모두 보임
# =========================

        if len(left_cones) >= 2 and len(right_cones) >= 2:

            
            offset_pts = []

            for x, y in cones:
                offset_pts.append([x + self.TRACK_OFFSET, y])

            pts = np.array(offset_pts, dtype=np.float32)
            # pts = np.array(cones)

            vx, vy, x0, y0 = cv2.fitLine(
                pts,
                cv2.DIST_L2,
                0,
                0.01,
                0.01
            )

            look_y = int(h * 0.65)

            target_x = int(
                x0 +
                (look_y - y0) * (vx / vy)
            )

        elif len(left_cones) > 0 and len(right_cones) == 0:

            cone_x, cone_y = left_cones[0]

            target_x = cone_x + self.TRACK_OFFSET

            look_y = cone_y

        else:

            angle = self.prev_angle * 0.8

            speed = 3

            cv2.imshow("BEV", debug)
            cv2.imshow("MASK", mask)
            cv2.waitKey(1)

            return angle, speed
        

        dx = target_x - center_x

        angle = np.degrees(
            np.arctan2(dx, 120)
        )

        angle *= 3.0

        angle = np.clip(
            angle,
            -self.MAX_ANGLE,
            self.MAX_ANGLE
        )

        angle = (
            self.steer_alpha * angle +
            (1.0 - self.steer_alpha) * self.prev_angle
        )


        self.prev_angle = angle

        if abs(angle) < 10:
            speed = 5
        elif abs(angle) < 20:
            speed = 4
        else:
            speed = 3

        # 디버그

        for x, y in left_cones:
            cv2.circle(debug, (x, y), 8, (255, 0, 0), -1)

        for x, y in right_cones:
            cv2.circle(debug, (x, y), 8, (0, 0, 255), -1)
        for x, y in cones:
            cv2.circle(debug, (x + 180, y), 5, (0,255,255), -1)
        cv2.circle(
            debug,
            (int(target_x), int(look_y)),
            12,
            (0, 255, 255),
            -1
        )

        cv2.line(
            debug,
            (center_x, h),
            (int(target_x), int(look_y)),
            (0, 255, 255),
            3
        )
        cv2.waitKey(1)
        print(
            f"L={len(left_cones)} "
            f"R={len(right_cones)} "
            f"target={target_x} "
            f"angle={angle:.1f}"
        )

        return float(angle), float(speed)

