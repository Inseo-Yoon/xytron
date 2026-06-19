#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, time, cv2, math
import numpy as np
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from .shortcut import Mission6Manager

class TrackDriverNode(Node):

    def __init__(self):
        super().__init__('driver')
        self.get_logger().info('----- Xycar self-driving node started -----')
        
        self.image = None 
        self.motor_msg = XycarMotor()        
        self.lidar_ranges = None
        self.bridge = CvBridge()
        
        # Mission Control Variables
        self.lap_count = 1 
        self.drive_state = "LANE" 
        self.is_lap_trigger = False 

        self.m6_manager = Mission6Manager()
        
        self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)
        self.sub_front = self.create_subscription(
            Image, '/usb_cam/image_raw/front', self.cam_callback, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        
        self.get_logger().info("Track Driver Node Initialized")
            
    def cam_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges   
      
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)

    def check_lap_count(self):
        """
        [Logic] Add line detection or marker detection code here.
        Example: If lap trigger condition is met, increment lap_count.
        """
        # if line_detected and not self.is_lap_trigger:
        #     self.lap_count += 1
        #     self.is_lap_trigger = True
        #     self.get_logger().info(f"Lap completed! Current Lap: {self.lap_count}")
        pass

    def detect_traffic_light(self):
        if self.image is None: return False
        return True 

    def is_police_car_blocking(self):
        if self.image is None: return False
        return False 

    def get_lidar_avoidance_angle(self):
        if self.lidar_ranges is None or len(self.lidar_ranges) == 0:
            return 0.0
            
        num_points = len(self.lidar_ranges)
        center_idx = num_points // 2
        scan_span = int(num_points * (30.0 / 360.0)) 
        
        left_sector = self.lidar_ranges[center_idx : center_idx + scan_span]
        right_sector = self.lidar_ranges[center_idx - scan_span : center_idx]
        
        left_dist = np.mean([d for d in left_sector if 0.1 < d < 10.0])
        right_dist = np.mean([d for d in right_sector if 0.1 < d < 10.0])
        
        if min(left_dist, right_dist) < 1.5:
            return -25.0 if left_dist > right_dist else 25.0
                
        return None

    def process_lane_following(self):
        return 0.0

    def main_loop(self):
        self.get_logger().info("======================================")
        self.get_logger().info("       START DRIVING ...              ")
        self.get_logger().info("======================================")

        BASE_SPEED = 5.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.image is None:
                continue
                
            self.check_lap_count()
            
            # State Machine
            if self.drive_state == "LANE":
                steer_angle = self.process_lane_following()
                self.drive(steer_angle, BASE_SPEED)
                
            elif self.drive_state == "INTERSECTION":
                left_signal = self.detect_traffic_light()
                blocked = self.is_police_car_blocking()
                
                if self.lap_count >= 2 and left_signal and not blocked:
                    self.get_logger().info("Shortcut condition met! Turning left.")
                    self.drive(-35.0, BASE_SPEED)
                    time.sleep(1.5) 
                    self.drive_state = "SHORTCUT"
                else:
                    self.get_logger().info("Shortcut unavailable. Continuing straight.")
                    self.drive(0.0, BASE_SPEED)
                    time.sleep(1.0)
                    self.drive_state = "LANE"
                    
            elif self.drive_state == "SHORTCUT":
                avoid_steer = self.get_lidar_avoidance_angle()
                
                if avoid_steer is not None:
                    self.drive(avoid_steer, BASE_SPEED)
                else:
                    steer_angle = self.process_lane_following()
                    self.drive(steer_angle, BASE_SPEED)
            
            if self.lap_count > 3:
                self.get_logger().info("Course completed! Stopping.")
                self.drive(0, 0)
                break

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

if __name__ == '__main__':
    main()