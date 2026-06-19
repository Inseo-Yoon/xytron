# -*- coding: utf-8 -*-
import cv2
import numpy as np

class Mission6Manager:
    def __init__(self):
        # 6번 미션에만 쓸 변수나 임계값(Threshold)이 있다면 여기 선언
        pass

    def detect_traffic_light(self, image):
        # 기존에 track_drive에 짜두셨던 신호등 오픈CV 코드 옮기기
        # self.image 대신 매개변수로 받은 image를 사용하게끔만 수정
        return True 

    def is_police_car_blocking(self, image):
        # 기존에 짜두셨던 경찰차 오픈CV 코드 옮기기
        return False

    def get_lidar_avoidance_angle(self, lidar_ranges):
        # 기존에 짜두셨던 라이다 계산 코드 옮기기
        return None