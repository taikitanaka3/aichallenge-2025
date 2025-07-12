#!/usr/bin/env python3
"""

Camera Controller for Vibe Coding Kit

author: ttanaka3

"""
import cv2
import numpy as np

class CameraController:
    def __init__(self):
        self.max_steering_angle = 0.5
        self.max_acceleration = 2.0
        # BEV変換用のsrc/dst（手動設定例）
        self.src = np.float32([
            [800, 650],   # 左上（画像座標）
            [1120, 650],  # 右上
            [1800, 1070], # 右下
            [120, 1070]   # 左下
        ])
        self.dst = np.float32([
            [600, 0],     # 左上（BEV座標）
            [1320, 0],    # 右上
            [1320, 1080], # 右下
            [600, 1080]   # 左下
        ])
        self.M = cv2.getPerspectiveTransform(self.src, self.dst)

    def bev_transform(self, image):
        height, width = image.shape[:2]
        bev = cv2.warpPerspective(image, self.M, (width, height))
        return bev

    def process_image(self, image: np.ndarray):
        bev = self.bev_transform(image)
        # 1. HSV変換で白・黄色マスク（BGR→HSV）
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, (0, 0, 180), (180, 30, 255))
        mask_yellow = cv2.inRange(hsv, (15, 60, 120), (35, 255, 255))
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        # 2. マスク画像でエッジ検出
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        # 3. ROI（BEV画像の下半分）
        height, width = edges.shape
        roi = edges[height//2:, :]
        # 4. Hough変換で直線検出
        lines = cv2.HoughLinesP(roi, 1, np.pi/180, threshold=30, minLineLength=40, maxLineGap=100)
        cx_list = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cx = (x1 + x2) // 2
                cx_list.append(cx)
        if cx_list:
            lane_center = int(np.mean(cx_list))
            error = (lane_center - (width // 2)) / (width // 2)
            steering_angle = error * self.max_steering_angle * 0.5
        else:
            steering_angle = 0.0
        acceleration = self.max_acceleration * 0.3
        # 可視化用
        self.last_bev = bev
        self.last_edges = edges
        self.last_lines = lines
        return steering_angle, acceleration

    def visualize_processing(self, image: np.ndarray, processed_image: np.ndarray = None) -> np.ndarray:
        # BEV画像・エッジ・直線を重ねて可視化
        vis = self.last_bev.copy() if hasattr(self, 'last_bev') else image.copy()
        if hasattr(self, 'last_edges'):
            edges_color = cv2.cvtColor(self.last_edges, cv2.COLOR_GRAY2BGR)
            vis = cv2.addWeighted(vis, 0.7, edges_color, 0.3, 0)
        if hasattr(self, 'last_lines') and self.last_lines is not None:
            height = vis.shape[0]
            for line in self.last_lines:
                x1, y1, x2, y2 = line[0]
                # ROI分だけy座標を下にずらす
                cv2.line(vis, (x1, y1+height//2), (x2, y2+height//2), (0,255,0), 3)
        return vis 
