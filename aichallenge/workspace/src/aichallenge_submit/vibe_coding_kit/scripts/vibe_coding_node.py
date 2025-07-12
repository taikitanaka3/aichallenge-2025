#!/usr/bin/env python3
"""

Vibe Coding Kit - Camera-based Control System

author: ttanaka3

"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand
import csv
import os
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage

from camera_controller import CameraController

# Define QoS profile for camera data
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class VibeCodingNode(Node):
    def __init__(self):
        super().__init__('vibe_coding_kit')
        self.get_logger().info("Vibe Coding Kit - Camera-based Control System")

        # Camera subscribers
        self.create_subscription(
            Image, 
            "/sensing/camera/image_raw", 
            self.on_camera_image, 
            best_effort_qos
        )
        self.create_subscription(
            CameraInfo, 
            "/sensing/camera/camera_info", 
            self.on_camera_info, 
            best_effort_qos
        )

        # Control command publisher
        self.control_cmd_pub = self.create_publisher(
            AckermannControlCommand, 
            '/control/command/control_cmd', 
            10
        )

        # Initialize camera controller
        self.camera_controller = CameraController()
        
        # Initialize camera data
        self.current_image = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.debug_image_pub = self.create_publisher(RosImage, '/vibe_coding_kit/debug_image', 1)
        self.csv_path = '/tmp/vibe_coding_kit_log.csv'
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['frame', 'steering_angle', 'acceleration'])
        self.steering_history = []
        self.accel_history = []
        plt.ion()
        fig, ax = plt.subplots()
        self.fig = fig
        self.ax = ax
        self.line1, = ax.plot([], [], label='Steering')
        self.line2, = ax.plot([], [], label='Accel')
        ax.legend()
        ax.set_xlabel('Frame')
        ax.set_ylabel('Value')
        ax.set_title('Control Command History')
        self.frame_count = 0
        # デバッグウィンドウをデフォルトで表示
        cv2.namedWindow("Vibe Coding Kit - Camera Processing", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Vibe Coding Kit - Camera Processing", 960, 540)

    def on_camera_image(self, msg: Image):
        """カメラ画像のコールバック関数"""
        try:
            # Convert ROS Image to OpenCV format
            height = msg.height
            width = msg.width
            encoding = msg.encoding
            
            # Convert to numpy array
            image_array = np.frombuffer(msg.data, dtype=np.uint8)
            image_array = image_array.reshape((height, width, 3))
            
            # Convert BGR to RGB if needed
            if encoding == 'bgr8':
                image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
            
            self.current_image = image_array
            self.frame_count += 1
            
            self.get_logger().info(f"Received camera image: {width}x{height}, frame: {self.frame_count}")
            
            # Process image and generate control commands
            if self.current_image is not None:
                self.process_image_and_control()
                
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def on_camera_info(self, msg: CameraInfo):
        """カメラ情報のコールバック関数"""
        self.camera_info = msg
        self.get_logger().info(f"Received camera info: {msg.width}x{msg.height}")

    def process_image_and_control(self):
        """画像を処理して制御指令を生成"""
        if self.current_image is None:
            return

        try:
            steering_angle, acceleration = self.camera_controller.process_image(self.current_image)
            vis = self.camera_controller.visualize_processing(self.current_image)
            # 1. OpenCVウィンドウ表示（デフォルトでウィンドウ生成済み）
            cv2.imshow("Vibe Coding Kit - Camera Processing", vis)
            cv2.waitKey(1)
            # 2. ROSトピックで配信
            debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding='rgb8')
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self.debug_image_pub.publish(debug_msg)
            # 3. CSV保存
            self.csv_writer.writerow([self.frame_count, steering_angle, acceleration])
            self.csv_file.flush()
            # 4. matplotlibグラフ
            self.steering_history.append(steering_angle)
            self.accel_history.append(acceleration)
            self.line1.set_data(range(len(self.steering_history)), self.steering_history)
            self.line2.set_data(range(len(self.accel_history)), self.accel_history)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            # 制御指令パブリッシュ
            self.publish_control_command(steering_angle, acceleration)
            self.get_logger().info(f"Control: steering={steering_angle:.3f}, accel={acceleration:.3f}")
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")

    def publish_control_command(self, steering_angle: float, acceleration: float):
        """制御指令をパブリッシュ"""
        stamp = Time(sec=int(self.get_clock().now().seconds_nanoseconds()[0]))
        
        # Create control command
        control_cmd = AckermannControlCommand()
        
        # Lateral command (steering)
        lateral_cmd = AckermannLateralCommand()
        lateral_cmd.stamp.sec = stamp.sec
        lateral_cmd.stamp.nanosec = stamp.nanosec
        lateral_cmd.steering_tire_angle = steering_angle
        lateral_cmd.steering_tire_rotation_rate = 0.0

        # Longitudinal command (speed/acceleration)
        longitudinal_cmd = LongitudinalCommand()
        longitudinal_cmd.stamp.sec = stamp.sec
        longitudinal_cmd.stamp.nanosec = stamp.nanosec
        longitudinal_cmd.speed = 0.0  # Speed control can be added here
        longitudinal_cmd.acceleration = acceleration
        longitudinal_cmd.jerk = 0.0

        # Set control command
        control_cmd.stamp.sec = stamp.sec
        control_cmd.stamp.nanosec = stamp.nanosec
        control_cmd.lateral = lateral_cmd
        control_cmd.longitudinal = longitudinal_cmd
        
        # Publish
        self.control_cmd_pub.publish(control_cmd)

    def destroy_node(self):
        self.csv_file.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VibeCodingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
