#!/usr/bin/env python3

"""
Sensor Data Visualization Example
This script visualizes data from various simulated sensors
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time


class SensorVisualizer(Node):
    """
    A node that visualizes sensor data from simulation
    """
    def __init__(self):
        super().__init__('sensor_visualizer')
        
        # Store the latest sensor data
        self.lidar_data = None
        self.camera_data = None
        self.depth_data = None
        self.imu_data = None
        
        # Create subscribers for different sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            'depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # Data storage for plotting
        self.imu_roll_history = deque(maxlen=100)
        self.imu_pitch_history = deque(maxlen=100)
        self.imu_yaw_history = deque(maxlen=100)
        self.time_history = deque(maxlen=100)
        
        # Timestamp for IMU history
        self.last_imu_time = 0.0
        
        # Create windows for visualization
        self.lidar_fig, self.lidar_ax = plt.subplots(figsize=(8, 8))
        self.camera_fig, self.camera_ax = plt.subplots(figsize=(10, 6))
        self.depth_fig, self.depth_ax = plt.subplots(figsize=(10, 6))
        self.imu_fig, self.imu_ax = plt.subplots(figsize=(10, 6))
        
        self.get_logger().info('Sensor Visualizer node initialized')
        
    def lidar_callback(self, msg):
        """Process LiDAR scan data"""
        self.lidar_data = msg
        
    def camera_callback(self, msg):
        """Process camera image data"""
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting camera image: {str(e)}')
    
    def depth_callback(self, msg):
        """Process depth image data"""
        try:
            # Convert to float32 for proper processing
            self.depth_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg
        
        # Convert quaternion to Euler angles
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Add to history for plotting
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time == 0.0:
            self.last_imu_time = current_time
            
        self.time_history.append(current_time - self.last_imu_time)
        self.imu_roll_history.append(np.degrees(roll))
        self.imu_pitch_history.append(np.degrees(pitch))
        self.imu_yaw_history.append(np.degrees(yaw))
        
    def visualize_lidar(self):
        """Visualize LiDAR scan in polar coordinates"""
        if self.lidar_data is None:
            return
            
        angles = np.arange(self.lidar_data.angle_min, 
                          self.lidar_data.angle_max + self.lidar_data.angle_increment,
                          self.lidar_data.angle_increment)
        
        # Handle the case where the number of angles doesn't match ranges
        if len(angles) != len(self.lidar_data.ranges):
            # Use linspace to generate correct number of angles
            angles = np.linspace(self.lidar_data.angle_min, 
                               self.lidar_data.angle_max, 
                               len(self.lidar_data.ranges))
        
        ranges = np.array(self.lidar_data.ranges)
        
        # Filter out invalid ranges (inf, nan)
        valid_indices = np.isfinite(ranges)
        angles = angles[valid_indices]
        ranges = ranges[valid_indices]
        
        # Create polar plot
        self.lidar_ax.clear()
        self.lidar_ax.scatter(angles, ranges, s=1)
        self.lidar_ax.set_theta_zero_location('N')  # 0° at the top
        self.lidar_ax.set_theta_direction(-1)  # Clockwise
        self.lidar_ax.set_title('LiDAR Scan')
        self.lidar_ax.set_ylim(0, 10)  # Limit to 10m for visibility
        
        plt.draw()
        plt.pause(0.001)
        
    def visualize_camera(self):
        """Visualize camera image"""
        if self.camera_data is None:
            return
            
        self.camera_ax.clear()
        # Convert BGR to RGB for matplotlib
        img_rgb = cv2.cvtColor(self.camera_data, cv2.COLOR_BGR2RGB)
        self.camera_ax.imshow(img_rgb)
        self.camera_ax.set_title('Camera Image')
        self.camera_ax.axis('off')
        
        plt.draw()
        plt.pause(0.001)
        
    def visualize_depth(self):
        """Visualize depth image"""
        if self.depth_data is None:
            return
            
        self.depth_ax.clear()
        # Normalize depth for visualization
        depth_normalized = cv2.normalize(self.depth_data, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = depth_normalized.astype(np.uint8)
        self.depth_ax.imshow(depth_normalized, cmap='viridis')
        self.depth_ax.set_title('Depth Image')
        self.depth_ax.axis('off')
        
        plt.draw()
        plt.pause(0.001)
        
    def visualize_imu(self):
        """Visualize IMU orientation over time"""
        if len(self.time_history) < 2:
            return
            
        self.imu_ax.clear()
        times = list(self.time_history)
        rolls = list(self.imu_roll_history)
        pitches = list(self.imu_pitch_history)
        yaws = list(self.imu_yaw_history)
        
        self.imu_ax.plot(times, rolls, label='Roll', linewidth=2)
        self.imu_ax.plot(times, pitches, label='Pitch', linewidth=2)
        self.imu_ax.plot(times, yaws, label='Yaw', linewidth=2)
        
        self.imu_ax.set_title('IMU Orientation Over Time')
        self.imu_ax.set_xlabel('Time (s)')
        self.imu_ax.set_ylabel('Angle (degrees)')
        self.imu_ax.legend()
        self.imu_ax.grid(True)
        
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    visualizer = SensorVisualizer()
    
    # Start visualization loop in a separate thread
    def viz_loop():
        while rclpy.ok():
            try:
                # Visualize each sensor type
                visualizer.visualize_lidar()
                visualizer.visualize_camera()
                visualizer.visualize_depth()
                visualizer.visualize_imu()
                
                time.sleep(0.1)  # Update every 100ms
                
            except KeyboardInterrupt:
                break
    
    viz_thread = threading.Thread(target=viz_loop, daemon=True)
    viz_thread.start()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()