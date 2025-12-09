#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np
from cv_bridge import CvBridge
import cv2


class SensorValidator(Node):
    """
    A node that validates sensor data from simulation
    """
    def __init__(self):
        super().__init__('sensor_validator')
        
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
        
        # Validation statistics
        self.lidar_valid_count = 0
        self.lidar_invalid_count = 0
        self.camera_valid_count = 0
        self.camera_invalid_count = 0
        self.depth_valid_count = 0
        self.depth_invalid_count = 0
        self.imu_valid_count = 0
        self.imu_invalid_count = 0
        
        self.get_logger().info('Sensor Validator node initialized')
        
    def lidar_callback(self, msg):
        """
        Validate LiDAR scan data
        """
        is_valid = True
        
        # Check if ranges array is not empty
        if len(msg.ranges) == 0:
            self.get_logger().error('LiDAR: Empty ranges array')
            is_valid = False
            
        # Check if ranges are within expected min/max values
        for range_val in msg.ranges:
            if not (msg.range_min <= range_val <= msg.range_max) and not np.isnan(range_val):
                if range_val != float('inf'):  # Inf is acceptable for no detection
                    self.get_logger().warning(f'LiDAR: Range {range_val} out of bounds [{msg.range_min}, {msg.range_max}]')
        
        # Check angle parameters
        if msg.angle_increment <= 0:
            self.get_logger().error('LiDAR: Invalid angle increment')
            is_valid = False
            
        # Calculate expected number of ranges
        expected_num = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        if len(msg.ranges) != expected_num:
            self.get_logger().warning(f'LiDAR: Expected {expected_num} ranges but got {len(msg.ranges)}')
        
        if is_valid:
            self.lidar_valid_count += 1
        else:
            self.lidar_invalid_count += 1
            
        self.get_logger().info(f'LiDAR validation - Valid: {self.lidar_valid_count}, Invalid: {self.lidar_invalid_count}')
    
    def camera_callback(self, msg):
        """
        Validate camera image data
        """
        is_valid = True
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check image dimensions
            if cv_image.size == 0:
                self.get_logger().error('Camera: Empty image')
                is_valid = False
            else:
                self.get_logger().info(f'Camera: Received image {cv_image.shape[1]}x{cv_image.shape[0]}')
                
        except Exception as e:
            self.get_logger().error(f'Camera: Error converting image: {str(e)}')
            is_valid = False
            
        if is_valid:
            self.camera_valid_count += 1
        else:
            self.camera_invalid_count += 1
            
        self.get_logger().info(f'Camera validation - Valid: {self.camera_valid_count}, Invalid: {self.camera_invalid_count}')
    
    def depth_callback(self, msg):
        """
        Validate depth image data
        """
        is_valid = True
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Check image dimensions
            if cv_image.size == 0:
                self.get_logger().error('Depth: Empty image')
                is_valid = False
            else:
                # Check depth values (should be positive, finite, and within reasonable range)
                finite_values = cv_image[np.isfinite(cv_image)]
                if len(finite_values) > 0:
                    if np.min(finite_values) < 0:  # Negative depth doesn't make sense
                        self.get_logger().warning(f'Depth: Found negative depth values (min: {np.min(finite_values)})')
                    if np.max(finite_values) > 100:  # Very large depth might indicate error
                        self.get_logger().info(f'Depth: Found large depth values (max: {np.max(finite_values)})')
                
                self.get_logger().info(f'Depth: Received image {cv_image.shape[1]}x{cv_image.shape[0]} with depth range [{np.min(cv_image):.2f}, {np.max(cv_image):.2f}]')
                
        except Exception as e:
            self.get_logger().error(f'Depth: Error converting image: {str(e)}')
            is_valid = False
            
        if is_valid:
            self.depth_valid_count += 1
        else:
            self.depth_invalid_count += 1
            
        self.get_logger().info(f'Depth validation - Valid: {self.depth_valid_count}, Invalid: {self.depth_invalid_count}')
    
    def imu_callback(self, msg):
        """
        Validate IMU data
        """
        is_valid = True
        
        # Validate orientation quaternion
        norm = msg.orientation.w**2 + msg.orientation.x**2 + msg.orientation.y**2 + msg.orientation.z**2
        if abs(norm - 1.0) > 0.1:  # Allow some tolerance
            self.get_logger().warning(f'IMU: Quaternion not normalized (norm = {norm})')
        
        # Validate linear acceleration (should be around 9.8 for z-axis when robot is stable)
        gravity = np.sqrt(msg.linear_acceleration.x**2 + 
                         msg.linear_acceleration.y**2 + 
                         msg.linear_acceleration.z**2)
        
        if msg.linear_acceleration.z > 0 and abs(msg.linear_acceleration.z - 9.8) > 2.0:
            self.get_logger().info(f'IMU: Z acceleration seems off (expected ~9.8, got {msg.linear_acceleration.z:.2f})')
        
        # Validate angular velocity (should be reasonable values)
        ang_vel_mag = np.sqrt(msg.angular_velocity.x**2 + 
                             msg.angular_velocity.y**2 + 
                             msg.angular_velocity.z**2)
        if ang_vel_mag > 10.0:  # Very high angular velocity might indicate error
            self.get_logger().info(f'IMU: High angular velocity detected: {ang_vel_mag:.2f} rad/s')
        
        if is_valid:
            self.imu_valid_count += 1
        else:
            self.imu_invalid_count += 1
            
        self.get_logger().info(f'IMU validation - Valid: {self.imu_valid_count}, Invalid: {self.imu_invalid_count}')


def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()