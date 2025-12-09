#!/usr/bin/env python3
"""
Isaac ROS Bridge Example
This script demonstrates how to interface with Isaac Sim using ROS 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class IsaacROSBridge(Node):
    """
    Example node demonstrating Isaac ROS bridge functionality
    """
    def __init__(self):
        super().__init__('isaac_ros_bridge')
        
        # Create a bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Create subscribers for Isaac Sim sensors
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgb_camera/final/image',
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera/depth/image',
            self.depth_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Create publisher for robot control commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('Isaac ROS Bridge node initialized')
        
        # Control variables
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.obstacle_detected = False
        
    def rgb_callback(self, msg):
        """
        Process RGB camera data from Isaac Sim
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform some computer vision processing
            # For example, detect edges using Canny
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            
            # Draw edges on original image
            color_edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            result = cv2.addWeighted(cv_image, 0.7, color_edges, 0.3, 0)
            
            # Display the result
            cv2.imshow('RGB Camera + Edges', result)
            cv2.waitKey(1)
            
            self.get_logger().debug(f'Received RGB image: {cv_image.shape}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing RGB image: {str(e)}')
    
    def depth_callback(self, msg):
        """
        Process depth camera data from Isaac Sim
        """
        try:
            # Convert ROS Image message to OpenCV image (depth format)
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Normalize depth for visualization
            depth_normalized = cv2.normalize(cv_depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_colormap = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            
            # Display the depth image
            cv2.imshow('Depth Camera', depth_colormap)
            cv2.waitKey(1)
            
            # Analyze depth data
            finite_depths = cv_depth[np.isfinite(cv_depth)]
            if len(finite_depths) > 0:
                avg_depth = np.mean(finite_depths)
                self.get_logger().debug(f'Average depth: {avg_depth:.2f}m')
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def lidar_callback(self, msg):
        """
        Process LiDAR data from Isaac Sim
        """
        # Check for obstacles in front of the robot
        # Assuming the front is at index len(ranges)//2
        if len(msg.ranges) > 0:
            front_idx = len(msg.ranges) // 2
            front_range = msg.ranges[front_idx]
            
            # Check if there's an obstacle within 1 meter
            if 0 < front_range < 1.0:
                self.obstacle_detected = True
                self.get_logger().warn('Obstacle detected in front!')
            else:
                self.obstacle_detected = False
    
    def control_loop(self):
        """
        Main control loop to send commands to the robot
        """
        cmd = Twist()
        
        if self.obstacle_detected:
            # If obstacle detected, turn left
            cmd.linear.x = 0.0  # Stop moving forward
            cmd.angular.z = 0.5  # Turn left
        else:
            # Otherwise, move forward
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0  # Don't turn
            
        # Publish the command
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the Isaac ROS Bridge node
    isaac_bridge = IsaacROSBridge()
    
    try:
        rclpy.spin(isaac_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up OpenCV windows
        cv2.destroyAllWindows()
        isaac_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()