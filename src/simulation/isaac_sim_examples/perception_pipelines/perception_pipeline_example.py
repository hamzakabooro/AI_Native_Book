#!/usr/bin/env python3
"""
Isaac ROS Perception Pipeline Example
This script demonstrates a complete perception pipeline using Isaac ROS
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters


class IsaacPerceptionPipeline(Node):
    """
    Example Isaac ROS perception pipeline node
    """
    def __init__(self):
        super().__init__('isaac_perception_pipeline')
        
        # Create a bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Create subscribers for sensor data
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_rect_color')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info')
        
        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.info_sub], 
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.image_info_callback)
        
        # Create publisher for processed data
        self.result_pub = self.create_publisher(
            MarkerArray,
            '/perception_pipeline/results',
            10
        )
        
        # Create publisher for robot commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Isaac Perception Pipeline initialized')
        
    def image_info_callback(self, image_msg, info_msg):
        """
        Process synchronized image and camera info
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Apply perception algorithms (example: color-based detection)
            result_image = self.color_detection(cv_image)
            
            # Process camera info if needed
            self.get_logger().debug(f'Camera info: {info_msg.width}x{info_msg.height}')
            
            # Simulate publishing results to visualization
            self.publish_results(result_image, image_msg.header)
            
            # Publish robot command based on perception
            self.publish_navigation_command(result_image)
            
        except Exception as e:
            self.get_logger().error(f'Error in perception pipeline: {str(e)}')
    
    def color_detection(self, image):
        """
        Example perception algorithm: detect objects by color
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color (example)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        # Upper red range (HSV wraps around)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        
        # Combine masks
        mask = mask1 + mask2
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw detected objects on the image
        result_image = image.copy()
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small contours
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Calculate center of the detected object
                center_x, center_y = x + w//2, y + h//2
                cv2.circle(result_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return result_image
    
    def publish_results(self, result_image, header):
        """
        Publish perception results
        """
        # In a real implementation, this would convert results to appropriate ROS messages
        # For now, just display the result
        cv2.imshow('Perception Pipeline Result', result_image)
        cv2.waitKey(1)
    
    def publish_navigation_command(self, result_image):
        """
        Publish navigation command based on perception results
        """
        # Simple navigation logic: if object detected in the center third of image, stop
        height, width, _ = result_image.shape
        
        # Check center region for detected objects (green rectangles)
        center_region = result_image[height//4:3*height//4, width//3:2*width//3]
        
        # Count pixels that are likely from detection rectangles (green)
        green_pixels = np.sum((center_region[:,:,1] > 200) & (center_region[:,:,0] < 50) & (center_region[:,:,2] < 50))
        
        cmd = Twist()
        if green_pixels > 1000:  # If many green pixels detected in center
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.0
            self.get_logger().info('Obstacle detected in front, stopping')
        else:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0
            self.get_logger().info('Path clear, moving forward')
            
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    perception_pipeline = IsaacPerceptionPipeline()
    
    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()