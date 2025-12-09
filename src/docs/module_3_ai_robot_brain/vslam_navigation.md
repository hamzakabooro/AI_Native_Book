# Visual SLAM and Navigation with Isaac ROS

## Overview
Visual Simultaneous Localization and Mapping (VSLAM) is a critical capability for autonomous robots. This module covers implementing Visual SLAM with Isaac ROS and using it for robot navigation in unknown environments.

## Learning Objectives
- Understand Visual SLAM principles and applications
- Implement VSLAM using Isaac ROS components
- Integrate VSLAM with navigation systems
- Evaluate VSLAM performance in simulation

## Visual SLAM Fundamentals

### Principles of Visual SLAM
Visual SLAM estimates the robot's position relative to its environment while simultaneously building a map of that environment using visual data from cameras. The process involves:

1. **Feature Detection**: Identifying distinctive points in images
2. **Feature Matching**: Corresponding features across frames
3. **Motion Estimation**: Calculating camera/robot motion
4. **Mapping**: Building a 3D map of the environment
5. **Loop Closure**: Recognizing previously visited places to correct drift

### Types of Visual SLAM
- **Monocular SLAM**: Uses a single camera, requires motion to estimate depth
- **Stereo SLAM**: Uses stereo cameras for direct depth estimation
- **RGB-D SLAM**: Uses RGB-D cameras with explicit depth information

## Isaac ROS Visual SLAM

### Available VSLAM Components
Isaac ROS provides optimized VSLAM components that leverage GPU acceleration:

- **Isaac ROS Visual SLAM**: Real-time Visual SLAM with loop closure
- **Isaac ROS Stereo Dense Reconstruction**: 3D environment reconstruction
- **Isaac ROS Occupancy Grid Localizer**: Localization in known maps

### Setting up Isaac ROS Visual SLAM
```bash
# Install Isaac ROS Visual SLAM
sudo apt install ros-humble-isaac-ros-visual-slam
```

## Implementing VSLAM with Isaac ROS

### Basic VSLAM Pipeline
Here's a basic configuration for Isaac ROS Visual SLAM:

```yaml
# vslam_pipeline.yaml
name: vslam_pipeline
nodes:
  - name: "rectify_left"
    namespace: "isaac_ros"
    package: "isaac_ros_image_proc"
    executable: "rectify_node"
    parameters:
      - use_sim_time: true
    remappings:
      - from: "image_raw"
        to: "/stereo_camera/left/image_raw"
      - from: "camera_info"
        to: "/stereo_camera/left/camera_info"
      - from: "image_rect"
        to: "/stereo_camera/left/image_rect_color"

  - name: "rectify_right"
    namespace: "isaac_ros"
    package: "isaac_ros_image_proc"
    executable: "rectify_node"
    parameters:
      - use_sim_time: true
    remappings:
      - from: "image_raw"
        to: "/stereo_camera/right/image_raw"
      - from: "camera_info"
        to: "/stereo_camera/right/camera_info"
      - from: "image_rect"
        to: "/stereo_camera/right/image_rect_color"

  - name: "stereo_image_proc"
    namespace: "isaac_ros"
    package: "isaac_ros_stereo_image_proc"
    executable: "isaac_ros_stereo_rectify_node"
    parameters:
      - use_sim_time: true
    remappings:
      - from: "left/image_rect"
        to: "/stereo_camera/left/image_rect_color"
      - from: "right/image_rect"
        to: "/stereo_camera/right/image_rect_color"
      - from: "left/camera_info"
        to: "/stereo_camera/left/camera_info"
      - from: "right/camera_info"
        to: "/stereo_camera/right/camera_info"
      - from: "disparity"
        to: "/disparity_map"

  - name: "visual_slam"
    namespace: "isaac_ros"
    package: "isaac_ros_visual_slam"
    executable: "visual_slam_node"
    parameters:
      - use_sim_time: true
      - enable_rectified_pose: true
      - enable_fisheye_distortion: false
    remappings:
      - from: "visual_slam_node/left/camera_info"
        to: "/stereo_camera/left/camera_info"
      - from: "visual_slam_node/right/camera_info"
        to: "/stereo_camera/right/camera_info"
      - from: "visual_slam_node/left/image"
        to: "/stereo_camera/left/image_rect_color"
      - from: "visual_slam_node/right/image"
        to: "/stereo_camera/right/image_rect_color"
      - from: "visual_slam_node/visual_slam/traj_estimate"
        to: "/camera/pose"
      - from: "visual_slam_node/visual_slam/mapped_points"
        to: "/mapped_points"
```

### Launching the VSLAM Pipeline
```bash
# Launch the visual SLAM pipeline
ros2 launch isaac_ros_visual_slam visual_slam.launch.py
```

## Integrating VSLAM with Navigation

### Creating a Map
Visual SLAM creates a map while localizing the robot. To create a map:

1. **Initialize SLAM**: Start the robot at a known location
2. **Explore the environment**: Navigate through the area
3. **Save the map**: Export the map for later use

```bash
# Save the map created by VSLAM
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Using VSLAM in Navigation
Once you have a map, you can use it for navigation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import numpy as np


class VSLAMNavigationNode(Node):
    """
    Node that integrates VSLAM with navigation
    """
    def __init__(self):
        super().__init__('vslam_navigation')
        
        # Create subscribers for VSLAM pose and camera data
        self.pose_sub = self.create_subscription(
            Odometry,
            '/camera/pose',
            self.pose_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )
        
        # Create publishers for navigation goals
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Create publisher for visualization
        self.marker_pub = self.create_publisher(
            Marker,
            '/vslam_navigation/markers',
            10
        )
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_pose = None
        self.map_points = []  # Store map points from VSLAM
        
        self.get_logger().info('VSLAM Navigation node initialized')
    
    def pose_callback(self, msg):
        """
        Handle VSLAM pose updates
        """
        self.current_pose = msg.pose.pose
        self.get_logger().debug(f'VSLAM pose updated: {self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}')
        
        # Update map if needed
        # In a real implementation, this would update the map with new observations
        
    def image_callback(self, msg):
        """
        Process camera images for navigation
        """
        # In a real implementation, this would run additional perception
        # to identify goals or obstacles from the camera image
        pass
    
    def compute_navigation_path(self, goal):
        """
        Compute path to goal using VSLAM map
        """
        if not self.current_pose:
            self.get_logger().warn('No current pose available for path planning')
            return False
            
        # In a real implementation, this would use:
        # 1. The VSLAM map to identify obstacles
        # 2. Path planning algorithms to find a safe path
        # 3. Local planning to avoid dynamic obstacles
        
        # Simple example: just move toward the goal
        cmd_x = goal.position.x - self.current_pose.position.x
        cmd_y = goal.position.y - self.current_pose.position.y
        
        # Normalize direction
        dist = np.sqrt(cmd_x**2 + cmd_y**2)
        if dist > 0.1:  # If not close to goal
            cmd_x /= dist
            cmd_y /= dist
        else:
            cmd_x = 0.0
            cmd_y = 0.0
            
        # Publish visualization marker
        self.publish_goal_marker(goal)
        
        return True
    
    def publish_goal_marker(self, goal):
        """
        Publish a visualization marker for the goal
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vslam_navigation"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = goal.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    
    vslam_nav = VSLAMNavigationNode()
    
    try:
        rclpy.spin(vslam_nav)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Performance Considerations

### Computational Requirements
VSLAM is computationally intensive. Isaac ROS optimizes performance through:

- GPU acceleration for feature detection and matching
- Efficient data structures for map representation
- Adaptive processing to balance quality and speed

### Accuracy vs. Speed Trade-offs
- **High accuracy**: More features, more complex matching, lower frame rate
- **High speed**: Fewer features, faster algorithms, less accuracy
- **Adaptive**: Adjust parameters based on scene complexity

## Evaluation Metrics

### Localization Accuracy
- **Absolute trajectory error (ATE)**: Difference between estimated and ground truth trajectories
- **Relative pose error (RPE)**: Error in relative motion estimates

### Mapping Quality
- **Map completeness**: Coverage of the environment
- **Map consistency**: Absence of conflicting geometric constraints

## Best Practices

### Environmental Considerations
- Ensure sufficient visual features (texture) in the environment
- Avoid repetitive patterns that cause ambiguity
- Consider lighting conditions for consistent feature detection

### Hardware Optimization
- Use stereo cameras for better depth estimation
- Ensure adequate GPU resources for real-time processing
- Consider dedicated compute hardware (e.g., NVIDIA Jetson) for mobile robots

### Algorithm Configuration
- Adjust feature detection parameters based on environment
- Configure loop closure detection to minimize drift
- Set appropriate map representation resolution

## Troubleshooting Common Issues

### Tracking Loss
- **Cause**: Feature-poor environments (white walls, sky)
- **Solution**: Use additional sensors (IMU) for temporary tracking

### Drift Accumulation
- **Cause**: Inaccurate motion estimation over time
- **Solution**: Enable loop closure detection

### Performance Degradation
- **Cause**: High computational requirements
- **Solution**: Reduce image resolution or processing frequency

## Real-World Applications

### Robotics
- Autonomous navigation in unknown environments
- Inspection and mapping of infrastructure
- Search and rescue operations

### Augmented Reality
- Real-time 3D scene understanding
- Virtual object placement in physical environments

## Resources and Further Learning

- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/released/packages/visual_slam/index.html)
- [Visual SLAM Tutorial](https://nvidia-isaac-ros.github.io/released/tutorials/visual_slam/index.html)
- [VSLAM Research Papers](https://research.nvidia.com/labs/toronto-ai/)
- [ROS Navigation Stack](http://navigation.ros.org/)

## Summary
Visual SLAM is a powerful technology that enables robots to navigate autonomously in unknown environments. Isaac ROS provides optimized implementations that leverage NVIDIA GPU hardware for real-time performance.

## Exercise
1. Set up Isaac ROS Visual SLAM in Isaac Sim
2. Navigate a robot through a simple environment
3. Save the map created by SLAM
4. Use the map for navigation to a new goal