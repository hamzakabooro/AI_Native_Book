# Nav2 Path Planning for Humanoid Robots

## Overview
This module covers implementing navigation capabilities for humanoid robots using Nav2 (Navigation Stack 2) path planning in simulation. You'll learn to create navigation graphs, plan paths, and execute navigation for humanoid robots in simulated environments.

## Learning Objectives
- Understand Nav2 architecture and components
- Configure Nav2 for humanoid robot navigation
- Create and optimize navigation graphs
- Plan and execute navigation for humanoid locomotion
- Evaluate navigation performance in simulation

## Nav2 Architecture

### Core Components
Nav2 is built around a behavior tree architecture with these main components:

- **Navigation Server**: Main entry point for navigation actions
- **Map Server**: Provides static and costmap representations
- **Local Planner**: Handles immediate obstacle avoidance and path following
- **Global Planner**: Computes optimal paths from start to goal
- **Behavior Trees**: Define navigation behaviors and control flow
- **Lifecycle Manager**: Manages state transitions of navigation components

### Navigation Pipeline
The navigation process follows this pipeline:
1. **Goal Input**: User specifies navigation goal
2. **Map Loading**: Static and cost maps are loaded
3. **Global Planning**: Path from start to goal is calculated
4. **Local Planning**: Robot moves along path, avoiding obstacles
5. **Recovery**: If stuck, recovery behaviors are executed

## Configuring Nav2 for Humanoid Robots

### Robot Configuration Considerations
When configuring Nav2 for humanoid robots, consider:

1. **Footprint**: Unlike wheeled robots, humanoids have unique footprints and turning characteristics
2. **Kinematics**: Humanoid robots have complex kinematics and balance constraints
3. **Dynamic Properties**: Different speed and acceleration capabilities than wheeled robots
4. **Step Planning**: For complex terrain, step planning might be necessary

### Configuration File Structure
```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: False
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_interval: 0.5
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_footprint"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/bt_xml_v0/nav_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "package://nav2_bt_navigator/bt_xml_v0/nav_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_rclcpp_node_bt_xml: "package://nav2_bt_navigator/bt_xml_v0/nav_to_pose_w_replanning_and_recovery_rclcpp.xml"
    plugin_lib_names:
      - "BackUp"
      - "Spin"
      - "Wait"
      - "AssistedTeleop"
      - "ClearEntireCostmap"
      - "RemovePassedGoals"
      - "PlannerSelector"
      - "ControllerSelector"
      - "GoalCheckerSelector"
      - "IsGoalReached"
      - "IsPathValid"
      - "IsGoalValid"
      - "IsBatteryLow"
      - "IsBatteryOperational"
      - "IsBatteryCharging"
      - "IsStuck"
      - "IsControllerActive"
      - "IsPlannerActive"
      - "IsRecoveryActive"
      - "IsGoalReached"
      - "IsPathValid"
      - "TruncatePath"
      - "TruncatePathLocal"
      - "ComputePathToPose"
      - "ComputePathThroughPoses"
      - "FollowPath"
      - "SmoothPath"
      - "ComputeVelocityCommands"
      - "CorrectPath"
      - "WaitAtGoal"
      - "OnFirstPass"
      - "BackUpAndStop"
      - "AvoidCollision"
      - "IsBatteryCharging"
      - "IsBatteryOperational"
      - "IsBatteryLow"
      - "IsControllerActive"
      - "IsGoalReached"
      - "IsGoalValid"
      - "IsPathValid"
      - "IsPlannerActive"
      - "IsRecoveryActive"
      - "IsStuck"
      - "IsTrajectoryValid"
      - "RecoveryNode"
      - "ControllerSelector"
      - "GoalCheckerSelector"
      - "PlannerSelector"
      - "TransformFootprint"
      - "ChangeGoal"
      - "GetPath"
      - "GetRootPose"
      - "IsGoalReached"
      - "IsGoalValid"
      - "IsPathValid"
      - "IsRecoveryActive"
      - "IsStuck"
      - "FollowPath"
      - "ComputePathToPose"
      - "ComputePathThroughPoses"
      - "SmoothPath"
      - "ComputeVelocityCommands"
      - "WaitUntilReady"
      - "BackUp"
      - "Spin"
      - "Wait"
      - "AssistedTeleop"
      - "BackupSpinRecovery"
      - "WaitCancel"
      - "WaitAtGoal"
      - "OnFirstPass"
      - "BackUpAndStop"
      - "AvoidCollision"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]
    
    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    # Goal checker parameters
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    
    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      # Inner controller for handling actual path following
      inner_controller:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        desired_linear_vel: 0.5
        lookahead_dist: 0.6
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        lookahead_time: 1.5
        rotate_to_heading_angular_vel: 1.0
        use_velocity_scaled_lookahead_dist: false
        min_approach_linear_velocity: 0.12
        approach_velocity_scaling_dist: 0.6
        use_approach_vel_scaling: true
        max_allowed_time_to_collision_up_to_carrot: 1.0
        use_regulated_linear_velocity_scaling: true
        use_cost_regulated_linear_velocity_scaling: true
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_rotation_shim: true
        rotation_shim:
          plugin: "nav2_controller::SimpleProgressChecker"
          min_rotational_vel: 0.4
          time_threshold_to_rotate: 1.0
          max_hysteresis_radius: 0.3
          min_hysteresis_radius: 0.15
          hysteresis_factor: 2.0
          xy_goal_tolerance: 0.025

global_costmap:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_footprint"
    update_frequency: 1.0
    publish_frequency: 1.0
    width: 40
    height: 40
    resolution: 0.05
    origin_x: -20.0
    origin_y: -20.0
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55
    always_send_full_costmap: True

local_costmap:
  ros__parameters:
    use_sim_time: True
    global_frame: "odom"
    robot_base_frame: "base_footprint"
    update_frequency: 5.0
    publish_frequency: 2.0
    width: 6
    height: 6
    resolution: 0.05
    origin_x: -3.0
    origin_y: -3.0
    rolling_window: True
    plugins: ["obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55
    always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    velocity_threshold: 0.0
    velocity_scale: 1.0
    accel_lim_v: 2.5
    accel_lim_theta: 3.2
    decel_lim_v: -2.5
    decel_lim_theta: -3.2
    odom_topic: "odom"
    robot_base_frame: "base_footprint"
    cmd_vel_topic: "cmd_vel"
    smoothed_cmd_vel_topic: "cmd_vel_smoothed"

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

## Creating Navigation Graphs

### Waypoint Navigation
For humanoid robots, navigation graphs often involve planning through a series of waypoints:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
import math


class NavigationGraphManager(Node):
    """
    Manages navigation graphs for humanoid robots
    """
    def __init__(self):
        super().__init__('navigation_graph_manager')
        
        # Action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )
        
        # Store predefined waypoints
        self.waypoints = {}
        
        self.get_logger().info('Navigation Graph Manager initialized')
    
    def create_navigation_graph(self, graph_name, waypoints_list):
        """
        Create a navigation graph with specified waypoints
        """
        self.waypoints[graph_name] = waypoints_list
        self.get_logger().info(f'Created navigation graph: {graph_name} with {len(waypoints_list)} waypoints')
    
    def navigate_through_graph(self, graph_name):
        """
        Navigate through all waypoints in a graph
        """
        if graph_name not in self.waypoints:
            self.get_logger().error(f'Navigation graph {graph_name} not found')
            return False
            
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create goal message with all waypoints
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints[graph_name]
        
        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_result_callback)
        
        return True
    
    def navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result()
        if result:
            self.get_logger().info('Navigation completed successfully')
        else:
            self.get_logger().error('Navigation failed')
    
    def add_waypoint_to_graph(self, graph_name, pose):
        """
        Add a waypoint to an existing graph
        """
        if graph_name not in self.waypoints:
            self.waypoints[graph_name] = []
        
        self.waypoints[graph_name].append(pose)
        self.get_logger().info(f'Added waypoint to graph {graph_name}')


def main(args=None):
    rclpy.init(args=args)
    
    nav_graph_manager = NavigationGraphManager()
    
    # Example: Create a simple square path
    square_waypoints = []
    
    # Add waypoints for a square path
    for i, (x, y, theta) in enumerate([
        (1.0, 0.0, 0.0),
        (1.0, 1.0, math.pi/2),
        (0.0, 1.0, math.pi),
        (0.0, 0.0, -math.pi/2)
    ]):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = nav_graph_manager.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import tf_transformations
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        square_waypoints.append(pose)
    
    # Create the navigation graph
    nav_graph_manager.create_navigation_graph('square_path', square_waypoints)
    
    # Navigate through the graph (uncomment to execute)
    # nav_graph_manager.navigate_through_graph('square_path')
    
    try:
        rclpy.spin(nav_graph_manager)
    except KeyboardInterrupt:
        pass
    finally:
        nav_graph_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

## Humanoid Locomotion Considerations

### Balance and Stability
Humanoid robots have unique challenges for navigation:

- **Center of Mass**: Must be carefully managed during movement
- **Step Planning**: Requires planning for each foot placement
- **Dynamic Walking**: Different from static wheeled robot navigation
- **Terrain Adaptation**: Ability to handle stairs, slopes, and uneven terrain

### Gait Patterns
Different walking patterns for various situations:
- **Static Gait**: Feet always in contact with ground
- **Dynamic Gait**: Some air time between steps
- **Narrow Walking**: For tight spaces
- **Backward Walking**: When needed for specific tasks

## Launching Nav2 for Humanoid Robots

### Basic Launch Command
```bash
# Launch Nav2 with custom configuration for humanoid
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/humanoid_nav2_params.yaml
```

### Simulation Integration
When using with Isaac Sim or Gazebo:
```bash
# Launch navigation with simulation time
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  map:=/path/to/your/map.yaml
```

## Path Planning Algorithms in Nav2

### Global Planners
- **NavFn**: Fast-marching method for any-angle path planning
- **Global Planner**: A* implementation for grid-based planning
- **Theta* Planner**: Any-angle path planning with line-of-sight optimization

### Local Planners
- **DWB (Dynamic Window Approach)**: Velocity-based obstacle avoidance
- **TEB (Timed Elastic Band)**: Elastic band optimization for trajectory planning
- **RPP (Rotation Prone Planner)**: Specialized for rotation-heavy scenarios

## Navigation Recovery Behaviors

### Common Recovery Actions
1. **Back Up**: Move robot backward if stuck
2. **Spin**: Rotate in place to clear local minima
3. **Wait**: Pause to allow dynamic obstacles to move

### Custom Recovery for Humanoids
Humanoid robots may need specialized recovery behaviors:
- **Step Back**: Carefully step backward maintaining balance
- **Side Step**: Move laterally around obstacles
- **Pause and Assess**: Stop and analyze the navigation situation

## Evaluating Navigation Performance

### Metrics for Success
- **Path Efficiency**: How close the path was to optimal
- **Navigation Success Rate**: Percentage of successful navigations
- **Time to Goal**: How long navigation took
- **Safety**: How well collisions were avoided
- **Smoothness**: Quality of path following

### Simulation Testing
Test navigation in simulation before deployment:
```bash
# Automated navigation testing script
ros2 run nav2_system_tests navigation2_tester.py \
  --test-scenario path_through_known_map.yaml \
  --metrics-output navigation_metrics.json
```

## Best Practices for Humanoid Navigation

### Configuration Guidelines
1. **Costmap Settings**: Adjust inflation and obstacle layers for humanoid footprint
2. **Controller Tuning**: Configure linear and angular velocities appropriate for walking
3. **Safety Margins**: Set larger safety margins due to humanoid balance constraints

### Environment Design
- Ensure paths are wide enough for humanoid width
- Avoid steep slopes unless the robot is specifically designed for them
- Include rest points where the robot can safely stand if needed

## Troubleshooting Common Issues

### Navigation Fails to Start
- Check if the robot is localized in the map
- Verify proper transforms between coordinate frames
- Confirm sensor topics are publishing data

### Robot Gets Stuck
- Increase costmap inflation radius
- Adjust controller parameters for better obstacle handling
- Check if the robot's footprint is properly configured

### Erratic Movement
- Lower maximum velocities
- Increase controller frequency
- Check for sensor noise affecting navigation

## Integration with Perception Systems

### Using VSLAM Maps
When using maps generated by Visual SLAM:
```yaml
map_server:
  ros__parameters:
    yaml_filename: "vslam_generated_map.yaml"
    # Additional parameters to handle VSLAM map characteristics
    topic_name: "vslam_map"
    frame_id: "map"
    output: "nav_msgs/OccupancyGrid"
```

### Dynamic Obstacle Avoidance
Combine perception data with navigation:
- Subscribe to object detection topics
- Update costmaps with dynamic obstacles
- Plan paths that account for predicted movements

## Real-World Deployment Considerations

### Mapping for Humanoid Navigation
- Create maps with sufficient resolution for humanoid navigation
- Include areas needed for turning and maneuvering
- Mark areas that are unsafe for humanoid navigation

### Safety Systems
- Implement safety limits on walking speeds
- Add emergency stop capabilities
- Include fall detection and recovery procedures

## Resources and Further Learning

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS Navigation Tutorials](http://wiki.ros.org/navigation/Tutorials)
- [Humanoid Robot Navigation Research](https://www.roml.org/navigation-humanoids)
- [Isaac ROS Navigation Integration](https://nvidia-isaac-ros.github.io/concepts/navigation/index.html)

## Summary
Nav2 provides powerful navigation capabilities that can be adapted for humanoid robots. By understanding the unique requirements of bipedal locomotion and configuring Nav2 appropriately, you can enable autonomous navigation for humanoid robots in both simulation and real-world environments.

## Exercise
1. Set up Nav2 with a humanoid robot model
2. Create a navigation graph with multiple waypoints
3. Navigate through the graph while avoiding obstacles
4. Evaluate navigation performance metrics