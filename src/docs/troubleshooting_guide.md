# Troubleshooting Guide for Gazebo-Isaac-ROS Simulation

## Overview
This guide provides solutions for common issues encountered when working with Gazebo, Isaac Sim, and ROS 2 simulations. This guide covers problems from setup to advanced simulation scenarios.

## Setup and Installation Issues

### ROS 2 Installation Issues
**Problem**: `ros2` command not found after installation
- Verify ROS 2 installation: `echo $ROS_DISTRO`
- Source the ROS 2 installation: `source /opt/ros/humble/setup.bash`
- Add to your `.bashrc`: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`

**Problem**: Package dependencies not found
- Update package lists: `sudo apt update`
- Install missing packages: `sudo apt install <package-name>`
- Run rosdep: `rosdep install --from-paths src --ignore-src -r -y`

### Gazebo Installation Problems
**Problem**: Gazebo fails to start with graphics errors
```bash
# Check graphics drivers
nvidia-smi  # For NVIDIA
# or
glxinfo | grep "OpenGL renderer"

# Set software rendering if needed
export LIBGL_ALWAYS_SOFTWARE=1
```

**Problem**: Gazebo GUI doesn't appear
- Check X11 forwarding if running in Docker
- Ensure DISPLAY variable is set: `echo $DISPLAY`
- Grant X11 access: `xhost +local:docker`

### Isaac Sim Installation Issues
**Problem**: Isaac Sim fails to start
- Verify NVIDIA GPU: `nvidia-smi`
- Check CUDA installation: `nvcc --version`
- Verify Isaac Sim prerequisites are met
- Ensure sufficient disk space (Isaac Sim requires significant storage)

## Simulation Runtime Issues

### Physics Simulation Issues
**Problem**: Robot falls through the ground or objects
- Check that collision geometry exists for the robot links
- Verify inertial properties in the URDF: mass, center of mass, and moments of inertia
- Check that Gazebo plugins are loaded properly

**Problem**: Robot explodes or moves erratically
- Verify that all joint limits and positions are properly defined
- Check that mass values are reasonable (not zero or negative)
- Ensure moments of inertia follow the triangle inequality principle

**Problem**: Simulation runs slowly
- Reduce the complexity of robot models
- Simplify collision and visual meshes
- Adjust physics parameters in the world file:
```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>
  <real_time_update_rate>100</real_time_update_rate>
</physics>
```

### Sensor Simulation Issues
**Problem**: LiDAR sensor not publishing data
- Check that the range values are within min/max bounds
- Verify the sensor is properly attached to a robot link
- Ensure the Gazebo plugin is correctly configured

**Problem**: Camera images appear black
- Verify the camera is properly mounted and orientation is correct
- Check that the resolution and format are supported
- Ensure proper lighting in the scene

**Problem**: IMU values are incorrect
- Verify the sensor mounting position and orientation
- Check that the coordinate frames align with expected conventions
- Validate that the robot is experiencing appropriate acceleration/speed

## ROS 2 Communication Issues

### Topic Connection Problems
**Problem**: Nodes can't connect to topics
- Verify ROS_DOMAIN_ID is consistent across all nodes
- Check network configuration if nodes are on different machines
- Ensure the ROS master is running

**Problem**: Sensor data not reaching processing nodes
- Use `ros2 topic list` to check if topics exist
- Use `ros2 topic echo` to verify data is being published
- Check topic remappings in launch files

### TF Tree Issues
**Problem**: TF transforms not available
- Verify that robot_state_publisher is running
- Check that joint_state_publisher is publishing joint states
- Use `ros2 run tf2_tools view_frames` to visualize the TF tree

## Isaac ROS Specific Issues

### Hardware Acceleration Problems
**Problem**: Isaac ROS components not utilizing GPU
- Check that CUDA and TensorRT are properly installed
- Verify GPU compatibility with Isaac ROS components
- Monitor GPU usage: `nvidia-smi`

**Problem**: Object detection not working
- Verify that detection models are properly loaded
- Check that input images are in the correct format and resolution
- Validate that calibration parameters are correct

## Navigation Issues

### Nav2 Problems
**Problem**: Robot doesn't navigate to goal
- Verify that the robot is properly localized
- Check that the map is loaded and accessible
- Ensure the costmaps are updating properly

**Problem**: Local planner fails to avoid obstacles
- Increase the inflation radius in costmap configuration
- Adjust controller parameters for better obstacle handling
- Verify that the robot's footprint is properly configured

**Problem**: Global planner can't find a path
- Check that the map is properly loaded
- Verify that the start and goal positions are on the map
- Adjust tolerance parameters for goal acceptance

## Performance Optimization

### Simulation Performance
**Problem**: High CPU usage during simulation
- Reduce physics update rate
- Simplify collision meshes
- Limit the number of active sensors

**Problem**: Low simulation speed factor (RTF)
- Balance physics accuracy with performance
- Use appropriate solver settings
- Reduce model complexity where possible

### Isaac Sim Performance
**Problem**: Isaac Sim running slowly
- Check that RTX ray tracing is configured properly
- Adjust quality settings in the renderer
- Monitor GPU memory usage

## Docker and Environment Issues

### Container Problems
**Problem**: Simulation not working inside Docker
- Ensure Docker is run with appropriate privileges
- Check GPU access with `--gpus all`
- Verify X11 forwarding for GUI applications

**Problem**: Shared memory issues in Docker
- Run with `--shm-size=1g` for larger shared memory
- This is important for sensor data transfer

## Safety and Best Practices

### Common Safety Issues
**Problem**: Robot behaves unexpectedly at high speeds
- Implement velocity limits in controllers
- Use safety monitors to prevent dangerous movements
- Test in simulation before any real robot deployment

### Best Practices for Error Prevention
1. **Always test in simulation first**: Before running on real robots
2. **Use version control**: Track all configuration changes
3. **Document configurations**: Keep track of working parameter sets
4. **Use appropriate safety margins**: Especially for physical robots
5. **Validate sensor data**: Ensure data quality before using in control systems

## Debugging Strategies

### Debugging Simulation Issues
1. **Check logs**: Use `ros2 launch` with `--log-level` to increase verbosity
2. **Visualize in RViz**: Confirm transforms and sensor data visually
3. **Use rqt tools**: For real-time monitoring of topics and parameters
4. **Save and replay**: Use ROS bags to replay scenarios

### Debugging ROS Components
```bash
# Monitor node status
ros2 lifecycle list <node_name>

# Get detailed parameter information
ros2 param describe <node_name> <param_name>

# Echo a topic with timestamps
ros2 topic echo --field header.stamp <topic_name>
```

## Common Error Messages and Solutions

### "Could not contact service" errors
- Check that the required service node is running
- Verify that the service name matches exactly
- Check network configuration if nodes are on different machines

### "Transform timeout" errors
- Ensure tf_publisher is running
- Check that the transform tree is connected
- Verify timestamps are synchronized

### "Failed to load Gazebo plugin" errors
- Verify the plugin library path is correct
- Check for missing dependencies
- Ensure the plugin is compatible with the current Gazebo version

## Recovery Procedures

### When the simulation gets stuck
1. Reset the simulation: `gz service -s /world/reset`
2. Restart ROS nodes systematically
3. Clear costmap: `rosservice call /local_costmap/clear_entirely_local_costmap`

### When the robot is in an unstable state
1. Stop the robot: `rostopic pub /cmd_vel geometry_msgs/Twist '{}'`
2. Reset the robot to a known state
3. Re-localize if needed

## Getting Help

### Resources for Further Assistance
- [ROS Answers](https://answers.ros.org/)
- [Gazebo Answers](https://answers.gazebosim.org/)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/)

### When to Seek Help
- After attempting standard troubleshooting steps
- When encountering issues with specific hardware configurations  
- When reproducible bugs are discovered
- When clarification on best practices is needed

## Summary
This troubleshooting guide covers the most common issues encountered when working with the Gazebo-Isaac-ROS simulation framework. Remember to start with basic checks like network connectivity, ROS domain IDs, and proper file paths before diving into more complex solutions.

## Quick Reference Commands
```bash
# Check running ROS nodes
ros2 node list

# Check ROS topics
ros2 topic list

# Check ROS services
ros2 service list

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor CPU and GPU usage
htop  # CPU
nvidia-smi  # GPU
```