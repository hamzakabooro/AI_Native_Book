# Gazebo Basics for Digital Twin Creation

## Overview
This module introduces the fundamentals of Gazebo simulation for creating digital twins. Gazebo is a powerful robotics simulation environment that provides accurate physics simulation, high-quality graphics, and support for various sensors.

## Learning Objectives
- Understand Gazebo's role in robotics simulation
- Learn to create a basic simulation environment
- Set up a robot model in Gazebo
- Run a physics simulation with realistic properties

## Prerequisites
- ROS 2 Humble installed on Ubuntu 22.04
- Basic understanding of URDF (Unified Robot Description Format)
- Familiarity with ROS 2 concepts (nodes, topics, services)

## Setting Up Your First Gazebo Simulation

### 1. Launching Gazebo with a Basic Robot
To launch your first simulation, use the following command:

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws
source install/setup.bash

# Launch the basic simulation
ros2 launch basic_digital_twin basic_sim.launch.py
```

This command will:
- Start Gazebo with an empty world
- Load your robot model from the URDF file
- Publish robot state information to RViz for visualization

### 2. Understanding the Simulation Environment
Once you launch the simulation, you'll see:
- The Gazebo GUI with your robot model
- A physics-enabled environment
- Real-time simulation of robot dynamics

### 3. Interacting with the Simulation
You can interact with your robot in simulation by publishing messages to ROS 2 topics. For example, to control the robot's movement, you can send velocity commands to the `/cmd_vel` topic.

## Physics Simulation Fundamentals

### Gravity and Forces
By default, Gazebo simulates Earth-like gravity (9.8 m/s²). You can modify this in your world file or through Gazebo services.

### Collision Detection
Gazebo uses collision meshes to detect when objects interact. These are typically simplified versions of visual meshes for performance.

### Sensors in Simulation
Gazebo can simulate various sensors including:
- LiDAR (2D and 3D)
- Cameras (RGB, depth, fisheye)
- IMU (Inertial Measurement Unit)
- Force/Torque sensors
- GPS
- Joint position/velocity sensors

## Best Practices

1. **Model Complexity**: Start with simple models and gradually increase complexity.
2. **Physics Parameters**: Verify that your model's mass, friction, and inertial properties are realistic.
3. **Simulation Speed**: Monitor real-time factor (RTF) to ensure efficient simulation.
4. **Visualization**: Use RViz alongside Gazebo to visualize sensor data and robot state.

## Troubleshooting Common Issues

- **Robot falls through the ground**: Check your collision meshes and inertial properties in the URDF
- **Simulation runs too slowly**: Reduce model complexity or adjust physics parameters
- **Joints behave strangely**: Verify joint limits, types, and position in the URDF

## Next Steps
After mastering these basics, you'll be ready to move on to sensor simulation and more complex robot models in the subsequent modules.

## References
- [Official Gazebo Documentation](http://gazebosim.org/)
- [ROS 2 Gazebo Integration Guide](https://classic.gazebosim.org/tutorials?tut=ros2_overview)
- [URDF Documentation](http://wiki.ros.org/urdf)