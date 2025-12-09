# URDF/SDF Modeling for Robot Customization

## Overview
This guide covers the fundamentals of creating and customizing robot models using URDF (Unified Robot Description Format) and SDF (Simulation Description Format). Understanding these formats is essential for creating digital twins in Gazebo and other simulation environments.

## URDF vs SDF: When to Use Each

### URDF (Unified Robot Description Format)
- Used primarily for representing robot kinematics and dynamics
- Commonly used with ROS and MoveIt
- Defines robot structure: links, joints, and their relationships
- Limited physics simulation parameters

### SDF (Simulation Description Format)
- Used specifically for simulation environments like Gazebo
- Supports more detailed physics simulation
- Defines materials, sensors, plugins, and complex environments
- Can include multiple robots and environments in one file

## URDF Structure

### Links
A link represents a rigid body in the robot. Each link has:
- Visual: How the link appears (geometry, material, origin)
- Collision: How the link interacts physically (geometry, origin)
- Inertial: Physical properties for dynamics simulation (mass, moments of inertia)

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="1.0 1.0 1.0"/>
    </geometry>
    <material name="color_name">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="1.0 1.0 1.0"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints
Joints connect links and define their motion constraints:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14159" upper="3.14159" effort="100" velocity="1"/>
</joint>
```

Joint types include:
- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No motion between links
- `floating`: 6 DOF unconstrained motion
- `planar`: Motion on a plane

## Practical Example: Customizing a Wheeled Robot

Let's customize our basic robot model by adding sensors and adjusting physical properties:

```xml
<?xml version="1.0"?>
<robot name="custom_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel with improved physical properties -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- LiDAR Sensor -->
  <link name="lidar_sensor">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="silver">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0002" ixy="0.0" ixz="0.0" iyy="0.0002" iyz="0.0" izz="0.0004"/>
    </inertial>
  </link>

  <!-- Wheel Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.2 -0.175 -0.075" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.2 0.175 -0.075" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Sensor Mount -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_sensor"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="lidar_sensor">
    <material>Gazebo/Silver</material>
  </gazebo>

  <!-- LiDAR Sensor Plugin -->
  <gazebo reference="lidar_sensor">
    <sensor type="ray" name="lidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/custom_robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Advanced Customization Techniques

### 1. Using Xacro for Complex Models
Xacro (XML Macros) allows you to create more maintainable and parameterized URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="custom_robot_xacro">

  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_size_x" value="0.5" />
  <xacro:property name="base_size_y" value="0.3" />
  <xacro:property name="base_size_z" value="0.15" />

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix joint_pos_x joint_pos_y">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.004"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${joint_pos_x} ${joint_pos_y} -0.075" rpy="1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_size_x} ${base_size_y} ${base_size_z}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_size_x} ${base_size_y} ${base_size_z}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Use wheel macro -->
  <xacro:wheel prefix="left" joint_pos_x="0.2" joint_pos_y="-0.175" />
  <xacro:wheel prefix="right" joint_pos_x="0.2" joint_pos_y="0.175" />

</robot>
```

### 2. Adding Transmission Elements
For ROS control, you need to define transmissions:

```xml
<transmission name="left_wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Validation and Testing

### Validating URDF Models
Use the check_urdf tool to verify your model:
```bash
check_urdf /path/to/your/robot.urdf
```

### Testing in Gazebo
1. Load your model in Gazebo
2. Verify that all joints move as expected
3. Check that collision meshes work properly
4. Validate that sensors publish data correctly

## Best Practices for Model Customization

1. **Start Simple**: Begin with basic shapes and add complexity gradually
2. **Correct Inertial Properties**: Ensure mass and inertia values are realistic
3. **Appropriate Collision Meshes**: Use simpler meshes for collision than visualization
4. **Realistic Joint Limits**: Set limits that reflect the physical constraints
5. **Parameterize with Xacro**: Use xacro for complex, configurable models
6. **Test Iteratively**: Validate at each step of development

## Troubleshooting Common Issues

- **"Joint not found" errors**: Verify joint names and parent/child relationships
- **Robot falls through the ground**: Check collision properties and inertial parameters
- **Simulation instability**: Verify mass and inertia values are reasonable
- **Sensors not publishing**: Check Gazebo plugins and ROS topic configurations

## Summary
Customizing robot models using URDF and SDF is a key skill for robotics simulation. By understanding the structure of these files and following best practices, you can create accurate digital twins of real robots and test their behavior in simulation.

## Resources
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo Model Tutorial](http://gazebosim.org/tutorials?tut=build_robot)
- [Xacro Documentation](http://wiki.ros.org/xacro)