# Sensor Simulation in Gazebo

## Overview
This module covers simulating various robot sensors in Gazebo, including LiDAR, depth cameras, and IMU sensors. You'll learn how to configure these sensors in your robot model, understand their simulated behavior, and validate the output data.

## Learning Objectives
- Understand different types of sensors available in Gazebo
- Configure LiDAR, camera, and IMU sensors on your robot model
- Validate sensor outputs in simulation
- Use sensor data in ROS 2 applications

## Types of Sensors in Gazebo

### Range Sensors (LiDAR)
LiDAR sensors simulate time-of-flight distance measurements. They can be 2D (scanning in a plane) or 3D (multiple scanning planes). In Gazebo, LiDAR sensors are implemented as ray sensors.

### Camera Sensors
Camera sensors simulate both RGB and depth cameras:
- RGB cameras provide color images
- Depth cameras provide distance information for each pixel
- Both can be configured with different resolutions, fields of view, and noise characteristics

### IMU Sensors
Inertial Measurement Unit (IMU) sensors provide:
- Linear acceleration measurements
- Angular velocity measurements
- Orientation information (if fused internally)

## Configuring Sensors in URDF

### Adding a LiDAR Sensor
Here's how to add a 2D LiDAR sensor to your robot:

```xml
<!-- LiDAR Link -->
<link name="lidar_link">
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

<!-- Mount the LiDAR to the base -->
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.1"/>
</joint>
```

And the corresponding Gazebo plugin:

```xml
<gazebo reference="lidar_link">
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
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### Adding a Depth Camera
Here's how to add a depth camera to your robot:

```xml
<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<!-- Mount the camera -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
</joint>
```

And the corresponding Gazebo plugin:

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
        <remapping>depth/image_raw:=depth/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### Adding an IMU Sensor
Here's how to add an IMU sensor:

```xml
<!-- IMU link (often co-located with base_link or another reference frame) -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

And the Gazebo plugin:

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Validating Sensor Outputs

### LiDAR Data Validation
The LiDAR sensor publishes data as `sensor_msgs/LaserScan` messages. You can validate the output by:

1. Check the number of ranges matches expected samples
2. Verify ranges are within specified min/max values
3. Ensure the time_increment is reasonable
4. Validate that objects in the environment appear in the scan data

```bash
# View LiDAR data
ros2 topic echo /robot/scan sensor_msgs/msg/LaserScan
```

### Camera Data Validation
Camera sensors publish both RGB and depth data. You can visualize this using:

```bash
# View camera feed
ros2 run image_view image_view --ros-args --remap image:=/robot/camera/image_raw

# View depth data
ros2 run image_view image_view --ros-args --remap image:=/robot/depth/image_raw
```

### IMU Data Validation
IMU sensors publish `sensor_msgs/Imu` messages. Validate by checking:

1. Acceleration in z-axis when robot is upright (should be ~9.8 m/s²)
2. Angular velocities near 0 when robot is static
3. Orientation is consistent with robot's physical orientation

```bash
# View IMU data
ros2 topic echo /robot/imu sensor_msgs/msg/Imu
```

## Troubleshooting Common Issues

### Sensor Not Publishing Data
- Check if the Gazebo plugin is correctly loaded
- Verify namespaces in the plugin configuration
- Ensure the simulation is running

### Incorrect Sensor Values
- Check sensor configuration parameters (min/max ranges, FOV, etc.)
- Verify sensor mounting position and orientation
- Look for noise parameters that might affect values

### Performance Issues
- Reduce the update rate for sensors not requiring high frequency
- Lower the resolution of camera sensors if not needed
- Limit the number of sensors if performance is critical

## Best Practices

1. **Realistic Noise**: Add appropriate noise models to make simulation more realistic
2. **Matching Real Hardware**: Configure sensors to match the specifications of real hardware
3. **Validation**: Always verify simulated sensor data matches expectations
4. **Performance**: Balance sensor fidelity with simulation performance
5. **Documentation**: Keep detailed records of sensor configurations for reproducibility

## Example Robot with Sensors
Let's enhance our basic robot model to include sensors:

```xml
<?xml version="1.0"?>
<robot name="sensor_robot">
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

  <!-- Wheels -->
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

  <!-- LiDAR -->
  <link name="lidar_link">
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

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joints -->
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

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- LiDAR Sensor Plugin -->
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <always_on>true</always_on>
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
          <namespace>/sensor_robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Camera Sensor Plugin -->
  <gazebo reference="camera_link">
    <sensor type="depth" name="depth_camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
        <ros>
          <namespace>/sensor_robot</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
          <remapping>depth/image_raw:=depth/image_raw</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Summary
Sensor simulation in Gazebo is a critical component of robotics development. By properly configuring LiDAR, camera, and IMU sensors, you can develop and test perception algorithms in a safe, controlled environment before deploying to real hardware.

## Resources
- [Gazebo Sensors Documentation](http://gazebosim.org/tutorials/?tut=ros2_sensors)
- [ROS 2 Sensor Message Types](http://docs.ros.org/en/rolling/api/sensor_msgs/html/index.html)
- [Gazebo ROS Sensor Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)