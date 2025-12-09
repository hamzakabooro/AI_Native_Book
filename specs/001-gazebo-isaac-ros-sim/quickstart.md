# Quickstart: Gazebo-Isaac-ROS Simulation

## Overview

This quickstart guide will help you set up the basic environment for the Gazebo-Isaac-ROS simulation modules. By the end of this guide, you'll have:

1. A working ROS 2 Humble installation on Ubuntu 22.04
2. Gazebo simulation environment
3. Basic robot model running in simulation
4. Understanding of the project structure

## Prerequisites

- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- Minimum 8GB RAM (16GB recommended)
- Multi-core processor (4+ cores recommended)
- NVIDIA GPU (for Isaac Sim, optional for basic Gazebo)
- Internet connection for package downloads

## Step 1: Install ROS 2 Humble

```bash
# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS 2 Humble packages
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source the ROS 2 installation
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 2: Install Gazebo

```bash
# Install Gazebo Garden (or Harmonic if available)
sudo apt install -y ros-humble-gazebo-*
sudo apt install -y gazebo

# Verify installation
gazebo --version
```

## Step 3: Set Up Your Workspace

```bash
# Create a workspace for the simulation examples
mkdir -p ~/ros2_simulation_ws/src
cd ~/ros2_simulation_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty, this sets up the structure)
colcon build --symlink-install

# Source the workspace
source install/local_setup.bash
```

## Step 4: Clone and Build Simulation Examples

```bash
# Navigate to your workspace source directory
cd ~/ros2_simulation_ws/src

# Clone the simulation examples (replace with actual repository when available)
# For now, we'll create basic example packages
cd ~/ros2_simulation_ws

# Build the workspace again
colcon build --symlink-install
source install/local_setup.bash
```

## Step 5: Run Your First Simulation

Let's create a simple robot model and run it in Gazebo:

```bash
# Create a simple URDF robot (we'll create a differential drive robot)
mkdir -p ~/ros2_simulation_ws/src/basic_robot_description/urdf
cat > ~/ros2_simulation_ws/src/basic_robot_description/urdf/basic_robot.urdf << EOF
<?xml version="1.0"?>
<robot name="basic_robot">
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
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
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
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
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
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.2 -0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.2 0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
EOF

# Create a launch file for the simulation
mkdir -p ~/ros2_simulation_ws/src/basic_robot_description/launch
cat > ~/ros2_simulation_ws/src/basic_robot_description/launch/basic_sim.launch.py << EOF
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'basic_robot_description'
    world_file_name = 'empty_world.world'  # Using empty world by default

    # Path to this package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_basic_robot = get_package_share_directory(package_name)

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(
                os.path.join(pkg_basic_robot, 'urdf', 'basic_robot.urdf')
            ).read()
        }]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'basic_robot'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])

def get_package_share_directory(package_name):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package_name)
EOF

# Create the package.xml file
cat > ~/ros2_simulation_ws/src/basic_robot_description/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>basic_robot_description</name>
  <version>0.0.1</version>
  <description>Basic robot model for simulation examples</description>
  <maintainer email="student@todo.todo">student</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>gazebo_ros_pkgs</depend>
  <depend>robot_state_publisher</depend>

  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > ~/ros2_simulation_ws/src/basic_robot_description/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project(basic_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/\${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF
```

## Step 6: Build and Run the Simulation

```bash
# Navigate to the workspace
cd ~/ros2_simulation_ws

# Build the new package
colcon build --packages-select basic_robot_description

# Source the workspace
source install/local_setup.bash

# Run the simulation
ros2 launch basic_robot_description basic_sim.launch.py
```

You should now see Gazebo launch with a simple robot model. If Gazebo doesn't start or crashes, make sure your system meets the requirements and that you have proper graphics drivers installed.

## Next Steps

1. **Module 2 - Digital Twin**: Explore advanced Gazebo features, sensor simulation, and environment modeling
2. **Module 3 - AI Robot Brain**: Learn about Isaac Sim and Isaac ROS for advanced perception
3. **Navigation**: Implement Nav2 path planning for your robot

## Troubleshooting

**Gazebo fails to start:**
- Check your graphics drivers are properly installed
- Try running: `export LIBGL_ALWAYS_SOFTWARE=1` before launching (software rendering)
- Verify your user has proper access to X11: `xhost +local:docker`

**Robot doesn't appear in Gazebo:**
- Check that the URDF file is valid: `check_urdf ~/ros2_simulation_ws/src/basic_robot_description/urdf/basic_robot.urdf`
- Verify the launch file has proper paths
- Look at the console output for error messages

**Performance issues:**
- Reduce physics update rate in Gazebo world config
- Close other applications to free up resources
- Ensure sufficient RAM and CPU resources are available

## Safety Note

Always follow robotics safety protocols when transitioning from simulation to real hardware. Simulation environments don't account for all real-world variables such as sensor noise, actuator delays, and unexpected environmental conditions.