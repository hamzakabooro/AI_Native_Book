# Use ROS 2 Humble Hawksbill base image
FROM osrf/ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install additional packages needed for robotics simulation
RUN apt-get update && apt-get install -y \
    # Gazebo and related packages
    ignition-harmonic \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-gazebo-ros2-control-demos \
    # Navigation
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # Perception
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-cv-bridge \
    # Additional tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    git \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS environment
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Create workspace directory
WORKDIR /ws
RUN mkdir -p /ws/src

# Copy the workspace files (if any exist)
COPY . /ws/src/

# Install dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select basic_digital_twin sensor_simulation

# Source the workspace
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]