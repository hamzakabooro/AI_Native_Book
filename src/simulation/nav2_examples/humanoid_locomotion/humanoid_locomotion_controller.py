#!/usr/bin/env python3
"""
Humanoid Locomotion Examples
This script demonstrates humanoid locomotion patterns and control for navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


class HumanoidLocomotionController(Node):
    """
    Controller for humanoid locomotion during navigation
    """
    def __init__(self):
        super().__init__('humanoid_locomotion_controller')
        
        # Create publishers for locomotion control
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        self.step_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/step_commands',
            10
        )
        
        # Subscription for navigation goals
        self.nav_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.nav_goal_callback,
            10
        )
        
        # Timer for locomotion control loop
        self.control_timer = self.create_timer(0.05, self.locomotion_control_loop)  # 20Hz
        
        # Locomotion state
        self.current_pose = None
        self.target_pose = None
        self.locomotion_mode = "standing"  # standing, walking, turning
        self.step_phase = 0  # For gait pattern
        self.gait_params = {
            'step_length': 0.3,  # meters
            'step_height': 0.05,  # meters
            'step_duration': 0.8,  # seconds
            'step_frequency': 1.0,  # Hz
            'max_linear_vel': 0.5,  # m/s
            'max_angular_vel': 0.5  # rad/s
        }
        
        self.get_logger().info('Humanoid Locomotion Controller initialized')
    
    def nav_goal_callback(self, msg):
        """
        Handle navigation goal
        """
        self.target_pose = msg.pose
        self.get_logger().info(f'Received navigation goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        
        # Switch to walking mode
        self.locomotion_mode = "walking"
    
    def locomotion_control_loop(self):
        """
        Main locomotion control loop
        """
        if self.locomotion_mode == "walking":
            # Generate step pattern for walking
            self._generate_walking_pattern()
        elif self.locomotion_mode == "turning":
            # Generate step pattern for turning
            self._generate_turning_pattern()
        elif self.locomotion_mode == "standing":
            # Keep robot standing
            self._generate_standing_pattern()
    
    def _generate_walking_pattern(self):
        """
        Generate walking pattern for humanoid
        """
        # Calculate required velocity based on target
        if self.target_pose and self.current_pose:
            dx = self.target_pose.position.x - self.current_pose.position.x
            dy = self.target_pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # If close to target, slow down
            if distance < 0.5:
                self.locomotion_mode = "standing"
                self.get_logger().info('Reached target, switching to standing mode')
                return
        
        # Generate step commands
        step_cmd = Float64MultiArray()
        
        # Simple bipedal gait - alternating steps
        current_time = self.get_clock().now().nanoseconds / 1e9
        phase = (current_time / self.gait_params['step_duration']) % 2
        
        # Left leg command (phase 0-1) or right leg command (phase 1-2)
        if 0 <= phase < 1:  # Left leg step
            step_cmd.data = [
                self.gait_params['step_length'] / 2,  # Forward position
                0.1,  # Lateral offset for left foot
                self.gait_params['step_height'],  # Vertical lift
                0.0,  # Yaw/Pitch/Roll adjustments
            ]
        else:  # Right leg step
            step_cmd.data = [
                self.gait_params['step_length'] / 2,  # Forward position
                -0.1,  # Lateral offset for right foot
                self.gait_params['step_height'],  # Vertical lift
                0.0,  # Yaw/Pitch/Roll adjustments
            ]
        
        # Publish step command
        self.step_cmd_pub.publish(step_cmd)
        
        # Also publish joint state for demonstration
        joint_state = JointState()
        joint_state.name = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'torso_joint'
        ]
        
        # Calculate joint angles based on gait phase
        t = self.step_phase
        
        # Hip joints - forward/backward movement
        left_hip = 0.1 * math.sin(2 * math.pi * t)
        right_hip = 0.1 * math.sin(2 * math.pi * t + math.pi)  # Opposite phase
        
        # Knee joints - bending to clear ground
        left_knee = 0.2 * max(0, math.sin(2 * math.pi * t))
        right_knee = 0.2 * max(0, math.sin(2 * math.pi * t + math.pi))
        
        # Ankle joints - keep feet level
        left_ankle = -0.05 * math.sin(2 * math.pi * t)
        right_ankle = -0.05 * math.sin(2 * math.pi * t + math.pi)
        
        # Torso - maintain balance
        torso = 0.05 * math.sin(2 * math.pi * t)  # Slight sway for balance
        
        joint_state.position = [
            left_hip, left_knee, left_ankle,
            right_hip, right_knee, right_ankle,
            torso
        ]
        
        # Update step phase
        self.step_phase += 0.05 * self.gait_params['step_frequency']
        if self.step_phase > 1.0:
            self.step_phase = 0.0
        
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_footprint'
        
        self.joint_cmd_pub.publish(joint_state)
    
    def _generate_turning_pattern(self):
        """
        Generate turning pattern for humanoid
        """
        # Turning pattern - stepping in place with different amplitudes
        step_cmd = Float64MultiArray()
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        phase = (current_time / self.gait_params['step_duration']) % 2
        
        # Turn by differentially stepping
        if 0 <= phase < 1:  # Left step
            step_cmd.data = [
                0.05,   # Smaller forward step
                0.15,   # Larger lateral offset for turn
                0.05,   # Small lift
                0.1     # Ankle adjustment for turning
            ]
        else:  # Right step
            step_cmd.data = [
                0.1,    # Larger forward step
                -0.05,  # Smaller lateral offset
                0.05,   # Small lift
                -0.1    # Ankle adjustment for turning
            ]
        
        self.step_cmd_pub.publish(step_cmd)
    
    def _generate_standing_pattern(self):
        """
        Generate standing pattern for humanoid
        """
        # Standing pattern - minimal movement
        step_cmd = Float64MultiArray()
        step_cmd.data = [0.0, 0.0, 0.0, 0.0]  # No step movement
        self.step_cmd_pub.publish(step_cmd)
        
        # Publish neutral joint positions
        joint_state = JointState()
        joint_state.name = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'torso_joint'
        ]
        
        # Neutral standing position
        joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_footprint'
        
        self.joint_cmd_pub.publish(joint_state)
    
    def set_locomotion_mode(self, mode):
        """
        Set the current locomotion mode
        """
        if mode in ["standing", "walking", "turning"]:
            self.get_logger().info(f'Switching to locomotion mode: {mode}')
            self.locomotion_mode = mode
        else:
            self.get_logger().error(f'Invalid locomotion mode: {mode}')


def main(args=None):
    rclpy.init(args=args)
    
    locomotion_controller = HumanoidLocomotionController()
    
    try:
        rclpy.spin(locomotion_controller)
    except KeyboardInterrupt:
        locomotion_controller.get_logger().info('Humanoid locomotion controller stopped')
    finally:
        locomotion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()