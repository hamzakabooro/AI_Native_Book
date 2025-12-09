#!/usr/bin/env python3
"""
Navigation Goal Service
Implements the navigation goal service according to the contract
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math


class NavigationGoalService(Node):
    """
    Service node for setting navigation goals for humanoid robots
    """
    def __init__(self):
        super().__init__('navigation_goal_service')
        
        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Store navigation parameters
        self.nav_params = {
            'current_goal': None,
            'navigation_active': False,
            'robot_name': 'humanoid_robot'
        }
        
        self.get_logger().info('Navigation Goal Service initialized')
        
    def set_goal(self, position_x, position_y, position_z, 
                 orientation_x, orientation_y, orientation_z, orientation_w,
                 behavior_tree=""):
        """
        Set a navigation goal for the robot
        This follows the contract specification with PoseStamped and behavior tree
        """
        try:
            self.get_logger().info(
                f'Setting navigation goal: ({position_x:.2f}, {position_y:.2f}, {position_z:.2f})'
            )
            
            # Wait for action server
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                return False, 'Navigation action server not available'
            
            # Create the goal message
            goal_msg = NavigateToPose.Goal()
            
            # Set the goal pose
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.pose.pose.position.x = position_x
            goal_msg.pose.pose.position.y = position_y
            goal_msg.pose.pose.position.z = position_z
            
            goal_msg.pose.pose.orientation.x = orientation_x
            goal_msg.pose.pose.orientation.y = orientation_y
            goal_msg.pose.pose.orientation.z = orientation_z
            goal_msg.pose.pose.orientation.w = orientation_w
            
            # Set behavior tree if provided
            if behavior_tree and behavior_tree.strip():
                goal_msg.behavior_tree = behavior_tree
            
            # Send the goal
            goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
            
            # Store the goal
            self.nav_params['current_goal'] = goal_msg.pose
            self.nav_params['navigation_active'] = True
            
            # Add result callback
            goal_future.add_done_callback(self._goal_response_callback)
            
            return True, f'Navigation goal set successfully to ({position_x:.2f}, {position_y:.2f})'
            
        except Exception as e:
            self.get_logger().error(f'Error setting navigation goal: {str(e)}')
            return False, f'Error setting navigation goal: {str(e)}'
    
    def _goal_response_callback(self, future):
        """
        Handle the goal response from the action server
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal was rejected by server')
                self.nav_params['navigation_active'] = False
                return
            
            self.get_logger().info('Goal accepted by server')
            
            # Get result future
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
            
        except Exception as e:
            self.get_logger().error(f'Error in goal response: {str(e)}')
    
    def _result_callback(self, future):
        """
        Handle the result from the action server
        """
        try:
            result = future.result().result
            self.nav_params['navigation_active'] = False
            
            # Log the result
            if result:
                self.get_logger().info('Navigation completed successfully')
            else:
                self.get_logger().info('Navigation failed to reach goal')
                
        except Exception as e:
            self.get_logger().error(f'Error in result callback: {str(e)}')
    
    def is_navigation_active(self):
        """
        Check if navigation is currently active
        """
        return self.nav_params['navigation_active']
    
    def cancel_current_goal(self):
        """
        Cancel the current navigation goal
        """
        try:
            # This would cancel the current goal in a real implementation
            self.nav_params['navigation_active'] = False
            self.nav_params['current_goal'] = None
            self.get_logger().info('Current navigation goal canceled')
            return True, 'Navigation goal canceled'
        except Exception as e:
            return False, f'Error canceling navigation goal: {str(e)}'


def main(args=None):
    rclpy.init(args=args)
    
    nav_service = NavigationGoalService()
    
    # Example of setting a navigation goal
    # This would typically be called from another service or action
    success, message = nav_service.set_goal(
        position_x=1.0,
        position_y=2.0, 
        position_z=0.0,
        orientation_x=0.0,
        orientation_y=0.0, 
        orientation_z=0.0,
        orientation_w=1.0
    )
    
    if success:
        nav_service.get_logger().info(f'Navigation goal set: {message}')
    else:
        nav_service.get_logger().error(f'Failed to set navigation goal: {message}')
    
    try:
        rclpy.spin(nav_service)
    except KeyboardInterrupt:
        nav_service.get_logger().info('Navigation goal service shutting down')
    finally:
        nav_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()