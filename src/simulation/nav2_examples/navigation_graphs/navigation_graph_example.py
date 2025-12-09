#!/usr/bin/env python3
"""
Navigation Graph Examples for Humanoid Robots
This script demonstrates creating and using navigation graphs for humanoid robots
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from std_msgs.msg import String
import math
import json


class NavigationGraphExample(Node):
    """
    Example node demonstrating navigation graphs for humanoid robots
    """
    def __init__(self):
        super().__init__('navigation_graph_example')
        
        # Action client for navigation
        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )
        
        # Store predefined navigation graphs
        self.navigation_graphs = {}
        
        # Load example graphs
        self._create_example_graphs()
        
        self.get_logger().info('Navigation Graph Example node initialized')
    
    def _create_example_graphs(self):
        """
        Create example navigation graphs
        """
        # Graph 1: Square path
        square_path = []
        for i, (x, y, theta) in enumerate([
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi/2),
            (0.0, 1.0, math.pi),
            (0.0, 0.0, -math.pi/2)
        ]):
            pose = self._create_pose_stamped(x, y, theta)
            square_path.append(pose)
        
        self.navigation_graphs['square_path'] = {
            'waypoints': square_path,
            'name': 'Square Navigation Path',
            'description': 'Simple square path for humanoid navigation testing'
        }
        
        # Graph 2: Corridor path
        corridor_path = []
        for i, (x, y, theta) in enumerate([
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
            (6.0, 0.0, 0.0),
            (8.0, 0.0, 0.0)
        ]):
            pose = self._create_pose_stamped(x, y, theta)
            corridor_path.append(pose)
        
        self.navigation_graphs['corridor_path'] = {
            'waypoints': corridor_path,
            'name': 'Corridor Navigation Path',
            'description': 'Straight path through a corridor'
        }
        
        # Graph 3: Room exploration
        room_path = []
        for i, (x, y, theta) in enumerate([
            (0.0, 0.0, 0.0),   # Start position
            (2.0, 0.5, 0.0),   # Move to first room
            (2.0, 2.0, math.pi/2),  # Turn toward window
            (2.5, 2.0, math.pi/2),  # Move to window
            (2.0, 2.0, math.pi),    # Turn back
            (0.0, 2.0, math.pi),    # Move to second room
            (0.0, 0.0, -math.pi/2)  # Return to start
        ]):
            pose = self._create_pose_stamped(x, y, theta)
            room_path.append(pose)
        
        self.navigation_graphs['room_exploration'] = {
            'waypoints': room_path,
            'name': 'Room Exploration Path',
            'description': 'Path for exploring multiple rooms'
        }
        
        self.get_logger().info(f'Created {len(self.navigation_graphs)} navigation graphs')
    
    def _create_pose_stamped(self, x, y, theta):
        """
        Helper function to create PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import math
        quat_w = math.cos(theta / 2.0)
        quat_x = 0.0
        quat_y = 0.0
        quat_z = math.sin(theta / 2.0)
        
        pose.pose.orientation.w = quat_w
        pose.pose.orientation.x = quat_x
        pose.pose.orientation.y = quat_y
        pose.pose.orientation.z = quat_z
        
        return pose
    
    def execute_navigation_graph(self, graph_name):
        """
        Execute a navigation graph
        """
        if graph_name not in self.navigation_graphs:
            self.get_logger().error(f'Navigation graph {graph_name} not found')
            return False
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create goal message with all waypoints
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.navigation_graphs[graph_name]['waypoints']
        
        self.get_logger().info(f'Executing navigation graph: {graph_name}')
        self.get_logger().info(f'Number of waypoints: {len(goal_msg.poses)}')
        
        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._navigation_result_callback)
        
        return True
    
    def _navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        try:
            goal_result = future.result()
            if goal_result:
                self.get_logger().info('Navigation completed successfully')
            else:
                self.get_logger().error('Navigation failed')
        except Exception as e:
            self.get_logger().error(f'Navigation error: {str(e)}')
    
    def list_available_graphs(self):
        """
        List all available navigation graphs
        """
        self.get_logger().info('Available navigation graphs:')
        for name, graph in self.navigation_graphs.items():
            self.get_logger().info(f'  - {name}: {graph["name"]}')
            self.get_logger().info(f'    Description: {graph["description"]}')
            self.get_logger().info(f'    Waypoints: {len(graph["waypoints"])}')
            self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    
    nav_graph_example = NavigationGraphExample()
    
    # List all available graphs
    nav_graph_example.list_available_graphs()
    
    # Execute one of the example graphs (uncomment to run)
    # nav_graph_example.execute_navigation_graph('square_path')
    
    try:
        # Keep the node running to maintain action client
        rclpy.spin(nav_graph_example)
    except KeyboardInterrupt:
        nav_graph_example.get_logger().info('Navigation graph example stopped by user')
    finally:
        nav_graph_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()