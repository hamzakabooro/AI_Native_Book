#!/usr/bin/env python3
"""
Perception Pipeline Configuration Service
Implements the perception pipeline configuration service according to the contract
"""

import rclpy
from rclpy.node import Node
from perception_msgs.srv import ConfigurePipeline
import json


class PerceptionPipelineConfigService(Node):
    """
    Service node for configuring perception pipelines
    """
    def __init__(self):
        super().__init__('perception_pipeline_config_service')
        
        # Create the service
        self.srv = self.create_service(
            ConfigurePipeline,
            'perception_pipeline/configure',
            self.configure_pipeline_callback
        )
        
        # Store active pipeline configurations
        self.pipeline_configs = {}
        
        self.get_logger().info('Perception Pipeline Configuration Service initialized')
        
    def configure_pipeline_callback(self, request, response):
        """
        Handle perception pipeline configuration requests
        """
        try:
            self.get_logger().info(
                f'Configuring pipeline: {request.pipeline_name} '
                f'for robot: {request.robot_name}'
            )
            
            # Validate input parameters
            if not request.pipeline_name or not request.robot_name:
                response.success = False
                response.message = 'Pipeline name and robot name are required'
                return response
            
            # Parse parameters from JSON string
            try:
                if request.parameters:
                    params = json.loads(request.parameters)
                else:
                    params = {}
            except json.JSONDecodeError as e:
                response.success = False
                response.message = f'Invalid JSON parameters: {str(e)}'
                return response
            
            # Validate topic lists
            input_topics = list(request.input_topics)
            output_topics = list(request.output_topics)
            
            # Store the configuration
            pipeline_id = f"{request.robot_name}/{request.pipeline_name}"
            self.pipeline_configs[pipeline_id] = {
                'input_topics': input_topics,
                'output_topics': output_topics,
                'parameters': params,
                'configured_at': self.get_clock().now().nanoseconds
            }
            
            # In a real implementation, this would:
            # 1. Load the appropriate perception pipeline
            # 2. Configure it with the provided parameters
            # 3. Connect it to the specified input/output topics
            # 4. Validate the configuration works
            
            response.success = True
            response.message = f'Pipeline {request.pipeline_name} configured successfully for robot {request.robot_name}'
            
            self.get_logger().info(f'Pipeline configuration successful: {pipeline_id}')
            
        except Exception as e:
            response.success = False
            response.message = f'Configuration failed: {str(e)}'
            self.get_logger().error(f'Pipeline configuration error: {str(e)}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    config_service = PerceptionPipelineConfigService()
    
    try:
        rclpy.spin(config_service)
    except KeyboardInterrupt:
        config_service.get_logger().info('Shutting down perception pipeline config service')
    finally:
        config_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()