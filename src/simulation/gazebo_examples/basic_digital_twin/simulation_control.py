import rclpy
from rclpy.node import Node
from sim_msgs.srv import StartSimulation

class SimulationControlService(Node):
    def __init__(self):
        super().__init__('simulation_control_service')
        self.srv = self.create_service(
            StartSimulation, 
            'simulation_control/start', 
            self.handle_start_simulation
        )
        self.simulation_counter = 0
        
    def handle_start_simulation(self, request, response):
        self.get_logger().info(
            f'Request to start simulation: world={request.world_name}, '
            f'physics_rate={request.physics_rate}, '
            f'rtf={request.real_time_factor}, '
            f'gui={request.enable_gui}'
        )
        
        # Simulate starting the simulation
        # In a real implementation, this would interface with Gazebo
        response.success = True
        response.message = f'Simulation started with world: {request.world_name}'
        self.simulation_counter += 1
        response.simulation_id = f'sim_{self.simulation_counter:04d}'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimulationControlService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()