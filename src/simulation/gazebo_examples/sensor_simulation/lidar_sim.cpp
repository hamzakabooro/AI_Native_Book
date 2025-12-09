#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarSimulator : public rclcpp::Node
{
public:
    LidarSimulator() : Node("lidar_simulator")
    {
        // Create a publisher for the laser scan topic
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan", 10);
        
        // Create a timer to publish data at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&LidarSimulator::publish_scan, this));
        
        RCLCPP_INFO(this->get_logger(), "Lidar Simulator node initialized");
    }

private:
    void publish_scan()
    {
        auto message = sensor_msgs::msg::LaserScan();
        
        // Fill in the header
        message.header.stamp = this->now();
        message.header.frame_id = "lidar_link";
        
        // Set scan parameters
        message.angle_min = -M_PI / 2;  // -90 degrees
        message.angle_max = M_PI / 2;   // 90 degrees
        message.angle_increment = M_PI / 180;  // 1 degree
        message.time_increment = 0.0;
        message.scan_time = 0.1;  // 10Hz
        message.range_min = 0.1;  // 0.1m
        message.range_max = 10.0; // 10m
        
        // Calculate number of ranges
        int num_ranges = std::ceil((message.angle_max - message.angle_min) / 
                                  message.angle_increment) + 1;
        
        // Resize ranges array
        message.ranges.resize(num_ranges);
        
        // Fill in ranges with simulated data
        for (int i = 0; i < num_ranges; i++) {
            // Simulate an obstacle at 2 meters at 0 degrees
            float angle = message.angle_min + i * message.angle_increment;
            
            // Simple simulation: if looking straight ahead, there's an obstacle at 2m
            // Otherwise, max range
            if (std::abs(angle) < 0.1) {  // About 0 degrees
                message.ranges[i] = 2.0;
            } else {
                message.ranges[i] = message.range_max;
            }
        }
        
        // Publish the message
        publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulator>());
    rclcpp::shutdown();
    return 0;
}