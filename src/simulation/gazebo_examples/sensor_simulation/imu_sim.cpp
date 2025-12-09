#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class ImuSimulator : public rclcpp::Node
{
public:
    ImuSimulator() : Node("imu_simulator")
    {
        // Create a publisher for the IMU topic
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu", 10);
        
        // Create a timer to publish data at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&ImuSimulator::publish_imu_data, this));
        
        // Initialize orientation to be roughly level (z pointing up)
        orientation_.w = 1.0;  // No rotation initially
        orientation_.x = 0.0;
        orientation_.y = 0.0;
        orientation_.z = 0.0;
        
        // Initialize angular velocity (rotating slowly around Z axis)
        angular_velocity_.x = 0.0;
        angular_velocity_.y = 0.0;
        angular_velocity_.z = 0.1;  // 0.1 rad/s around Z axis
        
        // Initialize linear acceleration (just gravity in Z direction)
        linear_acceleration_.x = 0.0;
        linear_acceleration_.y = 0.0;
        linear_acceleration_.z = 9.81;  // Gravity in the Z axis
        
        RCLCPP_INFO(this->get_logger(), "IMU Simulator node initialized");
    }

private:
    void publish_imu_data()
    {
        auto message = sensor_msgs::msg::Imu();
        
        // Fill in the header
        message.header.stamp = this->now();
        message.header.frame_id = "imu_link";
        
        // Set orientation (with small rotation to simulate movement)
        message.orientation = orientation_;
        
        // Set angular velocity
        message.angular_velocity = angular_velocity_;
        
        // Set linear acceleration
        message.linear_acceleration = linear_acceleration_;
        
        // Add covariance matrices (setting to zero since it's simulated)
        for (int i = 0; i < 9; i++) {
            message.orientation_covariance[i] = 0.0;
            message.angular_velocity_covariance[i] = 0.0;
            message.linear_acceleration_covariance[i] = 0.0;
        }
        
        // Publish the message
        publisher_->publish(message);
        
        // Update orientation based on angular velocity (simple integration)
        // Using basic approximation: q_new = q + 0.5*dt*w*q
        // where w is the angular velocity quaternion [0, wx, wy, wz]
        double dt = 0.01;  // 10ms based on 100Hz rate
        double qw = orientation_.w;
        double qx = orientation_.x;
        double qy = orientation_.y;
        double qz = orientation_.z;
        
        // Calculate quaternion derivative
        double dqw = -0.5 * (angular_velocity_.x * qx + angular_velocity_.y * qy + angular_velocity_.z * qz);
        double dqx =  0.5 * (angular_velocity_.w * qx + angular_velocity_.z * qy - angular_velocity_.y * qz);
        double dqy =  0.5 * (angular_velocity_.w * qy - angular_velocity_.z * qx + angular_velocity_.x * qz);
        double dqz =  0.5 * (angular_velocity_.w * qz + angular_velocity_.y * qx - angular_velocity_.x * qy);
        
        // Update orientation (with simple integration)
        orientation_.w = qw + dt * dqw;
        orientation_.x = qx + dt * dqx;
        orientation_.y = qy + dt * dqy;
        orientation_.z = qz + dt * dqz;
        
        // Normalize quaternion to maintain unit length
        double norm = sqrt(orientation_.w*orientation_.w + 
                          orientation_.x*orientation_.x + 
                          orientation_.y*orientation_.y + 
                          orientation_.z*orientation_.z);
        if (norm > 0.0) {
            orientation_.w /= norm;
            orientation_.x /= norm;
            orientation_.y /= norm;
            orientation_.z /= norm;
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    
    geometry_msgs::msg::Quaternion orientation_;
    geometry_msgs::msg::Vector3 angular_velocity_;
    geometry_msgs::msg::Vector3 linear_acceleration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSimulator>());
    rclcpp::shutdown();
    return 0;
}