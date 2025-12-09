#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class DepthCameraSimulator : public rclcpp::Node
{
public:
    DepthCameraSimulator() : Node("depth_camera_simulator")
    {
        // Create publishers for depth image and camera info
        depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "depth/image_raw", 10);
        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "depth/camera_info", 10);
        
        // Create a timer to publish data at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30Hz
            std::bind(&DepthCameraSimulator::publish_depth_image, this));
        
        // Initialize camera parameters
        camera_info_.header.frame_id = "camera_link";
        camera_info_.height = 480;
        camera_info_.width = 640;
        camera_info_.distortion_model = "plumb_bob";
        
        // Default camera matrix (these would be calibrated values in real usage)
        camera_info_.k = {909.8492431640625, 0.0, 650.4150390625, 
                          0.0, 909.6011962890625, 348.3458251953125, 
                          0.0, 0.0, 1.0};
        
        // Default projection matrix
        camera_info_.p = {909.8492431640625, 0.0, 650.4150390625, 0.0,
                          0.0, 909.6011962890625, 348.3458251953125, 0.0,
                          0.0, 0.0, 1.0, 0.0};
        
        RCLCPP_INFO(this->get_logger(), "Depth Camera Simulator node initialized");
    }

private:
    void publish_depth_image()
    {
        // Create a simulated depth image
        cv::Mat depth_image = cv::Mat::zeros(480, 640, CV_32FC1);
        
        // Fill with a gradient from 1m to 10m (top to bottom)
        for (int y = 0; y < depth_image.rows; y++) {
            for (int x = 0; x < depth_image.cols; x++) {
                float depth = 1.0 + (9.0 * y / depth_image.rows);  // 1m to 10m
                
                // Add a simulated object at the center (rectangular obstacle at 2m)
                if (y > 200 && y < 300 && x > 280 && x < 360) {
                    depth = 2.0;  // Object at 2m
                }
                
                depth_image.at<float>(y, x) = depth;
            }
        }
        
        // Convert to ROS2 message
        cv_bridge::CvImage cv_image;
        cv_image.header.frame_id = "camera_link";
        cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        cv_image.image = depth_image;
        
        // Update header timestamp
        cv_image.header.stamp = this->now();
        
        // Publish the depth image
        depth_publisher_->publish(*cv_image.toImageMsg());
        
        // Update and publish camera info
        camera_info_.header.stamp = this->now();
        camera_info_publisher_->publish(camera_info_);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sensor_msgs::msg::CameraInfo camera_info_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCameraSimulator>());
    rclcpp::shutdown();
    return 0;
}