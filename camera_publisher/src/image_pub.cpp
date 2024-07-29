#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "std_msgs/msg/header.hpp"

#include <chrono>
#include <functional>
#include <memory>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher"), count_(0)
    {
        // Publisher for the image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/android_camera/image", 10);

        // Timer to publish the image
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImagePublisher::publish_image, this));
    }
    
private:
    // Callback function to publish the image
    void publish_image()
    {
        // Create capture from camera 2 (change later)
        cv::VideoCapture cap(0);

        // Check if the camera is opened
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening camera");
            return;
        }

        // Create a frame to store the image
        cv::Mat frame;
        cap >> frame;

        // Resize the image
        cv::resize(frame, frame, cv::Size(960, 720));

        // Create a message to store the image
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publish the image
        publisher_->publish(*msg_);
        RCLCPP_INFO(this->get_logger(), "Published image %ld", count_++);

        // Release the camera
        cap.release();
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;   // Publisher for the image
    rclcpp::TimerBase::SharedPtr timer_;                                // Timer to publish the image
    sensor_msgs::msg::Image::SharedPtr msg_;                            // Message to store the image
    size_t count_;                                                      // Counter for the image                                          

};

int main(int argc, char *argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<ImagePublisher>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}