#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "std_msgs/msg/header.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <vector>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Package share directory
std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_publisher");

// Path to store the calibration parameters
const std::string calibration_path = package_share_directory + "/calibration_parameters/";

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher(int camera_id) : Node("image_publisher"), count_(0), camera_id_(camera_id)
    {
        // Publisher for the image
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/phone_camera/image", 10);

        // Publisher for the camera info
        info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/phone_camera/camera_info", 10);

        // Timer to publish the image
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImagePublisher::publish_image, this));

        // Camera calibration parameters path
        std::string file_path = calibration_path + "calibration_parameters.txt";
        // Check if the file exists
        if (std::filesystem::exists(file_path))
        {
            // Read the calibration parameters
            read_calibration_parameters(file_path);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Calibration parameters not found. Please calibrate the camera");
        }
    }

private:
    // Function to parse a matrix or vector from a line
    std::vector<double> parse_line(std::string line)
    {
        std::vector<double> output;
        std::istringstream iss(line);
        double value;

        // If the first character is , ; ] [ ignore it
        if (iss.peek() == ',' || iss.peek() == ';' || iss.peek() == ']' || iss.peek() == '[')
        {
            iss.ignore();
        }

        // Read the values from the line
        while (iss >> value)
        {
            // Push the value to the camera matrix
            output.push_back(value);
            // Ignore , ; ] [ characters
            if (iss.peek() == ',' || iss.peek() == ';' || iss.peek() == ']' || iss.peek() == '[')
            {
                iss.ignore();
            }
        }

        return output;
    }

    // Function to read the calibration parameters
    void read_calibration_parameters(std::string file_path)
    {
        // Read the calibration parameters file
        std::ifstream file(file_path);
        std::string line;

        // Check if the file is open
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening file");
            return;
        }

        // Find the string "Camera Matrix:" and go to the next line
        while (std::getline(file, line))
        {
            if (line.find("Camera Matrix:") != std::string::npos)
            {
                break;
            }
        }

        // Combine the next 3 lines into a single line
        std::string camera_matrix_line;
        for (int i = 0; i < 3; i++)
        {
            std::getline(file, line);
            camera_matrix_line += line;
        }

        // Parse the camera matrix
        std::vector<double> camera_matrix = parse_line(camera_matrix_line);

        // Find the string "Distortion Coefficients:" and go to the next line
        while (std::getline(file, line))
        {
            if (line.find("Distortion Coefficients:") != std::string::npos)
            {
                break;
            }
        }

        // Get next line
        std::getline(file, line);

        // Parse the distortion coefficients
        std::vector<double> distortion_coefficients = parse_line(line);

        // Create the camera info message
        camera_info_.header.frame_id = "phone_camera";

        camera_info_.height = 720;
        camera_info_.width = 960;

        camera_info_.distortion_model = "plumb_bob";
        camera_info_.d.resize(5);
        for (int i = 0; i < 5; i++)
        {
            camera_info_.d[i] = distortion_coefficients[i];
        }

        camera_info_.k.fill(0.0);
        for (int i = 0; i < 9; i++)
        {
            camera_info_.k[i] = camera_matrix[i];
        }

        camera_info_.r.fill(0.0);
        camera_info_.r[0] = 1.0;
        camera_info_.r[4] = 1.0;
        camera_info_.r[8] = 1.0;

        camera_info_.p.fill(0.0);
        camera_info_.p[0] = camera_matrix[0];
        camera_info_.p[2] = camera_matrix[2];
        camera_info_.p[5] = camera_matrix[4];
        camera_info_.p[6] = camera_matrix[5];
        camera_info_.p[10] = 1.0;


    }

    // Callback function to publish the image
    void publish_image()
    {
        // Create capture from camera_id
        cv::VideoCapture cap(camera_id_);

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
        image_publisher_->publish(*msg_);
        RCLCPP_INFO(this->get_logger(), "Published image %ld", count_++);

        // Release the camera
        cap.release();

        // Check if camera info is empty
        if (camera_info_.d.size() == 0)
        {
            return;
        }

        // Edit header of camera info
        camera_info_.header.stamp = this->now();

        // Publish the camera info
        info_publisher_->publish(camera_info_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;     // Publisher for the image
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_; // Publisher for the camera info
    rclcpp::TimerBase::SharedPtr timer_;                                        // Timer to publish the image
    sensor_msgs::msg::Image::SharedPtr msg_;                                    // Message to store the image
    size_t count_;                                                              // Counter for the image
    int camera_id_;                                                             // Camera ID
    sensor_msgs::msg::CameraInfo camera_info_;                                  // Camera info message
};

int main(int argc, char *argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Ask the user for the camera ID
    int camera_id;
    while (true)
    {
        std::cout << "Enter the camera ID: ";
        std::cin >> camera_id;

        // Check if the camera ID is valid
        cv::VideoCapture cap(camera_id);
        if (cap.isOpened())
        {
            cap.release();
            break;
        }
    }

    // Create the node
    auto node = std::make_shared<ImagePublisher>(camera_id);

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}