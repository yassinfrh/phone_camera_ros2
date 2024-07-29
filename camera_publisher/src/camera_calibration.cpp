#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <filesystem>
#include <thread>

// Path to images for calibration
const std::string images_path = "/home/yassin/ros_ws/src/android_camera/camera_publisher/calibration_images/";

// Path to store the calibration parameters
const std::string calibration_path = "/home/yassin/ros_ws/src/android_camera/camera_publisher/calibration_parameters/";

// Free function to handle the mouse callback
void onMouse(int event, int x, int y, int flags, void *userdata);


class CameraCalibration : public rclcpp::Node
{
public:
    CameraCalibration() : Node("camera_calibration")
    {
        // Timer to acquire images
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraCalibration::acquire_image, this));

        // Create the images directory if it does not exist
        if (!std::filesystem::exists(images_path))
        {
            std::filesystem::create_directories(images_path);
        }

        // Create the calibration directory if it does not exist
        if (!std::filesystem::exists(calibration_path))
        {
            std::filesystem::create_directories(calibration_path);
        }

        // Initialize the counter to 0 if there are no images
        if (std::filesystem::is_empty(images_path))
        {
            count_ = 0;
        }
        else
        {
            // Count the number of images in the directory
            count_ = std::distance(std::filesystem::directory_iterator(images_path), std::filesystem::directory_iterator{});
        }

        // Timer to display the image
        display_timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&CameraCalibration::show_image, this));
    }

    // Function to save the image
    void save_image()
    {
        // Check if the image is empty
        if (image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error saving image");
            return;
        }

        // Save the image
        std::string image_name = images_path + "image" + std::to_string(count_) + ".png";
        cv::imwrite(image_name, image_);

        // Increment the counter
        count_++;
    }

private:
    // Callback function to acquire images
    void acquire_image()
    {
        // Create capture from camera 2 (change later)
        cv::VideoCapture cap(0);

        // Check if the camera is opened
        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening camera");
            return;
        }

        // Store the image
        cap >> image_;

        // Resize the image
        cv::resize(image_, image_, cv::Size(960, 720));

        // Release the camera
        cap.release();
    }

    // Function to display the image
    void show_image()
    {
        // Check if the image is empty
        if (image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error displaying image");
            return;
        }

        // Display the image
        cv::imshow("Click on the image to save it", image_);

        // Set the mouse callback
        cv::setMouseCallback("Click on the image to save it", onMouse, this);

        // Wait for 1ms to allow the window to update
        cv::waitKey(1);
    }

    // Current image
    cv::Mat image_ = cv::Mat::zeros(720, 960, CV_8UC3);

    // Timers for acquiring and displaying images
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;

    // Counter for the image
    size_t count_;
};

// Free function to handle the mouse callback
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Retrieve the camera_calibration pointer from userdata
        CameraCalibration *camera_calibration = static_cast<CameraCalibration *>(userdata);
        if (camera_calibration)
        {
            // Save the image
            camera_calibration->save_image();
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    rclcpp::init(argc, argv);

    // Create the camera calibration object
    auto camera_calibration = std::make_shared<CameraCalibration>();

    // Spin the node
    rclcpp::spin(camera_calibration);

    // Shutdown the ROS node
    rclcpp::shutdown();

    return 0;
}