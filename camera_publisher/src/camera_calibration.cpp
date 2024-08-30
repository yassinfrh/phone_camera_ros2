#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Package share directory
std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_publisher");

// Path to store the calibration parameters
const std::string calibration_path = package_share_directory + "/calibration_parameters/";

// Free function to handle the mouse callback
void onMouse(int event, int x, int y, int flags, void *userdata);

class CameraCalibration : public rclcpp::Node
{
public:
    CameraCalibration(int camera_id, int checkerboard[2]) : Node("camera_calibration"), camera_id_(camera_id)
    {
        // Initialize the checkerboard size
        checkerboard_[0] = checkerboard[0];
        checkerboard_[1] = checkerboard[1];

        // Create the calibration directory if it does not exist
        if (!std::filesystem::exists(calibration_path))
        {
            std::filesystem::create_directories(calibration_path);
        }

        // Timer to display the image
        display_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraCalibration::show_image, this));
    }

    // Function to perform the calibration
    void perform_calibration()
    {
        // If the number of stored points is less than 10, print an error message
        if (objpoints_.size() < 10 || imgpoints_.size() < 10)
        {
            std::cout << "Error: Insufficient points for calibration" << std::endl;
            return;
        }

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);

        // Calibration parameters
        cv::Mat cameraMatrix, distCoeffs, R, T;

        /*
         * Performing camera calibration by
         * passing the value of known 3D points (objpoints)
         * and corresponding pixel coordinates of the
         * detected corners (imgpoints)
         */
        cv::calibrateCamera(objpoints_, imgpoints_, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

        // Print the calibration parameters
        std::cout << "Camera Matrix: " << cameraMatrix << std::endl;

        // Reinitialize the objpoints and imgpoints
        objpoints_.clear();
        imgpoints_.clear();
    }

    // Flag to store the points
    bool store_points_{false};

private:
    // Function to compute points for calibration
    cv::Mat compute_calibration_points(cv::Mat frame)
    {
        // Vector to store the pixel coordinates of detected checker board corners
        std::vector<cv::Point2f> corner_pts;

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for (int i{0}; i < checkerboard_[1]; i++)
        {
            for (int j{0}; j < checkerboard_[0]; j++)
                objp.push_back(cv::Point3f(j, i, 0));
        }

        // Convert the image to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners
        bool success = cv::findChessboardCorners(gray, cv::Size(checkerboard_[0], checkerboard_[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (success)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(checkerboard_[0], checkerboard_[1]), corner_pts, success);

            // If the stored points are more than 20, stop storing
            if (objpoints_.size() < 20 && imgpoints_.size() < 20 && store_points_)
            {
                objpoints_.push_back(objp);
                imgpoints_.push_back(corner_pts);
                store_points_ = false;

                std::cout << "Stored points: " << objpoints_.size() << std::endl;
            }
        }

        return frame;
    }

    // Function to display the image
    void show_image()
    {
        // Create capture from camera
        cv::VideoCapture cap(this->camera_id_);

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

        // Check if the image is empty
        if (image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error displaying image");
            return;
        }

        // Compute the calibration points
        image_ = compute_calibration_points(image_);

        std::string text = "Left click to store the points, Right click to calibrate the camera";

        // Display the image
        cv::imshow(text, image_);

        // Set the mouse callback
        cv::setMouseCallback(text, onMouse, this);

        // Wait for 1ms to allow the window to update
        cv::waitKey(1);

        // Release the camera
        cap.release();
    }

    // Camera ID
    int camera_id_;

    // Current image
    cv::Mat image_ = cv::Mat::zeros(720, 960, CV_8UC3);

    // Timers for acquiring and displaying images
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;

    // Checkerboard size
    int checkerboard_[2];

    // Vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f>> objpoints_;

    // Vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgpoints_;
};

// Free function to handle the mouse callback
void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_RBUTTONDOWN)
    {
        // Retrieve the camera_calibration pointer from userdata
        CameraCalibration *camera_calibration = static_cast<CameraCalibration *>(userdata);
        if (camera_calibration != nullptr)
        {
            // Perform the calibration
            camera_calibration->perform_calibration();
        }
    }
    else if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Retrieve the camera_calibration pointer from userdata
        CameraCalibration *camera_calibration = static_cast<CameraCalibration *>(userdata);
        if (camera_calibration != nullptr)
        {
            // Set the flag to store the points
            camera_calibration->store_points_ = true;
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
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

    // Ask the user for the checkerboard size
    int checkerboard_size[2];
    while (true)
    {
        std::cout << "Enter the number of rows in the checkerboard: ";
        std::cin >> checkerboard_size[0];
        std::cout << "Enter the number of columns in the checkerboard: ";
        std::cin >> checkerboard_size[1];

        if (checkerboard_size[0] > 0 && checkerboard_size[1] > 0)
        {
            break;
        }

        std::cout << "Invalid checkerboard size" << std::endl;
    }

    // Create the camera calibration object
    auto camera_calibration = std::make_shared<CameraCalibration>(camera_id, checkerboard_size);

    // Spin the node
    rclcpp::spin(camera_calibration);

    // Shutdown the ROS node
    rclcpp::shutdown();

    return 0;
}