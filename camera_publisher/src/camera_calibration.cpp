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
#include <thread>

// TODO: Create generlized path for the images and calibration parameters

// Path to images for calibration
const std::string images_path = "/home/yassin/ros_ws/src/phone_camera_ros2/camera_publisher/calibration_images/";

// Path to store the calibration parameters
const std::string calibration_path = "/home/yassin/ros_ws/src/phone_camera_ros2/camera_publisher/calibration_parameters/";

// Free function to handle the mouse callback
void onMouse(int event, int x, int y, int flags, void *userdata);

class CameraCalibration : public rclcpp::Node
{
public:
    CameraCalibration(int camera_id) : Node("camera_calibration"), camera_id_(camera_id)
    {
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

    // Function to perform the calibration
    void perform_calibration()
    {
        // TODO: ask for the checkerboard size
        // Checkboard size
        int CHECKERBOARD[2]{7, 10};

        // Check if there are enough images for calibration
        if (count_ < 10)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough images for calibration");
            return;
        }

        // Creating vector to store vectors of 3D points for each checkerboard image
        std::vector<std::vector<cv::Point3f>> objpoints;

        // Creating vector to store vectors of 2D points for each checkerboard image
        std::vector<std::vector<cv::Point2f>> imgpoints;

        // Defining the world coordinates for 3D points
        std::vector<cv::Point3f> objp;
        for (int i{0}; i < CHECKERBOARD[1]; i++)
        {
            for (int j{0}; j < CHECKERBOARD[0]; j++)
                objp.push_back(cv::Point3f(j, i, 0));
        }

        // Extracting path of individual image stored in a given directory
        std::vector<cv::String> images;

        // Path of the folder containing checkerboard images
        std::string path = images_path + "*.png";

        cv::glob(path, images);

        cv::Mat frame, gray;
        // vector to store the pixel coordinates of detected checker board corners
        std::vector<cv::Point2f> corner_pts;
        bool success;

        // Looping over all the images in the directory
        for (int i{0}; i < images.size(); i++)
        {
            frame = cv::imread(images[i]);
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // Finding checker board corners
            // If desired number of corners are found in the image then success = true
            success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

            /*
             * If desired number of corner are detected,
             * we refine the pixel coordinates and display
             * them on the images of checker board
            */
            if (success)
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);

                // refining pixel coordinates for given 2d points.
                cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                // Displaying the detected corner points on the checker board
                cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
            }

            cv::imshow("Image", frame);
            cv::waitKey(0);
        }

        cv::destroyAllWindows();

        cv::Mat cameraMatrix, distCoeffs, R, T;

        /*
         * Performing camera calibration by
         * passing the value of known 3D points (objpoints)
         * and corresponding pixel coordinates of the
         * detected corners (imgpoints)
        */
        cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

        
    }

private:
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

        std::string text = "Left click on the image to save it and right click to perform the calibration";

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
        if (camera_calibration != nullptr)
        {
            // Save the image
            camera_calibration->save_image();
        }
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        // Retrieve the camera_calibration pointer from userdata
        CameraCalibration *camera_calibration = static_cast<CameraCalibration *>(userdata);
        if (camera_calibration != nullptr)
        {
            // Perform the calibration
            camera_calibration->perform_calibration();
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

    // Ask for the camera name
    std::string camera_name;
    std::cout << "Enter the camera name: ";
    std::cin >> camera_name;

    // TODO: use the camera name to create a folder with the calibration images and parameters

    // Create the camera calibration object
    auto camera_calibration = std::make_shared<CameraCalibration>(camera_id);

    // Spin the node
    rclcpp::spin(camera_calibration);

    // Shutdown the ROS node
    rclcpp::shutdown();

    return 0;
}