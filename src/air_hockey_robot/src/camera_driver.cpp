#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include "camera_driver.hpp"

CameraDriver::CameraDriver() : Node("camera_driver")
{
    this->timer = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&CameraDriver::timer_callback, this));
    this->image_publisher = this->create_publisher<sensor_msgs::msg::Image>("usb_camera_image", 10);
    std::string camera_device_path = this->declare_parameter<std::string>("camera_device_path", "/dev/video0");
    if(!this->video_capture.open(camera_device_path, cv::CAP_V4L2))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s", camera_device_path.c_str());
        rclcpp::shutdown();
    }
    this->video_capture.set(cv::CAP_PROP_FPS, 30.0);
    this->camera_matrix = cv::Mat(3, 3, CV_64FC1, &camera_matrix_data);
    this->distortion_coefficients = cv::Mat(5, 1, CV_64FC1, &distortion_coefficients_data);
    cv::Mat dummy_frame;
    this->video_capture.read(dummy_frame);
    this->optimal_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, dummy_frame.size(), 0, dummy_frame.size());
    std::cout << "size: " << dummy_frame.size() << std::endl; 
}

void CameraDriver::timer_callback()
{
    sensor_msgs::msg::Image ros_image;
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";
    cv::Mat original_frame;
    this->video_capture.read(original_frame);
    cv::Mat undistorted_frame;
    cv::undistort(original_frame, undistorted_frame, this->camera_matrix, this->distortion_coefficients, this->optimal_camera_matrix);
    cv_image.image = undistorted_frame;
    cv_image.header.stamp = this->now();
    cv_image.toImageMsg(ros_image);
    this->image_publisher->publish(std::move(ros_image));
}
