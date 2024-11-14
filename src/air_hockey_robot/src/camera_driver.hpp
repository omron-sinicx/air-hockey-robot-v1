#ifndef USB_CAMERA_HPP_
#define USB_CAMERA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "air_hockey_robot_config.hpp"

class CameraDriver : public rclcpp::Node
{
public:
    CameraDriver();
private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    cv::VideoCapture video_capture;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::Mat optimal_camera_matrix;
    std::array<std::array<double, 3>, 3> camera_matrix_data = CAMERA_MATRIX;
    std::array<double, 5> distortion_coefficients_data = DISTORTION_COEFFICIENTS;
};

#endif /* USB_CAMERA_HPP_ */
