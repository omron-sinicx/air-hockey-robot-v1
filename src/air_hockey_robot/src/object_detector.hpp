#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "air_hockey_robot/msg/visual_points.hpp"

class ObjectDetector : public rclcpp::Node
{
public:
    ObjectDetector(std::string object); // ADD #include <string> ?????
    ~ObjectDetector();
private:
    void ObjectDetectorCallback(sensor_msgs::msg::Image::SharedPtr pImage);
    int detected_p, detected_h, detected_r;
    int not_detected_p, not_detected_h, not_detected_r;
    std::string object_name;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber;
    rclcpp::Publisher<air_hockey_robot::msg::VisualPoints>::SharedPtr PointsPublisher;
    static cv::RotatedRect Pt2RotatedRect(const cv::Point& rPt1, const cv::Point& rPt2);
    cv::Scalar min_hsv;
    cv::Scalar max_hsv;
    static int CurrentDisplayMode;
    enum class DISPLAY_MODE
    {
        IDLE = 0,
        AREA = 1,
        MASK = 2
    };
    enum class OBJECT_STATUS
    {
        OK               = 0,
        EXCEPTION_ERROR  = 1,
        OBJECT_NOT_FOUND = 2,
        NO_FRAME         = 3,
        CAMERA_CLOSED    = 4
    };
    OBJECT_STATUS Detect(const cv::Mat& rFrame, int* piXPx, int* piYPx, std::chrono::steady_clock::time_point* pTime);
    bool                InitDone;
    int                 NumDilation;
    int                 NumErodion;
    cv::Mat             MorphKernel;
    int PuckXPixel;
    int PuckYPixel;
    std::chrono::steady_clock::time_point Time;
    int counter;
};

#endif /* FRAME_ANALYZER_HPP_ */
