#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "object_detector.hpp"
#include "air_hockey_robot_config.hpp"
#include "air_hockey_robot/msg/visual_points.hpp"
#include <string>

ObjectDetector::ObjectDetector(std::string object_name): Node(object_name + "_analyzer") // Detector, not analyzer
{
    this->ImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>("usb_camera_image", 10, std::bind(&ObjectDetector::ObjectDetectorCallback, this, std::placeholders::_1));
    this->PointsPublisher = this->create_publisher<air_hockey_robot::msg::VisualPoints>(object_name + "_location", rclcpp::QoS(1).best_effort());
    this->InitDone = false;
    this->MorphKernel   = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    this->PuckXPixel = 0;
    this->PuckYPixel = 0;
    this->NumDilation = 1;
    this->NumErodion = 1;
    CurrentDisplayMode = static_cast<int>(DISPLAY_MODE::IDLE);
    this->object_name = object_name;
    this->detected_p = 0;
    this->detected_h = 0;
    this->detected_r = 0;
    this->not_detected_p = 0;
    this->not_detected_h = 0;
    this->not_detected_r = 0;
    this->counter = 0;
}

ObjectDetector::~ObjectDetector()
{
}

void ObjectDetector::ObjectDetectorCallback(sensor_msgs::msg::Image::SharedPtr pImage)
{
    cv_bridge::CvImageConstPtr CvImage = cv_bridge::toCvShare(pImage, pImage->encoding);
    if(CvImage->image.empty())
    {
        return;
    }
    air_hockey_robot::msg::VisualPoints Points = air_hockey_robot::msg::VisualPoints();
    std_msgs::msg::Header header = Points.header;
    // Puck    
    this->min_hsv = cv::Scalar(PUCK_MIN_HSV_H, PUCK_MIN_HSV_S, PUCK_MIN_HSV_V);
    this->max_hsv = cv::Scalar(PUCK_MAX_HSV_H, PUCK_MAX_HSV_S, PUCK_MAX_HSV_V);
    if (Detect(CvImage->image, &this->PuckXPixel, &this->PuckYPixel, &this->Time) == OBJECT_STATUS::OK)
    {
        detected_p += 1;
        Points.puck.x = this->PuckXPixel;
        Points.puck.y = this->PuckYPixel;
        Points.puck.valid = true;
    }
    else
    {
        not_detected_p += 1;
        Points.puck.x = 0;
        Points.puck.y = 0;
        Points.puck.valid = false;
    }
    // Human   
    this->min_hsv = cv::Scalar(HUMAN_MIN_HSV_H, HUMAN_MIN_HSV_S, HUMAN_MIN_HSV_V);
    this->max_hsv = cv::Scalar(HUMAN_MAX_HSV_H, HUMAN_MAX_HSV_S, HUMAN_MAX_HSV_V);
    if (Detect(CvImage->image, &this->PuckXPixel, &this->PuckYPixel, &this->Time) == OBJECT_STATUS::OK)
    {
        detected_h += 1;
        Points.human.x = this->PuckXPixel;
        Points.human.y = this->PuckYPixel;
        Points.human.valid = true;
    }
    else
    {
        not_detected_h += 1;
        Points.human.x = 0;
        Points.human.y = 0;
        Points.human.valid = false;
    }
    // Robot
    this->min_hsv = cv::Scalar(ROBOT_MIN_HSV_H, ROBOT_MIN_HSV_S, ROBOT_MIN_HSV_V);
    this->max_hsv = cv::Scalar(ROBOT_MAX_HSV_H, ROBOT_MAX_HSV_S, ROBOT_MAX_HSV_V);
    if (Detect(CvImage->image, &this->PuckXPixel, &this->PuckYPixel, &this->Time) == OBJECT_STATUS::OK)
    {
        detected_r += 1;
        Points.robot.x = this->PuckXPixel;
        Points.robot.y = this->PuckYPixel;
        Points.robot.valid = true;
    }
    else
    {
        not_detected_r += 1;
        Points.robot.x = 0;
        Points.robot.y = 0;
        Points.robot.valid = false;
    }
    Points.header.stamp = this->now();
    Points.header.frame_id = std::to_string(int(this->counter));
    this->PointsPublisher->publish(Points);
    this->counter = this->counter + 1;
}

ObjectDetector::OBJECT_STATUS ObjectDetector::Detect(const cv::Mat& Frame, int* XPixel, int* YPixel, std::chrono::steady_clock::time_point* Time)
{
    try
    {
        if(XPixel == nullptr || YPixel == nullptr || Time == nullptr)
        {
            return OBJECT_STATUS::EXCEPTION_ERROR;
        }
        cv::Mat HsvFrame;
        cv::Mat MaskFrame;
        cv::Mat MorpFrame;
        cv::Mat LabelFrame;
        if(Frame.empty())
        {
            *XPixel = *YPixel = 0;
            return OBJECT_STATUS::NO_FRAME;
        }
        cv::Mat Stats;
        cv::Mat Centroids;
        int     MaxArea = 0;
        int     Cx, Cy;
        *Time = std::chrono::steady_clock::now();
        cv::cvtColor(Frame, HsvFrame, cv::COLOR_BGR2HSV);
        cv::inRange(HsvFrame, this->min_hsv, this->max_hsv, MaskFrame);
        MorpFrame = MaskFrame;
        // Morphological Transformations (noise reduction)
        for(int i = 0; i < this->NumErodion; i++)
        {
            cv::erode(MorpFrame, MorpFrame, this->MorphKernel);
        }
        for(int i = 0; i < this->NumDilation; i++) {
            cv::dilate(MorpFrame, MorpFrame, this->MorphKernel);
        }
        int Label = cv::connectedComponentsWithStats(MorpFrame, LabelFrame, Stats, Centroids);
        for(int i = 1; i < Label; i++)
        {
            int    *Sparam = Stats.ptr<int>(i);
            int    Area    = Sparam[cv::ConnectedComponentsTypes::CC_STAT_AREA];
            double *Cparam = Centroids.ptr<double>(i);
            int    CxTmp   = static_cast<int>(Cparam[0]);
            int    CyTmp   = static_cast<int>(Cparam[1]);
            if(Area > MaxArea)
            {
                Cx = CxTmp;
                Cy = CyTmp;
                MaxArea = Area;
            }
        }
        if(MaxArea < MIN_OBJECT_AREA || MaxArea > MAX_OBJECT_AREA)
        {
            return OBJECT_STATUS::OBJECT_NOT_FOUND;
        }
        *XPixel = Cx;
        *YPixel = Cy;
        return OBJECT_STATUS::OK;
    }
    catch(const std::exception& e)
    {
        *XPixel = *YPixel = 0;
        return OBJECT_STATUS::EXCEPTION_ERROR;
    }
}

int ObjectDetector::CurrentDisplayMode;

