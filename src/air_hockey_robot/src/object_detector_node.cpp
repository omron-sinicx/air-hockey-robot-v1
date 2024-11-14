#include <rclcpp/rclcpp.hpp>
#include "object_detector.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>("object"));
    rclcpp::shutdown();
    return 0;
}
