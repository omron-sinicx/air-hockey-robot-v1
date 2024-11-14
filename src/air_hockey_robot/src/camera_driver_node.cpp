#include <rclcpp/rclcpp.hpp>
#include "camera_driver.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDriver>());
    rclcpp::shutdown();
    return 0;
}
