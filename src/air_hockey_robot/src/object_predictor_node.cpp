#include <rclcpp/rclcpp.hpp>
#include "object_predictor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPredictor>());
    rclcpp::shutdown();
    return 0;
}
