#include <rclcpp/rclcpp.hpp>
#include "velocity_estimator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityEstimator>("object"));
    rclcpp::shutdown();
    return 0;
}
