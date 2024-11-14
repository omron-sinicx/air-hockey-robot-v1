#ifndef VELOCITY_ESTIMATOR_HPP_
#define VELOCITY_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include "air_hockey_robot/msg/visual_points.hpp"
#include "air_hockey_robot/msg/objects_state.hpp"

class VelocityEstimator : public rclcpp::Node
{
public:
    VelocityEstimator(std::string object);
    ~VelocityEstimator();
private:
    void VelocityEstimatorCallback(air_hockey_robot::msg::VisualPoints::SharedPtr vpoints);
    rclcpp::Subscription<air_hockey_robot::msg::VisualPoints>::SharedPtr PointsSubscriber;
    rclcpp::Publisher<air_hockey_robot::msg::ObjectsState>::SharedPtr StatePublisher;
    double m_dMillimeterPerPixelX; // make this a constant, only use constexpr constants to calculate so should be posible
    double m_dMillimeterPerPixelY;
    double ConvertPixelToMillimeterX(int iXPixel) const;
    double ConvertPixelToMillimeterY(int iYPixel) const;
    void ConvertPixelToMillimeterPuck(int iXPixel, int iYPixel, double xy[2]) const;
    void ConvertPixelToMillimeterHuman(int iXPixel, int iYPixel, double xy[2]) const;
    double puck_old_x, puck_old_y, human_old_x, human_old_y;
    std::uint32_t puck_old_s, human_old_s;
};

// INCLUDE <cstdint> where needed!! also maybe chrono too
#endif /* PUCK_MOVEMENT_ANALYZER_HPP_ */

