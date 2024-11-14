#ifndef OBJECT_PREDICTOR_HPP_
#define OBJECT_PREDICTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "air_hockey_robot_config.hpp"
#include "object_motion_predictor.hpp"
#include "air_hockey_robot/msg/objects_state.hpp"
#include "air_hockey_robot/msg/object_prediction.hpp"

class ObjectPredictor : public rclcpp::Node
{
public:
    ObjectPredictor();
    ~ObjectPredictor();
private:
    rclcpp::Subscription<air_hockey_robot::msg::ObjectsState>::SharedPtr state_subscriber;
    rclcpp::Publisher<air_hockey_robot::msg::ObjectPrediction>::SharedPtr puck_publisher;
    rclcpp::Publisher<air_hockey_robot::msg::ObjectPrediction>::SharedPtr human_publisher;
    void state_callback(const air_hockey_robot::msg::ObjectsState::SharedPtr states);
    void compute_trajectory(const air_hockey_robot::msg::ObjectsState::SharedPtr& states);
    air_hockey_robot::msg::ObjectPrediction puck_prediction;
    air_hockey_robot::msg::ObjectPrediction human_prediction;
    double elastic_coefficient;
    unsigned int prediction_step;
    double sum_radius;
    double time_diff;
    double collide_limit;
    double detect_collide_time(double x0, double y0, double v0, double w0, double x1, double y1, double v1, double w1, double d);
};

#endif 
