#define _USE_MATH_DEFINES
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
#include <chrono>

#include "air_hockey_robot_config.hpp"
#include "arm_motion_planner.hpp"
#include "game_strategy.hpp" // rename this GameStrategy

// weight strategy

ArmMotionPlanner::ArmMotionPlanner() : Node("arm_motion_planner")
{
    this->PuckVectorSubscriber = this->create_subscription<air_hockey_robot::msg::ObjectMotionPrediction>("puck_vector", 10, std::bind(&ArmMotionPlanner::ArmMotionPlannerCallback, this, std::placeholders::_1));
    this->GameStrategy = this->create_publisher<air_hockey_robot::msg::GameStatus>("game_status", rclcpp::QoS(1).best_effort());
}

ArmMotionPlanner::~ArmMotionPlanner()
{
}

void ArmMotionPlanner::ArmMotionPlannerCallback(const air_hockey_robot::msg::ObjectMotionPrediction::SharedPtr puck_motion_prediction) // put const like this so that pointer is not modified
{
    GAME_STRATEGY game_strategy = STRATEGY::DEFAULT;
    
}

bool ArmMotionPlanner::snipe_attack()
{

}

// average of three decisions, if past one point do not change decision until it hits the other end...
