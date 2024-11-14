#define _USE_MATH_DEFINES
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <chrono>
#include <queue>
#include <vector>
#include <iostream>
#include "air_hockey_robot_config.hpp"
#include "object_predictor.hpp"
#include "air_hockey_robot/msg/objects_state.hpp"
#include "air_hockey_robot/msg/object_prediction.hpp"

ObjectPredictor::ObjectPredictor() : Node("object_motion_planner")
{
    this->elastic_coefficient = ELASTIC_COEFFICIENT;
    this->prediction_step = PREDICTION_STEP;
    this->sum_radius = SUM_RADIUS;
    this->time_diff = TIME_DIFF;
    this->collide_limit = COLLIDE_LIMIT;
    this->state_subscriber = this->create_subscription<air_hockey_robot::msg::ObjectsState>("object_state", 5, std::bind(&ObjectPredictor::state_callback, this, std::placeholders::_1));
    this->puck_publisher   = this->create_publisher<air_hockey_robot::msg::ObjectPrediction>("puck_prediction", 1);
    this->human_publisher  = this->create_publisher<air_hockey_robot::msg::ObjectPrediction>("human_prediction", 1);
}

ObjectPredictor::~ObjectPredictor()
{
}

void ObjectPredictor::state_callback(const air_hockey_robot::msg::ObjectsState::SharedPtr states)
{
    this->compute_trajectory(states);
    this->puck_publisher->publish(puck_prediction);
    this->human_publisher->publish(human_prediction);
}

// If using std::shared_ptr, always pass by reference. Simply accessing the data points does not affect the reference count.

void ObjectPredictor::compute_trajectory(const air_hockey_robot::msg::ObjectsState::SharedPtr& states)
{
    double puck_x  = double(states->puck.x);
    double puck_y  = double(states->puck.y);
    double puck_v  = double(states->puck.v);
    double puck_w  = double(states->puck.w);
    double human_x = double(states->human.x);
    double human_y = double(states->human.y);
    double human_v = double(states->human.v);
    double human_w = double(states->human.w);
    double t = 0.0;
    std::vector<air_hockey_robot::msg::Trajectory> puck_vector;
    std::vector<air_hockey_robot::msg::Trajectory> human_vector;
    for(int i = 0; i < int(this->prediction_step); ++i)
    {
        bool is_collide_x = false;
        bool is_collide_y = false;
        bool is_collide_m = false;
        double x_wall = 0.0;
        double y_wall = 0.0;
        double puck_x_v  = puck_x  + puck_v  * this->time_diff;
        double puck_y_v  = puck_y  + puck_w  * this->time_diff;
        double puck_v_v  = puck_v;
        double puck_w_v  = puck_w;
        double human_x_v = human_x + human_v * this->time_diff;
        double human_y_v = human_y + human_w * this->time_diff;
        double human_v_v = human_v;
        double human_w_v = human_w;
        double x_collide = puck_x;
        double y_collide = puck_y;
        double v_collide = puck_v;
        double w_collide = puck_w;
        double t_collide = 0.0;
        if(puck_x_v < AIR_HOCKEY_FIELD_MIN_X)
        {
            is_collide_x = true;
            x_wall = AIR_HOCKEY_FIELD_MIN_X;
        }
        if(puck_x_v > AIR_HOCKEY_FIELD_MAX_X)
        {
            is_collide_x = true;
            x_wall = AIR_HOCKEY_FIELD_MAX_X;
        }
        if(puck_y_v < AIR_HOCKEY_FIELD_MIN_Y)
        {
            is_collide_y = true;
            y_wall = AIR_HOCKEY_FIELD_MIN_Y;
        }
        if(puck_y_v > AIR_HOCKEY_FIELD_MAX_Y)
        {
            is_collide_y = true;
            y_wall = AIR_HOCKEY_FIELD_MAX_Y;
        }
        double x_diff_v = puck_x_v - human_x_v;
        double y_diff_v = puck_y_v - human_y_v;
        double l2_v = x_diff_v * x_diff_v + y_diff_v * y_diff_v; 
        if(l2_v < this->sum_radius * this->sum_radius && t < this->collide_limit)
        {
            is_collide_m = true;
        }    
        if(is_collide_x)
        {
            t_collide = std::abs(x_wall - puck_x) / std::abs(puck_v);
            x_collide = puck_x + puck_v * t_collide;
            y_collide = puck_y + puck_w * t_collide;
            v_collide = - puck_v * this->elastic_coefficient;
            w_collide = + puck_w;
        }
        if(is_collide_y)
        {
            t_collide = std::abs(y_wall - puck_y) / std::abs(puck_w);
            x_collide = puck_x + puck_v * t_collide;
            y_collide = puck_y + puck_w * t_collide;
            v_collide = + puck_v;
            w_collide = - puck_w * this->elastic_coefficient;
        }
        if(is_collide_m)
        {
            t_collide = this->detect_collide_time(puck_x, puck_y, puck_v, puck_w, human_x, human_y, human_v, human_w, this->sum_radius);
            v_collide = human_v;
            w_collide = human_w;
            x_collide = puck_x + v_collide * t_collide;
            y_collide = puck_y + w_collide * t_collide;
        }
        if(is_collide_x || is_collide_y || is_collide_m)
        {
            double t_after = this->time_diff - t_collide;
            puck_x = x_collide + v_collide * t_after;
            puck_y = y_collide + w_collide * t_after;
            puck_v = v_collide;
            puck_w = w_collide;
        }
        else
        {
            puck_x = puck_x_v;
            puck_y = puck_y_v;
            puck_v = puck_v_v;
            puck_w = puck_w_v;
        }
        human_x = human_x_v;
        human_y = human_y_v;
        human_v = human_v_v;
        human_w = human_w_v;
        t += this->time_diff;
        int sec = int(t);
        int nanosec = int((t - sec) * 1e+9);
        rclcpp::Time rtime(sec, nanosec);
        air_hockey_robot::msg::Trajectory puck;
        air_hockey_robot::msg::Trajectory human;
        puck.point.x  = puck_x;
        puck.point.y  = puck_y;
        puck.time     = rtime;
        human.point.x = human_x;
        human.point.y = human_y;
        human.time    = rtime;
        puck_vector.push_back(puck);
        human_vector.push_back(human);
    }
    this->puck_prediction.header.stamp  = this->get_clock()->now();
    this->human_prediction.header.stamp = this->get_clock()->now();
    this->puck_prediction.header.frame_id  = states->header.frame_id;
    this->human_prediction.header.frame_id = states->header.frame_id;
    this->puck_prediction.trajectory  = puck_vector;
    this->human_prediction.trajectory = human_vector;
    return;
}

double ObjectPredictor::detect_collide_time(double x0, double y0, double v0, double w0, double x1, double y1, double v1, double w1, double d)
{
    double a = (v0 - v1) * (v0 - v1) + (w0 - w1) * (w0 - w1);
    double b = (x0 - x1) * (v0 - v1) + (y0 - y1) * (w0 - w1);
    double c = (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) - d * d;
    double t = (- b - std::sqrt(b * b - a * c)) / a;
    return t;      
}

