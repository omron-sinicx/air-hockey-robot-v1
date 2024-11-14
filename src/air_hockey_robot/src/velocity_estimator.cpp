#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <queue>
#include <cmath>
#include "air_hockey_robot/msg/visual_point.hpp"
#include "air_hockey_robot/msg/visual_points.hpp"
#include "air_hockey_robot/msg/objects_state.hpp"
#include "velocity_estimator.hpp"
#include "air_hockey_robot_config.hpp"

VelocityEstimator::VelocityEstimator(std::string object) : Node(object + "_state")
{
    PointsSubscriber = this->create_subscription<air_hockey_robot::msg::VisualPoints>(object + "_location", rclcpp::QoS(1).best_effort(), std::bind(&VelocityEstimator::VelocityEstimatorCallback, this, std::placeholders::_1));
    StatePublisher = this->create_publisher<air_hockey_robot::msg::ObjectsState>(object + "_state", 1);
    m_dMillimeterPerPixelX = std::abs(m_kdRefPointMillimeterArm1X2 - m_kdRefPointMillimeterArm1X1) / std::abs(m_kdRefPointPixelX2 - m_kdRefPointPixelX1);
    m_dMillimeterPerPixelY = std::abs(-m_kdRefPointMillimeterArm1Y2 - (-m_kdRefPointMillimeterArm2Y1)) / std::abs(m_kdRefPointPixelY2 - m_kdRefPointPixelY1);
    this->puck_old_x  = 0.0;
    this->puck_old_y  = 0.0;
    this->puck_old_s  = 0.0;
    this->human_old_x = 0.0;
    this->human_old_y = 0.0;
    this->human_old_s = 0.0;
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::VelocityEstimatorCallback(air_hockey_robot::msg::VisualPoints::SharedPtr vpoints)
{
    air_hockey_robot::msg::VisualPoint puck  = vpoints->puck;
    air_hockey_robot::msg::VisualPoint human = vpoints->human; // human mallet
    std_msgs::msg::Header Header = vpoints->header;
    std::uint32_t t_now = Header.stamp.sec * (1000000000) + Header.stamp.nanosec; // [ns]
    double puck_x = 0.0;
    double puck_y = 0.0;
    double puck_v = 0.0;
    double puck_w = 0.0;
    double human_x = 0.0;
    double human_y = 0.0;
    double human_v = 0.0;
    double human_w = 0.0;
    if(puck.valid)
    {
        double xy[2];
        ConvertPixelToMillimeterPuck(puck.x, puck.y, xy);
        puck_x = xy[0];
        puck_y = xy[1];
        double puck_x_diff = puck_x - this->puck_old_x;
        double puck_y_diff = puck_y - this->puck_old_y;
        double puck_t_diff = double((t_now - this->puck_old_s) * 1e-9); // [sec]
        this->puck_old_x = puck_x;
        this->puck_old_y = puck_y;
        this->puck_old_s = t_now;
        puck_v = puck_x_diff / puck_t_diff;
        puck_w = puck_y_diff / puck_t_diff;
    }
    if(human.valid)
    {
        double xy[2];
        ConvertPixelToMillimeterHuman(human.x, human.y, xy);
        human_x = xy[0];
        human_y = xy[1];
        double human_x_diff = human_x - this->human_old_x;
        double human_y_diff = human_y - this->human_old_y;
        double human_t_diff = double((t_now - this->human_old_s) * 1e-9); // [sec]
        this->human_old_x = human_x;
        this->human_old_y = human_y;
        this->human_old_s = t_now;
        human_v = human_x_diff / human_t_diff;
        human_w = human_y_diff / human_t_diff;
    }
    air_hockey_robot::msg::ObjectsState objects_state = air_hockey_robot::msg::ObjectsState();
    objects_state.puck.x = float(puck_x);
    objects_state.puck.y = float(puck_y);
    objects_state.puck.v = float(puck_v);
    objects_state.puck.w = float(puck_w);
    objects_state.human.x = float(human_x);
    objects_state.human.y = float(human_y);
    objects_state.human.v = float(human_v);
    objects_state.human.w = float(human_w);
    objects_state.puck.valid = puck.valid;
    objects_state.human.valid = human.valid;
    objects_state.header.stamp = this->get_clock()->now();
    objects_state.header.frame_id = vpoints->header.frame_id;
    StatePublisher->publish(objects_state);
    return;
}

void VelocityEstimator::ConvertPixelToMillimeterPuck(int iXPixel, int iYPixel, double xy[2]) const// make it a const func reseach it again
{
    const double x  = double(iXPixel);
    const double y  = double(iYPixel);
    const double kx_f = PUCK_KX_F;
    const double ky_f = PUCK_KY_F;
    const double xo_f = PUCK_XO_F;
    const double yo_f = PUCK_YO_F;
    const double xo_w = PUCK_XO_W;
    const double yo_w = PUCK_YO_W;
    const double a_w  = PUCK_A_W;
    const double k_w  = PUCK_K_W;
    const double x_f = kx_f * x + xo_f;
    const double y_f = ky_f * y + yo_f;
    const double x2 = k_w*std::cos(a_w)*x_f - k_w*std::sin(a_w)*y_f + xo_w;
    const double y2 = k_w*std::sin(a_w)*x_f + k_w*std::cos(a_w)*y_f + yo_w;
    xy[0] = x2;
    xy[1] = y2;
}

void VelocityEstimator::ConvertPixelToMillimeterHuman(int iXPixel, int iYPixel, double xy[2]) const// make it a const func reseach it again
{
    const double x  = double(iXPixel);
    const double y  = double(iYPixel);
    const double kx_f = HUMAN_KX_F;
    const double ky_f = HUMAN_KY_F;
    const double xo_f = HUMAN_XO_F;
    const double yo_f = HUMAN_YO_F;
    const double xo_w = HUMAN_XO_W;
    const double yo_w = HUMAN_YO_W;
    const double a_w  = HUMAN_A_W;
    const double k_w  = HUMAN_K_W;
    const double x_f = kx_f * x + xo_f;
    const double y_f = ky_f * y + yo_f;
    const double x2 = k_w*std::cos(a_w)*x_f - k_w*std::sin(a_w)*y_f + xo_w;
    const double y2 = k_w*std::sin(a_w)*x_f + k_w*std::cos(a_w)*y_f + yo_w;
    xy[0] = x2;
    xy[1] = y2;
}

double VelocityEstimator::ConvertPixelToMillimeterX(int iXPixel) const// make it a const func reseach it again
{
    double dXmm = m_dMillimeterPerPixelX * (static_cast<double>(iXPixel) - m_kdRefPointPixelX1) + m_kdRefPointMillimeterArm1X1;
    return dXmm;
}

double VelocityEstimator::ConvertPixelToMillimeterY(int iYPixel) const
{
    double dYmm = m_dMillimeterPerPixelY * (static_cast<double>(-iYPixel) + m_kdRefPointPixelY1) + m_kdRefPointMillimeterArm1Y1;
    return dYmm;
}

