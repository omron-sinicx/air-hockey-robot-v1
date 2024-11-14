#ifndef AIR_HOCKEY_ROBOT_HPP_
#define AIR_HOCKEY_ROBOT_HPP_

#include <cstdint>
#include <string>
#include <limits>

/* OBJECT DETECTOR*/
static constexpr double ELASTIC_COEFFICIENT = 0.7;
static constexpr int PREDICTION_STEP = 60;
static constexpr double SUM_RADIUS = 18.0 + 25.0;
static constexpr double TIME_DIFF = 1.0 / 120.0;
static constexpr double COLLIDE_LIMIT = 0.4;

/* CAMERA */
static constexpr std::array<std::array<double, 3>, 3> CAMERA_MATRIX = {{
        {{470.267371, 0.000000, 327.281264}},
        {{0.000000, 471.436145, 235.528228}},
        {{0.000, 0.00, 1.000000}}
    }};
static constexpr std::array<double, 5> DISTORTION_COEFFICIENTS = {-0.389809, 0.141177, 0.001647, -0.000438, 0.000000};

/* ARM */
static constexpr int    ARM1_LINK1_LENGTH              {200}; // 189
static constexpr int    ARM1_LINK2_LENGTH              {200}; // 158
// static constexpr int    ARM1_LINK1_LENGTH              {189}; // 189
// static constexpr int    ARM1_LINK2_LENGTH              {158}; // 158
static constexpr int    ARM2_LINK1_LENGTH              {290};
static constexpr int    ARM2_LINK2_LENGTH              {261};
static constexpr double ARM1_DEFAULT_POSITION_XMM      {245.0};
static constexpr double ARM1_DEFAULT_POSITION_YMM      {0.0};
static constexpr double ARM1_PARK_THE_BUS_POSITION_XMM {180.0};
static constexpr double ARM1_PARK_THE_BUS_POSITION_YMM {0.0};
static constexpr double ARM2_DEFAULT_POSITION_XMM      {335.0};
static constexpr double ARM2_DEFAULT_POSITION_YMM      {0.0};
static constexpr int    DEFENCE_LINE_XMM               {235}; //215
static constexpr int    ATTACK_LINE_XMM                {320}; //285
static constexpr double ARM_MIN_XMM                    {200.0};
static constexpr double ARM_MAX_XMM                    {390.0}; // 326
static constexpr double ARM_MIN_YMM                    {-107.0};
static constexpr double ARM_MAX_YMM                    {107.0};
static constexpr double ARM1_OWN_HALF                  {360.0}; // 360

/* PUCK */
static constexpr double       MAXIMUM_PUCK_MOVABLE_BOUNDRY_YMM {120.0};
static constexpr double       MINIMUM_PUCK_MOVABLE_BOUNDRY_YMM {-120.0};
static constexpr unsigned int MAX_PUCK_REBOUND_NUM             {10};

static constexpr int MIN_OBJECT_AREA {200}; // 200
static constexpr int MAX_OBJECT_AREA {10000}; // 2500
static constexpr int MIN_PUCK_AREA   {200}; // 200
static constexpr int MAX_PUCK_AREA   {10000}; // 2500
static constexpr int MIN_HSV_H       {0};
static constexpr int MIN_HSV_S       {0};
static constexpr int MIN_HSV_V       {0};
static constexpr int MAX_HSV_H       {179};
static constexpr int MAX_HSV_S       {255};
static constexpr int MAX_HSV_V       {255};
static constexpr int EXPANSION_HSV_H {3};
static constexpr int EXPANSION_HSV_S {15};
static constexpr int EXPANSION_HSV_V {15};

static constexpr double AIR_HOCKEY_FIELD_MIN_X = 106.0;
static constexpr double AIR_HOCKEY_FIELD_MAX_X = 532.0;
static constexpr double AIR_HOCKEY_FIELD_MIN_Y = -130.0;
static constexpr double AIR_HOCKEY_FIELD_MAX_Y = 110.0;

// HSV parameters for PUCK
static constexpr int PUCK_MIN_HSV_H       {140};
static constexpr int PUCK_MIN_HSV_S       {150};
static constexpr int PUCK_MIN_HSV_V       {20};
static constexpr int PUCK_MAX_HSV_H       {180};
static constexpr int PUCK_MAX_HSV_S       {255};
static constexpr int PUCK_MAX_HSV_V       {255};

// HSV parameters for PUCK
static constexpr int HUMAN_MIN_HSV_H       {70};
static constexpr int HUMAN_MIN_HSV_S       {50};
static constexpr int HUMAN_MIN_HSV_V       {20};
static constexpr int HUMAN_MAX_HSV_H       {100};
static constexpr int HUMAN_MAX_HSV_S       {255};
static constexpr int HUMAN_MAX_HSV_V       {255};

// HSV parameters for PUCK
static constexpr int ROBOT_MIN_HSV_H       {40};
static constexpr int ROBOT_MIN_HSV_S       {50};
static constexpr int ROBOT_MIN_HSV_V       {20};
static constexpr int ROBOT_MAX_HSV_H       {60};
static constexpr int ROBOT_MAX_HSV_S       {255};
static constexpr int ROBOT_MAX_HSV_V       {255};

/* VELOCITY */
const double PUCK_KX_F = -0.9646568382714151;
const double PUCK_KY_F = 0.8627463308062775;
const double PUCK_XO_F = 463.5249058497726;
const double PUCK_YO_F = -119.912059942736;
const double PUCK_XO_W = 219.24552722834164;
const double PUCK_YO_W = -96.48139885204944;
const double PUCK_A_W  = -0.027177768306392958;
const double PUCK_K_W  = 0.9803587443660582;

const double HUMAN_KX_F = -0.8554054441734112;
const double HUMAN_KY_F = 0.7722444752542543;
const double HUMAN_XO_F = 427.2807561329786;
const double HUMAN_YO_F = -99.22479635742127;
const double HUMAN_XO_W = 219.24552722834164;
const double HUMAN_YO_W = -96.48139885204944;
const double HUMAN_A_W  = -0.027177768306392958;
const double HUMAN_K_W  = 0.9803587443660582;

static constexpr int    m_kiQueueSize {3}; // uint or something
static constexpr double m_kdFlickerTolerance {5.0};
static constexpr double m_kdRefPointPixelX1 {50.0}; // this should be measurable during run time
static constexpr double m_kdRefPointPixelY1 {95.0};
static constexpr double m_kdRefPointPixelX2 {583.0};
static constexpr double m_kdRefPointPixelY2 {379.0};
static constexpr double m_kdRefPointMillimeterArm1X1 {180.0};
static constexpr double m_kdRefPointMillimeterArm1Y1 {120.0};
static constexpr double m_kdRefPointMillimeterArm1X2 {620.0};
static constexpr double m_kdRefPointMillimeterArm1Y2 {-120.0};
static constexpr double m_kdRefPointMillimeterArm2X1 {300.0};
static constexpr double m_kdRefPointMillimeterArm2Y1 {120.0};
static constexpr double m_kdRefPointMillimeterArm2X2 {740.0};
static constexpr double m_kdRefPointMillimeterArm2Y2 {-120.0};
static constexpr int m_kiImageWidth {640};
static constexpr int m_kiImageHeight {480}; // make this global and only have one variable defined


/* DYNAMIXEL MOTOR */
// static constexpr double        DYNAMIXEL_PROTOCOL_VERSION                     {2.0};
// static constexpr unsigned int  MIN_DYNAMIXEL_MOTOR_DEGREE                     {70};
// static constexpr unsigned int  MAX_DYNAMIXEL_MOTOR_DEGREE                     {290};
// static constexpr unsigned int  DEFAULT_DYNAMIXEL_POSITIVE_MOTOR_DEGREE        {180};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPEED_MAX               {0x0000};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPEED_FULL              {0x03FF};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPEED_HALF              {0x01FF};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPEED_QUARTER           {0x00FF};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPEED_ONE_EIGHTH        {0x007F};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPEED_ONE_SIXTEENTH     {0x003F};
// static constexpr std::uint16_t DYNAMIXEL_MOTOR_MOVING_SPPED_ONE_THIRTY_SECOND {0x001F};
// static constexpr std::uint16_t ADDR_MX_CW_ANGLE_LIMIT                         {6};
// static constexpr std::uint16_t ADDR_MX_CCW_ANGLE_LIMIT                        {8};
// static constexpr std::uint16_t ADDR_MX_TORQUE_ENABLE                          {24};
// static constexpr std::uint16_t ADDR_MX_GOAL_POSITION                          {30};
// static constexpr std::uint16_t ADDR_MX_MOVING_SPEED                           {32};
// static constexpr std::uint16_t ADDR_MX_PRESENT_POSITION                       {36};

static constexpr double        DYNAMIXEL_PROTOCOL_VERSION                     {2.0};
static constexpr std::uint16_t DEFAULT_DYNAMIXEL_MOTOR_MOVING_SPEED_ID1       {0x7FFF}; // 0
static constexpr std::uint16_t DEFAULT_DYNAMIXEL_MOTOR_MOVING_SPEED_ID2       {0x7FFF}; // 0
static constexpr std::uint32_t MAX_DYNAMIXEL_MOTOR_POSITION                   {4095}; // meaningless
static constexpr std::uint32_t MIN_DYNAMIXEL_MOTOR_POSITION                   {0}; // meaning less make the program set operating mode
static constexpr std::uint16_t ADDR_DYNAMIXEL_MIN_POSITION_LIMIT              {52};
static constexpr std::uint16_t ADDR_DYNAMIXEL_MAX_POSITION_LIMIT              {48};
static constexpr std::uint16_t ADDR_DYNAMIXEL_TORQUE_ENABLE                   {64};
static constexpr std::uint16_t ADDR_DYNAMIXEL_MOVING_STATUS                   {123};
static constexpr std::uint16_t ADDR_DYNAMIXEL_GOAL_POSITION                   {116};
static constexpr std::uint16_t ADDR_DYNAMIXEL_PROFILE_VELOCITY                {112};
static constexpr std::uint16_t ADDR_DYNAMIXEL_PRESENT_POSITION                {132};

static constexpr double        DELTA_POSITION_TOLERANCE                       {25.0};
static constexpr double        DYNAMIXEL_PROFILE_VELOCITY_UNIT_IN_RPM         {0.229};
static constexpr int           BAUDRATE                                       {57600};
// static constexpr int           BAUDRATE                                       {1000000};
enum class DYNAMIXEL_MOTOR_ID : std::uint8_t
{
    ID1 = 1,
    ID2 = 2 // only params here
};
enum class STRATEGY : std::int8_t
{
    DEFAULT      = 0,
    PARK_THE_BUS = 1,
    BLOCK        = 2,
    REGULAR_ATTACK    = 3,
    FULL_SWING_ATTACK = 4,
    SWEEP        = 5
};

enum class GAME_STRATEGY : std::int8_t
{
    DEFAULT        = 0,
    BLOCK          = 1,
    CONTROL        = 2,
    REGULAR_ATTACK = 3,
    SNIPE_ATTACK   = 4,
    SWEEP          = 5
};

struct Point
{
    double x;
    double y;
    Point(double x = 0.0, double y = 0.0)
    {
        this->x = x;
        this->y = y;
    }
};

struct LineSegment
{
    Point Start;
    Point End;
    LineSegment(Point Start = Point(), Point End = Point())
    {
        this->Start = Start;
        this->End   = End;
    }
};

#endif /* AIR_HOCKEY_ROBOT_CONFIG_HPP_ */

// maybe change PuckVector to Roi Vector?????
