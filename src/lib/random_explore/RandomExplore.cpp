/**
 * @file RandomExplore.cpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief 
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <lib/random_explore/RandomExplore.hpp>
#include <functional>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using namespace matrix;

RandomExplore::RandomExplore() :
    rand_engine_(std::random_device{}()),
    action_type_(0, 2), 
    vel_horizontal_(-1.5, 1.5), // horizontal acceleration magnitude
    vel_direction_(-135.0, 135.0), // horizontal acceleration direction in earth-fixed NED frame
    // vel_altitude_(0.2, 0.5), // z direction velocity in ned frame
	logger_(rclcpp::get_logger("RandomExplore"))
{
    // initialize drone states 
    setpoint_vel_ = {0.0, 0.0, 0.0};
    current_velocity_ = {0.0, 0.0, 0.0};
    current_yaw_ = 0.0;
}

/**
 * @brief for testing collision prevention
 * 
 */
void RandomExplore::goStraight()
{
    setpoint_vel_.vx = -0.5;
    setpoint_vel_.vy = 0.0;
    setpoint_vel_.vz = 0.0;

    setpoint_yaw_ = 180.0;
}

void RandomExplore::performRandomAction()
{
    action = getRandomAction();
    // action is given in ned frame
    switch (action)
    {
        case Action::MOVE_HORIZONTAL:
        {   
            float velocity = vel_horizontal_(rand_engine_);
            float direction = vel_direction_(rand_engine_);
            moveHorizontal(velocity, direction);
            RCLCPP_INFO(logger_, "Drone starts to move horizontally with vx: %f and vy: %f", setpoint_vel_.vx, setpoint_vel_.vy);
            break;
        }
        case Action::ROTATE:
        {
            rotateYaw();
            RCLCPP_INFO(logger_, "Drone is rotating along z axis");
            break;
        }
        case Action::MOVE_FORWARD:
        {
            float velocity = vel_horizontal_(rand_engine_);
            moveForward(velocity);
            if (velocity >= 0) 
            {
                RCLCPP_INFO(logger_, "Drone is moving forward with vx: %f and vy: %f", setpoint_vel_.vx, setpoint_vel_.vy);
            } else
            {
                RCLCPP_INFO(logger_, "Drone is moving backward with vx: %f and vy: %f", setpoint_vel_.vx, setpoint_vel_.vy);
            }
        }
        // case Action::ASCEND:
        // {
        //     float vz = -vel_altitude_(rand_engine_); // Negative because NED frame
        //     changeAltitude(vz);
        //     break;
        // }
        // case Action::DESCEND:
        // {
        //     float vz = vel_altitude_(rand_engine_);
        //     changeAltitude(vz);
        //     break;
        // }
    }
}

void RandomExplore::moveHorizontal(float velocity, float direction_body_frame)
{   
    // Ensure minimum velocity magnitude
    if (std::abs(velocity) < 0.5f) {
        velocity = 0.5f * (velocity < 0 ? -1 : 1); // Preserve the direction (negative or positive)
    }

    // Compute the direction angle in the earth-fixed NED frame
    float direction_ned_frame = current_yaw_ + direction_body_frame;
    float direction_ned_frame_rad = direction_ned_frame * M_PI / 180.0;

    // Compute the velocity components in the earth-fixed NED frame
    setpoint_vel_.vx = velocity * cos(direction_ned_frame_rad);
    setpoint_vel_.vy = velocity * sin(direction_ned_frame_rad);
    setpoint_vel_.vz = 0.0;

    // Maintain the current yaw in degree
    setpoint_yaw_ = current_yaw_;
}

void RandomExplore::moveForward(float velocity)
{   
    // Ensure minimum velocity magnitude
    if (std::abs(velocity) < 0.5f) {
        velocity = 0.5f * (velocity < 0 ? -1 : 1); // Preserve the direction (negative or positive)
    }

    // computing xy velocities in earth fixed ned frame
    float yaw_rad = current_yaw_ * M_PI / 180.0;
    setpoint_vel_.vx = velocity * cos(yaw_rad);
    setpoint_vel_.vy = velocity * sin(yaw_rad);
    setpoint_vel_.vz = 0.0;
    setpoint_yaw_ = current_yaw_;
}

void RandomExplore::rotateYaw()
{
    // velocities are 0 when spinning
    setpoint_vel_.vx = 0.0;
    setpoint_vel_.vy = 0.0;
    setpoint_vel_.vz = 0.0;

    setpoint_yaw_ = std::numeric_limits<float>::quiet_NaN();
}

void RandomExplore::changeAltitude(float vel_z)
{
    setpoint_vel_.vx = 0.0;
    setpoint_vel_.vy = 0.0;
    setpoint_vel_.vz = vel_z;
    setpoint_yaw_ = current_yaw_;
}

RandomExplore::Action RandomExplore::getRandomAction()
{
    int action = action_type_(rand_engine_);
    return static_cast<Action>(action);
}

void RandomExplore::setCurrentVelocity(const float vx, const float vy, const float vz)
{   
    // NED eath fixed frame
    current_velocity_.vx = vx;
    current_velocity_.vy = vy;
    current_velocity_.vz = vz;
}

void RandomExplore::setCurrentYaw(const float yaw)
{
    current_yaw_ = yaw;

    // Constrain yaw to -180° to 180°
    if (current_yaw_ > 180.0)
        current_yaw_ -= 360.0;
    else if (current_yaw_ < -180.0)
        current_yaw_ += 360.0;
}

// Getters for setpoint in NED frame
void RandomExplore::getSetpoint(float &vx, float &vy, float &vz, float &yaw)
{
    vx = setpoint_vel_.vx;
    vy = setpoint_vel_.vy;
    vz = setpoint_vel_.vz;

    yaw = setpoint_yaw_;
    if (!std::isnan(yaw)) {
        yaw = yaw * M_PI / 180.0; // Convert to radians
    }
}

void RandomExplore::getActionType(Action &act_typ)
{
    act_typ = action;
}