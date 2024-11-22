/**
 * @file RandomExplore.cpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief This random exploration algorithm is proposed for the early stage of exploring new 
 * environment as the point cloud obtained by VI SLAM is still too sparse. For safety
 * issue, we leverage the collision prevention lib for avoiding obstaclees in the horizontal plain. 
 * 
 * The class RandomExplore is implemented as a ROS node to publish the following topics:
 * 1. obstacle_distance_fused, which is used for updating the obstacle map.
 * 2. collision_constraints
 * 
 * To successfully publish these topics, RandomExplore requires the following subscriptions:
 * 1. obstacle_distance, which is the latest obstacle map captured by the 2D LiDAR
 * 2. vehicle_attitude, which has the quaternion rotation from the FRD body frame to the NED earth frame
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
    vel_horizontal_(-2.0, 2.0), // horizontal velocity in ned frame
    vel_direction_(-60.0, 60.0),
    delta_yaw_(-90.0, 90.0), // change of yaw angle in body frame
    // vel_altitude_(0.2, 0.5), // z direction velocity in ned frame
	logger_(rclcpp::get_logger("RandomExplore"))
{
    // initialize drone states 
    setpoint_vel_ = {0.0, 0.0, 0.0};
    current_velocity_ = {0.0, 0.0, 0.0};
    current_yaw_ = 0.0;
}

void RandomExplore::performRandomAction()
{
    Action action = getRandomAction();

    // action is given in ned frame
    switch (action)
    {
        case Action::MOVE_HORIZONTAL:
        {
            float velocity = vel_horizontal_(rand_engine_);
            float direction = vel_direction_(rand_engine_);
            moveHorizontal(velocity, direction);
            RCLCPP_INFO(logger_, "Drone is moving in horizontal plain with vx: %f and vy: %f", setpoint_vel_.vx, setpoint_vel_.vy);
            break;
        }
        case Action::ROTATE:
        {
            float angle = delta_yaw_(rand_engine_);
            rotateYaw(angle);
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

void RandomExplore::moveHorizontal(float velocity, float direction_angle_deg_body)
{
    // Convert yaw and body frame direction angle from degrees to radians
    float yaw_rad = current_yaw_ * M_PI / 180.0;
    float direction_angle_rad_body = direction_angle_deg_body * M_PI / 180.0;

    // Compute the direction angle in the earth-fixed NED frame
    float direction_angle_rad_ned = yaw_rad + direction_angle_rad_body;

    // Compute the velocity components in the earth-fixed NED frame
    setpoint_vel_.vx = velocity * cos(direction_angle_rad_ned);
    setpoint_vel_.vy = velocity * sin(direction_angle_rad_ned);
    setpoint_vel_.vz = 0.0;

    // Maintain the current yaw or update it if required
    setpoint_yaw_ = current_yaw_; // Adjust based on application needs

    // Collision prevention is activated when moving in the xy horizontal plane
}

void RandomExplore::moveForward(float velocity)
{   
    // computing xy velocities in earth fixed ned frame
    float yaw_rad = current_yaw_ * M_PI / 180.0;
    setpoint_vel_.vx = velocity * cos(yaw_rad);
    setpoint_vel_.vy = velocity * sin(yaw_rad);
    setpoint_vel_.vz = 0.0;
    setpoint_yaw_ = current_yaw_;
    // collision prevention is activated when moving in xy horizontal plain
}

void RandomExplore::rotateYaw(float delta_yaw)
{
    setpoint_yaw_ = current_yaw_ + delta_yaw; // In NED frame
    // Constrain current_yaw_ to -180° to 180°
    if (setpoint_yaw_ > 180.0)
        setpoint_yaw_ -= 360.0;
    else if (setpoint_yaw_ < -180.0)
        setpoint_yaw_ += 360.0;

    // velocities are 0 when rotating
    setpoint_vel_.vx = 0.0;
    setpoint_vel_.vy = 0.0;
    setpoint_vel_.vz = 0.0;
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
    yaw = yaw * M_PI / 180.0; // Convert to radians
}