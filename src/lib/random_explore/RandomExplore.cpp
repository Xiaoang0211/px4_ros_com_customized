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
 * 3. vehicle_command, which is for publishing the new set point for the random walk.
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

RandomExplore::RandomExplore() :
    rand_engine_(std::random_device{}()),
    action_type_(0, 3),
    vel_horizontal_(-0.5, 0.5), // horizontal velocity in ned frame
    delta_yaw_(-45.0, 45.0), // change of yaw angle in body frame
    vel_altitude_(0.2, 0.5) // z direction velocity in ned frame
{
    // initialize drone states 
    current_position_ = {0.0, 0.0, 0.0};
    setpoint_vel_ = {0.0, 0.0, 0.0};
    current_velocity_ = {0.0, 0.0, 0.0};
    current_yaw_ = 0.0;
    collision_prevention_ = CollisionPrevention();
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
            moveHorizontal(velocity);
            // Adjust setpoint regarding collision prevention
            matrix::Vector2f setpoint_vel_xy = {setpoint_vel_.vx, setpoint_vel_.vy};
            matrix::Vector2f current_xy = {current_position_.x, current_position_.y};
            matrix::Vector2f current_vel_xy = {current_velocity_.vx, current_velocity_.vy};
            float max_vel = 0.5;
            collision_prevention_.modifySetpoint(setpoint_vel_xy, max_vel, current_xy, current_vel_xy);
            // assign the modified horizontal velocity to setpoint_vel_
            setpoint_vel_.vx = setpoint_vel_xy(0);
            setpoint_vel_.vy = setpoint_vel_xy(1);
            break;
        }
        case Action::ROTATE:
        {
            float angle = delta_yaw_(rand_engine_);
            rotateYaw(angle);
            break;
        }
        case Action::ASCEND:
        {
            float vz = -vel_altitude_(rand_engine_); // Negative because NED frame
            changeAltitude(vz);
            break;
        }
        case Action::DESCEND:
        {
            float vz = vel_altitude_(rand_engine_);
            changeAltitude(vz);
            break;
        }

    }
}

void RandomExplore::moveHorizontal(float velocity)
{
    float yaw_rad = current_yaw_ * M_PI / 180.0;
    setpoint_vel_.vx = velocity * cos(yaw_rad);
    setpoint_vel_.vy = velocity * sin(yaw_rad);
    setpoint_vel_.vz = 0.0;
    setpoint_yaw_ = current_yaw_;
    // collision prevention is activated when moving in xy horizontal plain
}

void RandomExplore::rotateYaw(float delta_yaw)
{
    current_yaw_ += delta_yaw; // In NED frame
    // Constrain current_yaw_ to -180° to 180°
    if (current_yaw_ > 180.0)
        current_yaw_ -= 360.0;
    else if (current_yaw_ < -180.0)
        current_yaw_ += 360.0;

    // velocities are 0 when rotating
    setpoint_vel_.vx = 0.0;
    setpoint_vel_.vy = 0.0;
    setpoint_vel_.vz = 0.0;

    setpoint_yaw_ = current_yaw_;
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

/**
 * @brief 
 * 
 * @param x, y, z message obtained from the ros topic VehicleLocalPosition
 */
void RandomExplore::setCurrentPosition(float x, float y, float z)
{   
    // NED earth fixed frame
    current_position_.x = x;
    current_position_.y = y;
    current_position_.z = z;
}

void RandomExplore::setCurrentVelocity(float vx, float vy, float vz)
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

// Setters for subscribed messages, which should used by Offboard Control node
void RandomExplore::setObstacleDistance(const ObstacleDistance &msg)
{
    collision_prevention_.setObstacleDistance(msg);
}

void RandomExplore::setVehicleAttitude(const VehicleAttitude &msg)
{
    collision_prevention_.setVehicleAttitude(msg);
}

// Getters for setpoint in NED frame
void RandomExplore::getSetpointAndYaw(float &vx, float &vy, float &vz, float &yaw)
{
    vx = setpoint_vel_.vx;
    vy = setpoint_vel_.vy;
    vz = setpoint_vel_.vz;

    yaw = setpoint_yaw_;
}

void RandomExplore::getCollisionConstraints(CollisionConstraints& msg)
{
    collision_prevention_.getCollisionConstraints(msg);
}

void RandomExplore::getObstacleDistanceFused(ObstacleDistance& msg)
{
    collision_prevention_.getObstacleDistanceFused(msg);
}