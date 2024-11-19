/**
 * @file RandomExplore.hpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief Header of RandomExplore.cpp
 * @version 0.1
 * @date 2024-11-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <lib/collision_prevention/CollisionPrevention.hpp>
#include <random>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class RandomExplore
{
public:
    RandomExplore();
    ~RandomExplore() = default;

    void performRandomAction();

    // Setters for current state in NED frame from VehicleLocalPosition message
    void setCurrentPosition(float x, float y, float z);
    void setCurrentVelocity(float vx, float vy, float vz);
    void setCurrentYaw(float yaw);

    // Setters for obstacle and attitude messages
    void setObstacleDistance(const ObstacleDistance &msg);
    void setVehicleAttitude(const VehicleAttitude &msg);

    // getters for setpoint and yaw
    void getSetpoint(float &x, float &y, float &z, float &yaw);
    void getCollisionConstraints(CollisionConstraints& msg);
    void getObstacleDistanceFused(ObstacleDistance& msg);

protected:

    void moveHorizontal(float velocity);
    void rotateYaw(float angle);
    void changeAltitude(float delta_z);

    // Random number generators
    std::default_random_engine rand_engine_;
    std::uniform_int_distribution<int> action_type_;
    std::uniform_real_distribution<float> vel_horizontal_;
    std::uniform_real_distribution<float> delta_yaw_;
    std::uniform_real_distribution<float> vel_altitude_;

    enum class Action { MOVE_HORIZONTAL = 0, ROTATE, ASCEND, DESCEND };

    Action getRandomAction();

    struct Position
    {
        float x;
        float y;
        float z;
    } current_position_;

    struct Velocity
    {
        float vx;
        float vy;
        float vz;
    } current_velocity_, setpoint_vel_;

    float current_yaw_;
    float setpoint_yaw_;

    // Subscribed obstacle and attitude messages
    ObstacleDistance::SharedPtr obstacle_distance_msg_;
    VehicleAttitude::SharedPtr vehicle_attitude_msg_;

    // Collision Prevention module
    CollisionPrevention collision_prevention_;
};

