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

#include <float.h>
#include <chrono>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/math.hpp>
#include <limits>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class RandomExplore
{
public:
    RandomExplore();
    ~RandomExplore() = default;

    // enum class Action { MOVE_HORIZONTAL = 0, ROTATE, ASCEND, DESCEND };
    enum class Action { MOVE_HORIZONTAL = 0, ROTATE, MOVE_FORWARD };

    void performRandomAction();
    void goStraight();

    // Setters for current state in NED frame from VehicleLocalPosition message
    void setCurrentPosition(const float x, const float y, const float z);
    void setCurrentVelocity(const float vx, const float vy, const float vz);
    void setCurrentYaw(const float yaw);

    // Setter for obstacle message
    void setObstacleDistance(const ObstacleDistance &msg);

    // getters for setpoint and yaw
    void getSetpoint(float &vx, float &vy, float &vz, float &yaw);
    void getActionType(Action &act_typ);
    
    
protected:

    // action functions
    void moveHorizontal(float velocity, float direction);
    void moveForward(float velocity);
    void rotateYaw();
    void changeAltitude(float delta_z);

    // Random number generators
    std::default_random_engine rand_engine_;
    std::uniform_int_distribution<int> action_type_;
    std::uniform_real_distribution<float> vel_horizontal_;
    std::uniform_real_distribution<float> vel_direction_;
    std::uniform_real_distribution<float> delta_yaw_;
    // std::uniform_real_distribution<float> vel_altitude_;

    Action getRandomAction();

    struct Velocity
    {
        float vx;
        float vy;
        float vz;
    } current_velocity_, setpoint_vel_;

    float current_yaw_;
    float setpoint_yaw_;

    // Subscribed obstacle messages
    ObstacleDistance::SharedPtr obstacle_distance_msg_;

private:
    // ros logger
	rclcpp::Logger logger_;
    // action type
    Action action;
};

