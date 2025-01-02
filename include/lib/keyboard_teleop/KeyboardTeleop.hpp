/**
 * @file keyboard_teleop.hpp
 * @author Xiaoang (jesse1008611@gmail.com)
 * @brief 
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

using namespace geometry_msgs::msg;

class KeyboardTeleop
{
public:
    KeyboardTeleop();
    ~KeyboardTeleop() = default;

    std::vector<int> getKeys();
    void processKeys(const std::vector<int> &key);
    void getKeyMsg(Twist &msg);

private:

    Twist _vel_msg;

    const float _linear_speed = 1.0f; // m/s
    const float _altitude_speed = 0.5f; // m/s
    const float _angular_speed = 0.3f; // rad/s

    // ROS2 logger
    rclcpp::Logger logger_;
};