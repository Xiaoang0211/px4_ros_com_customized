/**
 * @file keyboard_teleop.cpp
 * @author Xiaoang (jesse1008611@gmail.com)
 * @brief 
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <lib/keyboard_teleop/KeyboardTeleop.hpp>

using namespace geometry_msgs::msg;

KeyboardTeleop::KeyboardTeleop() :
    logger_(rclcpp::get_logger("KeyboardTeleop"))
{}

std::vector<int> KeyboardTeleop::getKeys() 
{   
    std::vector<int> pressed_keys;
    struct termios oldt, newt;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // raw mode
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    int ch;
    while ((ch = getchar()) != EOF) {
        pressed_keys.push_back(ch);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return pressed_keys;
}


void KeyboardTeleop::processKeys(const std::vector<int> &keys)
{   
    bool w_pressed = false;
    bool s_pressed = false;
    bool a_pressed = false;
    bool d_pressed = false;
    bool q_pressed = false;
    bool e_pressed = false;
    
    for (int key: keys) {
        switch (key) {
            case 'w':
            case 'W':
                w_pressed = true;
                break;
            case 's':
            case 'S':
                s_pressed = true;
                break;
            case 'a':
            case 'A':
                a_pressed = true;
                break;
            case 'd':
            case 'D':
                d_pressed = true;
                break;
            case 'q':
            case 'Q':
                q_pressed = true;
                break;
            case 'e':
            case 'E':
                e_pressed = true;
            default:
                // ignore all other keys
                break; 
        }
    }
    
    if (w_pressed && !s_pressed) {
        _vel_msg.linear.x = _linear_speed;
    } else if (!w_pressed && s_pressed)
    {
        _vel_msg.linear.x = -_linear_speed;
    } else {
        _vel_msg.linear.x = 0.0f;
    }

    if (a_pressed && !d_pressed) {
        _vel_msg.angular.z = _angular_speed;
    } else if (!a_pressed && d_pressed)
    {
        _vel_msg.angular.z = _angular_speed;
    } else {
        _vel_msg.angular.z = 0.0f;
    }

    if (q_pressed && !e_pressed) {
        _vel_msg.linear.z = -_altitude_speed;
    } else if (!q_pressed && e_pressed)
    {
        _vel_msg.linear.z = _altitude_speed;
    } else {
        _vel_msg.linear.z = 0.0f;
    }
}

void KeyboardTeleop::getKeyMsg(Twist &msg)
{
    msg = _vel_msg;
}