/**
 * @file keyboard_control.hpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief 
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <lib/keyboard_teleop/KeyboardTeleop.hpp>
#include <lib/collision_prevention/CollisionPrevention.hpp>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using namespace matrix;

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl();
    ~KeyboardControl() override = default;

    void setCurrentPosition(const float x, const float y, const float z);
    void setCurrentVelocity(const float vx, const float vy, const float vz);
    
    rmw_qos_profile_t qos_profile;


private:
    // Timer to periodically execute subscribed velocity setpoints from NoMaD
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
   
    // Subscribers
    rclcpp::Subscription<ObstacleDistance>::SharedPtr obstacle_distance_subscriber_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

    // keyboard teleoperation
    KeyboardTeleop keyboard_teleop_;
    // velocity commands in body frame
    Twist _velocity_msg;

    // horizontal position, velocity, acceleration setpoint
    Vector2f _position_setpoint;
    Vector2f _velocity_setpoint;
    Vector2f _velocity_setpoint_prev;
    // altitude setting
    float _altitude_setpoint;
    float _altitude_velocity_setpoint;

    // yaw (in earth-fixed NED frame) setpoint
    float _yaw_setpoint;
    float _yaw_speed_setpoint;

    bool _start_exploring;
    bool _collision;
    int _collision_countdown{0};
    // rclcpp::Time last_time_;                 // Track time for integration

    // Collision Prevention
    CollisionPrevention collision_prevention_;

    // current drone states
	Vector3f _current_position{0.0, 0.0, 0.0};
    Vector3f _current_velocity{0.0, 0.0, 0.0};
    float _current_yaw;

    // Offboard control state
    uint64_t _offboard_setpoint_counter;

    void getKeyCommand(); // convert keyboard velocity commands to earth-fixed NED frame
    void generateFeasibleSetpoints(const Vector3f &current_pos, const Vector2f &current_vel_xy);
    void lockPosition(const Vector3f &pos, const Vector2f &vel_sp_feedback);

    // Timer callback
    void timerCallback();

    // Publish functions
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    // Arm/Disarm functions
    void arm();
    void disarm();
};