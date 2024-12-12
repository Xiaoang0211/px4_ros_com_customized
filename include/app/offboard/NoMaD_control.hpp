/**
 * @file NoMaD_control.hpp
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
#include <lib/collision_prevention/CollisionPrevention.hpp>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using namespace matrix;

class NoMaDControl : public rclcpp::Node
{
public:
    NoMaDControl();
    ~NoMaDControl() override = default;

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
    // rclcpp::Publisher<CollisionConstraints>::SharedPtr collision_constraints_publisher_;
    // rclcpp::Publisher<ObstacleDistance>::SharedPtr obstacle_distance_fused_publisher_;

    // Subscribers
    rclcpp::Subscription<ObstacleDistance>::SharedPtr obstacle_distance_subscriber_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_setpoint_subscriber_; // linear and angular velocity setpoints from NoMaD 


    // // collision prevention publisher messages
    // CollisionConstraints collision_constraints_msg_;
    // ObstacleDistance obstacle_distance_fused_msg_;

    // NoMaD velocity commands in ned frame
    Vector2f _nomad_velocity;
    float _nomad_yaw_speed;
    rclcpp::Time _last_time_nomad_received;
    const rclcpp::Duration NOMAD_TIMEOUT = rclcpp::Duration::from_nanoseconds(1000000000);

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

    void getNoMaDSetpoint();

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
