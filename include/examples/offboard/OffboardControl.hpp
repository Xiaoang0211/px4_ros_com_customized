// offboard_control.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/collision_constraints.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <lib/random_explore/RandomExplore.hpp>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();
    ~OffboardControl() override = default;

private:
    // Timer to periodically execute random movements
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<CollisionConstraints>::SharedPtr collision_constraints_publisher_;
    rclcpp::Publisher<ObstacleDistance>::SharedPtr obstacle_distance_fused_publisher_;

    // Subscribers
    rclcpp::Subscription<ObstacleDistance>::SharedPtr obstacle_distance_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscriber_;

    // Callback functions
    void obstacleDistanceCallback(const ObstacleDistance::SharedPtr msg);
    void vehicleAttitudeCallback(const VehicleAttitude::SharedPtr msg);
    void localPositionCallback(const VehicleLocalPosition::SharedPtr msg);

    // collision prevention publisher messages
    CollisionConstraints collision_constraints_msg_;
    ObstacleDistance obstacle_distance_fused_msg_;

    // setpoints obtained by RandomExplore with collision prevention
    float setpoint_vx, setpoint_vy, setpoint_vz;
    float setpoint_yaw;

    // Random exploration
    RandomExplore random_explore_;

    // Offboard control state
    uint64_t offboard_setpoint_counter_;

    // Timer callback
    void timerCallback();

    // Publish functions
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publishCollisionConstraints();
    void publishObstacleDistanceFused();

    // Arm/Disarm functions
    void arm();
    void disarm();
};
