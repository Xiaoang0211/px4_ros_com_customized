/**
 * @brief Offboard control with random exploration and collision prevention
 * @file offboard_ctrl.cpp
 * @addtogroup examples
 * @author Xiaoang Zhang <jesse1008611@gmail.com>
 */

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <lib/examples/offboard/OffboardControl.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

OffboardControl::OffboardControl() 
    : Node("offboard_control"), 
    offboard_setpoint_counter_(0)
{

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    collision_constraints_publisher = this->create_publisher<CollisionConstraints>(
        "collision_constraints", 10);
    obstacle_distance_fused_publisher = this->create_publisher<ObstacleDistance>(
        "obstacle_distance_fused", 10);
    
    
    // initialize subscribers
    obstacle_distance_subscriber_ = this->create_subscription<ObstacleDistance>(
        "/fmu/in/obstacle_distance", 10,
        std::bind(&OffboardControl::obstacleDistanceCallback, this, std::placeholders::_1));

    vehicle_attitude_subscriber_ = this->create_subscription<ObstacleDistance>(
        "/fmu/out/vehicle_attitude", 10,
        std::bind(&OffboardControl::vehicleAttitudeCallback, this, std::placeholders::_1));

    local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", 10,
        std::bind(&OffboardControl::localPositionCallback, this, std::placeholders::_1));

    arm();

    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timerCallback, this));
}

void OffboardControl::timerCallback()
{   
    // 1 second of warm-up
    if (offboard_setpoint_counter_ == 10) {
        // Change to Offboard mode after 10 setpoints
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        RCLCPP_INFO(this->get_logger(), "Sent Offboard mode command");
    }

    // pass the subscribed messages to RandomExplore for collision prevention
    random_explore_.setObstacleDistanceMsg(obstacle_distance_msg_);
    random_explore_.setVehicleAttitudeMsg(vehicle_attitude_msg_);

    // perform a random action with collision prevention
    random_explore_.performRandomAction();

    publishCollisionConstraints();
    publishObstacleDistanceFused();

	// offboard control mode needs to be paired with trajectory setpoint
    publishOffboardControlMode();
    publishTrajectorySetpoint();

    // Increase the setpoint counter
    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}

void OffboardControl::arm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Sent arming command");
}

void OffboardControl::disarm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Sent disarming command");
}

void OffboardControl::publishOffboardControlMode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::obstacleDistanceCallback(const ObstacleDistance::SharedPtr msg)
{   
    random_explore_.setObstacleDistance(*msg);
}

void OffboardControl::vehicleAttitudeCallback(const VehicleAttitude::SharedPtr msg)
{
    // Extract yaw from the attitude quaternion
    const auto &q = msg->q; // Assuming q is [w, x, y, z]
    float siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    float current_yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI; // Convert to degrees

    // Update current yaw in RandomExplore
    random_explore_.setCurrentYaw(current_yaw);
    random_explore_.setVehicleAttitude(*msg);
}

void OffboardControl::localPositionCallback(const VehicleLocalPosition::SharedPtr msg)
{
    // Update current position and velocity in RandomExplore
    random_explore_.setCurrentPosition(msg->x, msg->y, msg->z);
    random_explore_.setCurrentVelocity(msg->vx, msg->vy, msg->vz);
}

/**
 * @brief 
 * 
 * @param xy_vel_setpoint set velocities in xy horizontal plain
 * @param altitude_vel set velocity in z direction
 */
void OffboardControl::publishTrajectorySetpoint()
{	
    // get the desired setpoint and yaw angle from RandomExplore
    random_explore_.getSetpoint(setpoint_vx, setpoint_vy, setpoint_vz); // set point of xyz direction velocities
    random_explore_.getSetpointYaw(setpoint_yaw);
    setpoint_yaw = setpoint_yaw * M_PI / 180.0; // Convert to radians

    TrajectorySetpoint msg{};
    msg.velocity = {vx_setpoint, vy_setpoint, altitude_vel}
    msg.yaw = yaw_setpoint;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publishCollisionConstraints()
{	
    // get the message from random exploration
    random_explore_.getCollisionConstraints(collision_constraints_msg_);
    // publish
    collision_constraints_publisher->publish(collision_constraints_msg_);
}

void OffboardControl::publishObstacleDistanceFused()
{	
    // get the message from random exploration
    random_explore_.getObstacleDistanceFused(obstacle_distance_fused_msg_);
    // publish
    obstacle_distance_fused_publisher->publish(obstacle_distance_fused_msg_);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}