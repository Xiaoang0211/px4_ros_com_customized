/**
 * @brief Offboard control with random exploration and collision prevention
 * @file offboard_ctrl.cpp
 * @addtogroup examples
 * @author Xiaoang Zhang <jesse1008611@gmail.com>
 */

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <examples/offboard/OffboardControl.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

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
    collision_constraints_publisher_ = this->create_publisher<CollisionConstraints>(
        "collision_constraints", 10);
    obstacle_distance_fused_publisher_ = this->create_publisher<ObstacleDistance>(
        "obstacle_distance_fused", 10);
    
    
    // initialize subscribers
    obstacle_distance_subscriber_ = this->create_subscription<ObstacleDistance>(
        "/fmu/in/obstacle_distance", 10,
        std::bind(&OffboardControl::obstacleDistanceCallback, this, _1));

    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", 10,
        std::bind(&OffboardControl::vehicleOdometryCallback, this, _1));

    auto timerCallback = [this]() -> void {
        if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
		}

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
    };

    timer_ = this->create_wall_timer(100ms, timerCallback);
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

void OffboardControl::vehicleOdometryCallback(const VehicleOdometry::SharedPtr msg)
{   // Getting the current vehicle position and velocity through uorb topic VehicleOdometry
    // Update current position and velocity in RandomExplore
    random_explore_.setCurrentPosition(msg->position[0], msg->position[1], msg->position[2]);
    random_explore_.setCurrentVelocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    // const auto &q = msg->q;
    const std::array<float, 4> &q = msg->q;
    float siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    float current_yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI; // Convert to degrees
    // Update current yaw in RandomExplore
    random_explore_.setCurrentYaw(current_yaw);
    random_explore_.setCurrentQuat(q);
}

void OffboardControl::publishOffboardControlMode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publishTrajectorySetpoint()
{	
    // get the desired setpoint and yaw angle from RandomExplore
    random_explore_.getSetpoint(setpoint_vx, setpoint_vy, setpoint_vz, setpoint_yaw); // set point of xyz direction velocities
    setpoint_yaw = setpoint_yaw * M_PI / 180.0; // Convert to radians

    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    // msg.velocity = {setpoint_vx, setpoint_vy, setpoint_vz};
    // msg.yaw = setpoint_yaw;
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publishCollisionConstraints()
{	
    // get the message from random exploration
    random_explore_.getCollisionConstraints(collision_constraints_msg_);
    // publish
    collision_constraints_publisher_->publish(collision_constraints_msg_);
}

void OffboardControl::publishObstacleDistanceFused()
{	
    // get the message from random exploration
    random_explore_.getObstacleDistanceFused(obstacle_distance_fused_msg_);
    // publish
    obstacle_distance_fused_publisher_->publish(obstacle_distance_fused_msg_);
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