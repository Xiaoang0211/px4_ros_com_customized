/**
 * @file NoMaD_control.cpp
 * @author Xiaoang Zhang (jesse1008611@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <app/offboard/NoMaD_control.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace matrix;

NoMaDControl::NoMaDControl() 
    : Node("NoMaD_control"), 
    start_exploring_(false),
    offboard_setpoint_counter_(0),
    collision_prevention_({0.7f, 0.4f, 30.0f, false, 0.95f, 0.5f, 1.0f, 1.8f, 11.98f})
{   
    qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    
    
    
    // initialize subscribers
    obstacle_distance_subscriber_ = this->create_subscription<ObstacleDistance>(
        "/fmu/in/obstacle_distance", qos,
        [this](const ObstacleDistance::SharedPtr msg) {
            collision_prevention_.setObstacleDistance(*msg);
        });

    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        [this](const VehicleOdometry::SharedPtr msg) {
            // Getting the current vehicle position and velocity through uorb topic VehicleOdometry
            // Update current position and velocity in OffboardControl
            this->setCurrentPosition(msg->position[0], msg->position[1], msg->position[2]);
            this->setCurrentVelocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

            // const &q = msg->q;
            const std::array<float, 4> &q = msg->q;
            current_yaw = Eulerf(Quatf(q.data())).psi(); // yaw angle in radians

            // Update current vehicle odometry for CollisionPrevention
            collision_prevention_.setVehicleOdometry(current_yaw, current_velocity.xy());
        });

    velocity_setpoint_subscriber_ = this->create_subscription<Twist>(
        "/cmd_vel_mux/input/navi", qos, 
        [this](const Twist::SharedPtr msg) {
            // receive velocity control commads from NoMaD
            // calculate the velocity in earth-fixed NED frame
            _nomad_velocity(0) = msg->linear.x * cos(current_yaw);
            _nomad_velocity(1) = msg->linear.x * sin(current_yaw);
            _nomad_yaw_speed = msg->angular.z;
            _last_time_nomad_received = this->get_clock()->now();
        });

	// last_time_ = this->get_clock()->now();

    auto timerCallback = [this]() -> void {
        if (offboard_setpoint_counter_ == 10) {
            // Change to Offboard mode after 1 second 
            this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this->arm();
		}

        if (offboard_setpoint_counter_ >= 11 && offboard_setpoint_counter_ < 100) {
            // take off to the starting point (0.0, 0.0, -10.0)
            _position_setpoint(0) = 0.0;
            _position_setpoint(1) = 0.0;
            _yaw_setpoint = 0.0;
            _altitude_setpoint = -10.0;
        } else if (offboard_setpoint_counter_ == 100) {   
            start_exploring_ = true;
            RCLCPP_INFO(this->get_logger(), "Start exploring with NoMaD...");
        }

        // start velocity control mode for exploring
        if (start_exploring_) {
            // overwrite the velocity setpoint with the current NoMaD setpoint
            getNoMaDSetpoint();

            // set the vertical velocity to 0.0
            _altitude_velocity_setpoint = 0.0;

            // generate collision-free setpoints
            // or lock position if the setpoint speed is 0.0
            generateFeasibleSetpoints(current_position, current_velocity.xy());
        }

        // offboard control mode needs to be paired with trajectory setpoint
        publishOffboardControlMode();
        publishTrajectorySetpoint();

        // Increase the setpoint counter before starting with exploring
        if (offboard_setpoint_counter_ < 101) {
            offboard_setpoint_counter_++;
        }
    };
    timer_ = this->create_wall_timer(100ms, timerCallback);
}

void NoMaDControl::getNoMaDSetpoint()
{   
    // lock position when nomad message is stale
    rclcpp::Duration nomad_data_age(this->get_clock()->now() - _last_time_nomad_received);
    if (nomad_data_age > NOMAD_TIMEOUT) {
        _velocity_setpoint = Vector2f({0.0, 0.0});
        _yaw_speed_setpoint = 0.0;
    }
    _velocity_setpoint = _nomad_velocity;
    _yaw_speed_setpoint = _nomad_yaw_speed;
}

void NoMaDControl::generateFeasibleSetpoints(const Vector3f &current_pos, const Vector2f &current_vel_xy)
{   
    // activate collision prevention by setting cp_dist to positive value
    if (collision_prevention_.is_active()) {
        collision_prevention_.modifySetpoint(_velocity_setpoint, _yaw_setpoint);
    }

	lockPosition(current_pos, current_vel_xy);
}

/**
 * @brief 
 * 
 * @param pos current position x y z
 * @param vel_sp_feedback current horizontal veclocity vx
 * @param dt 
 */
void NoMaDControl::lockPosition(const Vector3f &pos, const Vector2f &vel_sp_feedback)
{
	const bool moving = _velocity_setpoint.norm_squared() > FLT_EPSILON;
	const bool position_locked = Vector2f(_position_setpoint).isAllFinite();

	// lock position
	if (!moving && !position_locked) {
		_position_setpoint = pos.xy();
	}

	// open position loop, velocity control mode
	if (moving && position_locked) {
		_position_setpoint.setNaN();

		// // avoid velocity setpoint jump caused by ignoring remaining position error
		// if (vel_sp_feedback.isAllFinite()) {
		// 	_velocity_setpoint = vel_sp_feedback;
		// }
	}
}

void NoMaDControl::arm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Sent arming command");
}

void NoMaDControl::disarm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Sent disarming command");
}

void NoMaDControl::publishVehicleCommand(uint16_t command, float param1, float param2)
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

void NoMaDControl::publishOffboardControlMode()
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

void NoMaDControl::publishTrajectorySetpoint()
{	
    TrajectorySetpoint msg{};
    // RCLCPP_INFO(this->get_logger(), "Position Setpoint: [%f, %f, %f]", _position_setpoint(0), _position_setpoint(1), _altitude_setpoint);
    // RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", current_position(0), current_position(1), current_position(2));
    // RCLCPP_INFO(this->get_logger(), "Velocity Setpoint: [%f, %f, %f]", _velocity_setpoint(0), _velocity_setpoint(1), _altitude_velocity_setpoint);
    // RCLCPP_INFO(this->get_logger(), "Velocity: [%f, %f, %f]", current_velocity(0), current_velocity(1), current_velocity(2));
    msg.position = {_position_setpoint(0), _position_setpoint(1), _altitude_setpoint};
    msg.velocity = {_velocity_setpoint(0), _velocity_setpoint(1), _altitude_velocity_setpoint};
    msg.yaw = std::numeric_limits<float>::quiet_NaN();
    msg.yawspeed = _yaw_speed_setpoint; // yaw speed is constant
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void NoMaDControl::setCurrentPosition(const float x, const float y, const float z)
{   
    // NED eath fixed frame
    current_position(0) = x;
    current_position(1) = y;
    current_position(2) = z;
}

void NoMaDControl::setCurrentVelocity(const float vx, const float vy, const float vz)
{   
    // NED eath fixed frame
    current_velocity(0) = vx;
    current_velocity(1) = vy;
    current_velocity(2) = vz;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoMaDControl>());

    rclcpp::shutdown();
    return 0;
}