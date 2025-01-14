/**
 * @file keyboard_control.cpp
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
#include <app/offboard/keyboard_control.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace geometry_msgs::msg;
using namespace matrix;

KeyboardControl::KeyboardControl() 
    : Node("keyboard_control"), 
    _start_exploring(false),
    _offboard_setpoint_counter(0),
    collision_prevention_({-1.0f, 0.4f, 30.0f, false, 0.95f, 0.5f, 1.0f, 1.8f, 11.98f})
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
            _current_yaw = Eulerf(Quatf(q.data())).psi(); // yaw angle in radians

            // Update current vehicle odometry for CollisionPrevention
            collision_prevention_.setVehicleOdometry(_current_yaw, _current_velocity.xy());
        });

    auto timerCallback = [this]() -> void {
        if (_offboard_setpoint_counter == 10) {
            // Change to Offboard mode after 1 second 
            this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this->arm();
		}

        if (_offboard_setpoint_counter >= 11 && _offboard_setpoint_counter < 100) {
            // take off to the starting point (0.0, 0.0, -10.0)
            _position_setpoint(0) = 0.0;
            _position_setpoint(1) = 0.0;
            _yaw_setpoint = -M_PI;
            _altitude_setpoint = -10.0;
        } else if (_offboard_setpoint_counter == 100) {   
            _start_exploring = true;
            RCLCPP_INFO(this->get_logger(), "Start exploring with NoMaD...");
        }

        // start velocity control mode (both linear and angular) for exploring
        if (_start_exploring) {
            _yaw_setpoint = std::numeric_limits<float>::quiet_NaN();
            // get the keyboard command for velocity setpoint
            getKeyCommand();

            // generate collision-free setpoint or lock position if the setpoint speed is 0.0
            generateFeasibleSetpoints(_current_position, _current_velocity.xy());
        }

        // offboard control mode needs to be paired with trajectory setpoint
        publishOffboardControlMode();
        publishTrajectorySetpoint();

        // Increase the setpoint counter before starting with exploring
        if (_offboard_setpoint_counter < 101) {
            _offboard_setpoint_counter++;
        }
    };
    timer_ = this->create_wall_timer(100ms, timerCallback);
}

void KeyboardControl::getKeyCommand()
{
    auto keys = keyboard_teleop_.getKeys();
    if (keys.empty()) {
        _velocity_setpoint(0) = 0.0f;
        _velocity_setpoint(1) = 0.0f;
        _altitude_velocity_setpoint = 0.0f;
        _yaw_speed_setpoint = 0.0f;
        return;
    }

    // if some keys are pressed, then process key commands
    keyboard_teleop_.processKeys(keys);
    keyboard_teleop_.getKeyMsg(_velocity_msg);

    _velocity_setpoint(0) = _velocity_msg.linear.x * cos(_current_yaw);
    _velocity_setpoint(1) = _velocity_msg.linear.x * sin(_current_yaw);
    _altitude_velocity_setpoint = _velocity_msg.linear.z;
    _yaw_speed_setpoint = _velocity_msg.angular.z;
}

void KeyboardControl::generateFeasibleSetpoints(const Vector3f &current_pos, const Vector2f &current_vel_xy)
{   
    Vector2f original_vel_setpoint = _velocity_setpoint;

    // activate collision prevention by setting cp_dist to positive value
    if (collision_prevention_.is_active()) {
        collision_prevention_.modifySetpoint(_velocity_setpoint, _yaw_setpoint);
    }

    lockPosition(current_pos, current_vel_xy);

    if (Vector2f(original_vel_setpoint - _velocity_setpoint).norm() > FLT_EPSILON) {
        _collision = true;
        RCLCPP_INFO(this->get_logger(), "collision!!!!!!!!!!!!!");
    } else {
        _collision = false;
    }
}

/**
 * @brief 
 * 
 * @param pos current position x y z
 * @param vel_sp_feedback current horizontal veclocity vx
 * @param dt 
 */
void KeyboardControl::lockPosition(const Vector3f &pos, const Vector2f &vel_sp_feedback)
{   
    // determine if honrizontally moving
	const bool horizontal_moving = _velocity_setpoint.norm_squared() > FLT_EPSILON;
	const bool position_locked = Vector2f(_position_setpoint).isAllFinite();

    // determin if vertically moving
    const bool vertical_moving = fabsf(_altitude_velocity_setpoint) > FLT_EPSILON;
    const bool altitude_locked = std::isfinite(_altitude_setpoint);

	// lock position
	if (!horizontal_moving && !position_locked) {
		_position_setpoint = pos.xy();
	}

	// open position loop, velocity control mode
	if (horizontal_moving && position_locked) {
		_position_setpoint.setNaN();
	}

    if (!vertical_moving && !altitude_locked) {
        _altitude_setpoint = pos(2);
    }

    if (vertical_moving && altitude_locked) {
        _altitude_setpoint = std::numeric_limits<float>::quiet_NaN();
    }
}

void KeyboardControl::arm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Sent arming command");
}

void KeyboardControl::disarm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Sent disarming command");
}

void KeyboardControl::publishVehicleCommand(uint16_t command, float param1, float param2)
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

void KeyboardControl::publishOffboardControlMode()
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

void KeyboardControl::publishTrajectorySetpoint()
{	
    TrajectorySetpoint msg{};
    // RCLCPP_INFO(this->get_logger(), "Position Setpoint: [%f, %f, %f]", _position_setpoint(0), _position_setpoint(1), _altitude_setpoint);
    // RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", _current_position(0), _current_position(1), _current_position(2));
    // RCLCPP_INFO(this->get_logger(), "Velocity Setpoint: [%f, %f, %f]", _velocity_setpoint(0), _velocity_setpoint(1), _altitude_velocity_setpoint);
    // RCLCPP_INFO(this->get_logger(), "Velocity: [%f, %f, %f]", _current_velocity(0), _current_velocity(1), _current_velocity(2));
    msg.position = {_position_setpoint(0), _position_setpoint(1), _altitude_setpoint};
    msg.velocity = {_velocity_setpoint(0), _velocity_setpoint(1), _altitude_velocity_setpoint};
    msg.yaw = _yaw_setpoint;
    msg.yawspeed = _yaw_speed_setpoint; // yaw speed is constant
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void KeyboardControl::setCurrentPosition(const float x, const float y, const float z)
{   
    // NED eath fixed frame
    _current_position(0) = x;
    _current_position(1) = y;
    _current_position(2) = z;
}

void KeyboardControl::setCurrentVelocity(const float vx, const float vy, const float vz)
{   
    // NED eath fixed frame
    _current_velocity(0) = vx;
    _current_velocity(1) = vy;
    _current_velocity(2) = vz;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControl>());

    rclcpp::shutdown();
    return 0;
}