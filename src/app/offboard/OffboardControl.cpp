/**
 * @brief Offboard control with random exploration and collision prevention
 * @file offboard_ctrl.cpp
 * @addtogroup examples
 * @author Xiaoang Zhang <jesse1008611@gmail.com>
 */

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <app/offboard/OffboardControl.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace matrix;

OffboardControl::OffboardControl() 
    : Node("offboard_control"), 
    start_exploring_(false),
    offboard_setpoint_counter_(0),
    counter(0),
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
    collision_constraints_publisher_ = this->create_publisher<CollisionConstraints>(
        "collision_constraints", 10);
    obstacle_distance_fused_publisher_ = this->create_publisher<ObstacleDistance>(
        "obstacle_distance_fused", 10);
    
    
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

            // Update current yaw and velocities for RandomExplore
            random_explore_.setCurrentVelocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
            random_explore_.setCurrentYaw(math::degrees(current_yaw));

            // Update current vehicle odometry for CollisionPrevention
            collision_prevention_.setVehicleOdometry(current_yaw, current_velocity.xy());
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
            // take off to starting point (0.0, 0.0, -10.0)
            _position_setpoint(0) = 0.0;
            _position_setpoint(1) = 0.0;
            _yaw_setpoint = 0.0;
            _altitude_setpoint = -10.0;
        } else if (offboard_setpoint_counter_ == 100) {   
            start_exploring_ = true;
            RCLCPP_INFO(this->get_logger(), "Start exploring...");
        }

        if (start_exploring_) {
            counter += 1;
            if (counter >= counter_limit) {

                // generate a new random action
                random_explore_.performRandomAction();
                random_explore_.getActionType(action);

                // // go straight for testing collision prevention
                // random_explore_.goStraight();
          

                random_explore_.getSetpoint(_velocity_setpoint(0), 
                                            _velocity_setpoint(1), 
                                            _altitude_velocity_setpoint, 
                                            _yaw_setpoint);
                counter = 0;
            } 
            // generate collision-free setpoints
            // auto current_time = this->get_clock()->now();
            // float dt = (current_time.nanoseconds() - last_time_.nanoseconds()) / 1e9; // seconds
            generateSetpoints(current_position, current_velocity.xy());
            // last_time_ = current_time;

            if (action == RandomExplore::Action::ROTATE) {
                if (counter >= counter_limit * 0.25 && counter <= counter_limit * 0.75) {
                    _yaw_speed_setpoint = 0.2181662; // Flip direction
                } else {
                    _yaw_speed_setpoint = -0.2181662;
                }
            }

            publishCollisionConstraints();
            publishObstacleDistanceFused();
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

/**
 * @brief this comes after the random action generation to 
 * get the feasible and collision free setpoints for acce-
 * -leration and velocity.
 * 
 */
void OffboardControl::generateSetpoints(const Vector3f &current_pos, const Vector2f &current_vel_xy)
{   
    // activate collision prevention by setting cp_dist to positive value

    if (collision_prevention_.is_active()) {
        // auto start_time = std::chrono::steady_clock::now(); // Start time
        
        collision_prevention_.modifySetpoint(_velocity_setpoint, _yaw_setpoint);
        
        // auto end_time = std::chrono::steady_clock::now(); // End time
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        
        // RCLCPP_INFO(this->get_logger(), 
        //             "Collision prevention runtime: %ld microseconds", duration);
    }
    
	lockPosition(current_pos, current_vel_xy);
    // Vector2f current_pos_xy  = current_pos.xy();
    // _position_setpoint(0) = current_pos_xy(0) + _velocity_setpoint(0) * dt;
    // _position_setpoint(1) = current_pos_xy(1) + _velocity_setpoint(1) * dt;
}

void OffboardControl::VelocitySmoothing(const float alpha)
{
    _velocity_setpoint(0) = alpha * _velocity_setpoint(0) + (1.0f - alpha) * _velocity_setpoint_prev(0);
    _velocity_setpoint(1) = alpha * _velocity_setpoint(1) + (1.0f - alpha) * _velocity_setpoint_prev(1);

    // update the previous velocity setpoint
    _velocity_setpoint_prev = _velocity_setpoint;
}

/**
 * @brief 
 * 
 * @param pos current position x y z
 * @param vel_sp_feedback current horizontal veclocity vx
 * @param dt 
 */
void OffboardControl::lockPosition(const Vector3f &pos, const Vector2f &vel_sp_feedback)
{
	const bool moving = _velocity_setpoint.norm_squared() > FLT_EPSILON;
	const bool position_locked = Vector2f(_position_setpoint).isAllFinite();

	// lock position
	if (!moving && !position_locked) {
		_position_setpoint = pos.xy();
	}

	// open position loop, horizontal again in velocity control mode
	if (moving && position_locked) {
		_position_setpoint.setNaN();

		// // avoid velocity setpoint jump caused by ignoring remaining position error
		// if (vel_sp_feedback.isAllFinite()) {
		// 	_velocity_setpoint = vel_sp_feedback;
		// }
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
    TrajectorySetpoint msg{};
    // RCLCPP_INFO(this->get_logger(), "Position Setpoint: [%f, %f, %f]", _position_setpoint(0), _position_setpoint(1), _altitude_setpoint);
    // RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", current_position(0), current_position(1), current_position(2));
    // RCLCPP_INFO(this->get_logger(), "Velocity Setpoint: [%f, %f, %f]", _velocity_setpoint(0), _velocity_setpoint(1), _altitude_velocity_setpoint);
    // RCLCPP_INFO(this->get_logger(), "Velocity: [%f, %f, %f]", current_velocity(0), current_velocity(1), current_velocity(2));
    msg.position = {_position_setpoint(0), _position_setpoint(1), _altitude_setpoint};
    msg.velocity = {_velocity_setpoint(0), _velocity_setpoint(1), _altitude_velocity_setpoint};
    msg.yaw = _yaw_setpoint;
    msg.yawspeed = _yaw_speed_setpoint; // yaw speed is constant
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publishCollisionConstraints()
{	
    // get the message from random exploration
    collision_prevention_.getCollisionConstraints(collision_constraints_msg_);
    // publish
    collision_constraints_publisher_->publish(collision_constraints_msg_);
}

void OffboardControl::publishObstacleDistanceFused()
{	
    // get the message from random exploration
    collision_prevention_.getObstacleDistanceFused(obstacle_distance_fused_msg_);
    // publish
    obstacle_distance_fused_publisher_->publish(obstacle_distance_fused_msg_);
}

void OffboardControl::setCurrentPosition(const float x, const float y, const float z)
{   
    // NED eath fixed frame
    current_position(0) = x;
    current_position(1) = y;
    current_position(2) = z;
}

void OffboardControl::setCurrentVelocity(const float vx, const float vy, const float vz)
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
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}