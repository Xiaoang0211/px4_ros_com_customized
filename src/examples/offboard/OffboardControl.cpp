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

OffboardControl::OffboardControl() 
    : Node("offboard_control"), 
    start_exploring_(false),
    offboard_setpoint_counter_(0),
    counter(0),
    collision_prevention_({1.0f, 0.1f, 45.0f, true, 1.0f, 1.0f, 1.0f})
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
            // Update current velocity in RandomExplore
            random_explore_.setCurrentVelocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

            // Update current position and velocity in OffboardControl
            this->setCurrentXYPosition(msg->position[0], msg->position[1]);
            this->setCurrentXYVelocity(msg->velocity[0], msg->velocity[1]);

            // const &q = msg->q;
            const std::array<float, 4> &q = msg->q;
            float siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
            float cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
            float current_yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI; // Convert to degrees
            // Update current yaw in RandomExplore
            random_explore_.setCurrentYaw(current_yaw);

            // Update current vehicle attitude in CollisionPrevention
            collision_prevention_.setCurrentAttitude(q);

        });

    // initialize to 0 ensure they do not have any garbage values
    current_xy_position = matrix::Vector2f({0.0, 0.0});
    current_xy_velocity = matrix::Vector2f({0.0, 0.0});

	last_time_ = this->get_clock()->now();

    auto timerCallback = [this]() -> void {
        if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 1 second 
				this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				// Arm the vehicle
				this->arm();
		}

        if (offboard_setpoint_counter_ >= 11 && offboard_setpoint_counter_ < 500) {
            // take off to starting point (0.0, 0.0, -10.0)
            setpoint_x = 0.0;
            setpoint_y = 0.0;
            setpoint_z = -10.0;
        } else if (offboard_setpoint_counter_ == 500)
        {   
            start_exploring_ = true;
            RCLCPP_INFO(this->get_logger(), "Start exploring...");
        }

        if (start_exploring_) {
            counter += 1;
            if (counter == 50) {
                // generate a new random action
                random_explore_.performRandomAction();
                random_explore_.getSetpoint(setpoint_vx, setpoint_vy, setpoint_vz, setpoint_yaw); // set point of xyz direction velocities
                // Reset the counter after executing the action for 5 seconds
                counter = 0;
            } 
            auto current_time = this->get_clock()->now();
            float dt = (current_time.nanoseconds() - last_time_.nanoseconds()) / 1e9; // seconds
            // update the position setpoint
            setpoint_x = current_xy_position(0) + setpoint_vx * dt;
            setpoint_y = current_xy_position(1) + setpoint_vy * dt;
            
            // TODO: add collision prevention here to constrain setpoint
            // collision_prevention_.modifySetpoint(vel_setpoint, max_speed, current_xy_position, current_xy_velocity);

            // setpoint_z = setpoint_vz * dt;
            last_time_ = current_time;

            publishCollisionConstraints();
            publishObstacleDistanceFused();
        }

        // offboard control mode needs to be paired with trajectory setpoint
        publishOffboardControlMode();
        publishTrajectorySetpoint();

        // Increase the setpoint counter before starting with exploring
        if (offboard_setpoint_counter_ < 501) {
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
    // msg.position = {0.0, 0.0, -5.0};
    msg.position = {setpoint_x, setpoint_y, setpoint_z};
    msg.velocity = {setpoint_vx, setpoint_vy, setpoint_vz};
    msg.yaw = setpoint_yaw;
    msg.yawspeed = 0.314;
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

void OffboardControl::setCurrentXYPosition(const float x, const float y)
{   
    // NED eath fixed frame
    current_xy_position(0) = x;
    current_xy_position(1) = y;
}

void OffboardControl::setCurrentXYVelocity(const float vx, const float vy)
{   
    // NED eath fixed frame
    current_xy_velocity(0) = vx;
    current_xy_velocity(1) = vy;
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