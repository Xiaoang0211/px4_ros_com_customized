/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CollisionPrevention.hpp
 * @author Tanja Baumann <tanja@auterion.com>
 * @author Claudio Chies <claudio@chies.com>
 * @author Xiaoang Zhang <jesse1008611@gmail.com>
 *
 * CollisionPrevention controller.
 *
 */

#pragma once

#include <float.h>
#include <px4/px4_custom_mode.h>
#include <chrono>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/math.hpp>
#include <limits>
#include <mutex>
#include <px4_msgs/msg/collision_constraints.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

using hrt_abstime = uint64_t; // in nano seconds
using hrt_duration = uint64_t;
using namespace px4_msgs::msg;
using namespace std::chrono_literals;
using namespace matrix;

namespace
{
static constexpr int BIN_SIZE = 5; //cannot be lower than 5 degrees, should divide 360 evenly
static constexpr int BIN_COUNT = 360 / BIN_SIZE;
static constexpr uint64_t RANGE_STREAM_TIMEOUT_US = std::chrono::duration_cast<std::chrono::nanoseconds>(500ms).count();
} // namespace


struct CollisionPreventionParameters
{
	float cp_dist;
	float cp_delay;
	float cp_guide_ang;
	bool cp_go_no_data;
	float mpc_xy_p;
	float mpc_jerk_max;
	float mpc_acc_hor;
	float mpc_vel_p_acc; /**< p gain from velocity controller*/
	float mpc_xy_vel_max; /**< maximum velocity in offboard control mode*/
};

class CollisionPrevention
{
public:
	CollisionPrevention(const CollisionPreventionParameters& params);
	~CollisionPrevention() = default;

	/**
	 * Returns true if Collision Prevention is running
	 */
	bool is_active();

	/**
	 * @brief Setters for input data
	 * 
	 */
	void setObstacleDistance(const ObstacleDistance& msg);
	void setVehicleOdometry(const float yaw, const Vector2f current_vel);

	/**
	 * @brief Getters for output data
	 * 
	 */
	void getCollisionConstraints(CollisionConstraints& msg);
	void getObstacleDistanceFused(ObstacleDistance& msg);

	/**
	 * Computes collision free setpoints
	 * @param original_setpoint, setpoint before collision prevention intervention
	 * @param max_speed, maximum xy speed
	 * @param curr_pos, current vehicle position
	 * @param curr_vel, current vehicle velocity
	 */
	void modifySetpoint(Vector2f& setpoint_vel, float& setpoint_yaw);


protected:

	ObstacleDistance _obstacle_map_body_frame;
	bool _data_fov[BIN_COUNT] {};
	uint64_t _data_timestamps[BIN_COUNT] {};
	uint16_t _data_maxranges[BIN_COUNT] {}; /**< in cm */

	//Timing functions. Necessary to mock time in the tests
	virtual hrt_abstime getTime();
	virtual hrt_duration getElapsedTime(const hrt_abstime& start_time);

	/**
	 * Aggregates the sensor data into a internal obstacle map in body frame
	 */
	void _updateObstacleMap();

	void _addObstacleSensorData(const ObstacleDistance &obstacle, const float vehicle_yaw);

	bool _enterData(int map_index, float sensor_range, float sensor_reading);

	void _updateObstacleData();

    void _adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index, float &setpoint_yaw);

	bool _checkSetpointDirectionFeasibility();

	bool _concaveDetection(const Vector2f velocity_dir);

	int _getDirectionIndexBodyFrame(const Vector2f &direction);

	Vector2f _getPointVelocityFrame(int bin_index, const Vector2f &velocity_dir, const Vector2f &right_dir);

	float _getObstacleDistance(const Vector2f &direction);

	float _getScale(const float &reference_distance);

private:
	std::shared_mutex data_mutex;
	// the messages streamed through subscription
    ObstacleDistance current_obstacle_distance;
    float current_yaw;
	Vector2f current_vel; /**<current xy position> */

	// safe copies of the streamed input messages that we use in the calculation 
	ObstacleDistance _obstacle_distance;

	// current yaw and velocity in ned frame
	float _vehicle_yaw{0.f}; // rad
	Vector2f _vehicle_vel{0.f, 0.f};

	bool _obstacle_data_present{false};		/**< states if obstacle data is present */
	bool _was_active{false};				/**< states if the collision prevention interferes with the user input */
	bool _is_concave{false};
	int _setpoint_index{};
	Vector2f _setpoint_dir{};		/**< direction of the setpoint*/
	
	float _closest_dist{};
	Vector2f _closest_dist_dir{NAN, NAN};

	float _min_dist_to_keep{};

	// flags for checking if new message is received
    bool _obstacle_distance_received{false};
    bool _vehicle_odometry_received{false};

	// Parameters
	CollisionPreventionParameters _params;

	// ROS logger
	rclcpp::Logger logger_;

	// Collision prevention constraints should be published by the ros node OffboardControl
	CollisionConstraints constraints;

	hrt_abstime _last_timeout_warning{0};
	hrt_abstime _time_activated{0};

	void _calculateConstrainedSetpoint(Vector2f &setpoint_vel, const Vector2f &_vehicle_vel, float &setpoint_yaw, const float &_vehicle_yaw);


	static float _wrap_360(const float f);
	static int _wrap_bin(int i);
};
