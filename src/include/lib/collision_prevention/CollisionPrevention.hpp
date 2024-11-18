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
#include <px4_msgs/msg/collision_constraints.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using hrt_abstime = uint64_t; // in nano seconds
using hrt_duration = uint64_t;
using namespace px4_msgs::msg;
using namespace std::chrono_literals;

struct CollisionPreventionParameters
{
	float cp_dist;
	float cp_delay;
	float cp_guide_ang;
	bool cp_go_nodata;
	float mpc_xy_p;
	float mpc_jerk_max;
	float mpc_acc_hor;
}

class CollisionPrevention
{
public:
	CollisionPrevention(const CollisionPreventionParameters& params);
	~CollisionPrevention() override = default;

	/**
	 * Returns true if Collision Prevention is running
	 */
	bool is_active();

	/**
	 * @brief Setters for input data
	 * 
	 */
	void setObstacleDistance(const ObstacleDistance& msg);
	void setVehicleAttitude(const VehicleAttitude& msg);

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
	void modifySetpoint(matrix::Vector2f &original_setpoint, const float max_speed,
			    const matrix::Vector2f &curr_pos, const matrix::Vector2f &curr_vel);

protected:

	ObstacleDistance _obstacle_map_body_frame;
	bool _data_fov[sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0])];
	uint64_t _data_timestamps[sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0])];
	uint16_t _data_maxranges[sizeof(_obstacle_map_body_frame.distances) / sizeof(
										    _obstacle_map_body_frame.distances[0])]; /**< in cm */

	// the messages streamed through subscription
    ObstacleDistance _latest_obstacle_distance;
    VehicleAttitude _latest_vehicle_attitude;

	// flags for checking if new message is received
    bool _new_obstacle_distance_received;
    bool _new_vehicle_attitude_received;

	/**
	 * Updates obstacle distance message with measurement from offboard
	 * @param obstacle, obstacle_distance message to be updated
	 */
	void _addObstacleSensorData(const ObstacleDistance &obstacle, const matrix::Quatf &vehicle_attitude);

	/**
	 * Computes an adaption to the setpoint direction to guide towards free space
	 * @param setpoint_dir, setpoint direction before collision prevention intervention
	 * @param setpoint_index, index of the setpoint in the internal obstacle map
	 * @param vehicle_yaw_angle_rad, vehicle orientation
	 */
	void _adaptSetpointDirection(matrix::Vector2f &setpoint_dir, int &setpoint_index, float vehicle_yaw_angle_rad);

	/**
	 * Determines whether a new sensor measurement is used
	 * @param map_index, index of the bin in the internal map the measurement belongs in
	 * @param sensor_range, max range of the sensor in meters
	 * @param sensor_reading, distance measurement in meters
	 */
	bool _enterData(int map_index, float sensor_range, float sensor_reading);

	/**
	 * Computes collision free setpoints
	 * @param setpoint, setpoint before collision prevention intervention
	 * @param curr_pos, current vehicle position
	 * @param curr_vel, current vehicle velocity
	 */
	void _calculateConstrainedSetpoint(matrix::Vector2f &setpoint, const matrix::Vector2f &curr_pos,
					   const matrix::Vector2f &curr_vel);

	//Timing functions. Necessary to mock time in the tests
	virtual hrt_abstime getTime();
	virtual hrt_duration getElapsedTime(const hrt_abstime& start_time);

private:
	// Parameters
	CollisionPreventionParameters _params;

	bool _interfering{false};		/**< states if the collision prevention interferes with the user input */
	bool _was_active{false};		/**< states if the collision prevention interferes with the user input */

	// the snapshots of the streamed input messages that we use in the calculation 
	ObstacleDistance obstacle_distance;
    VehicleAttitude vehicle_attitude;

	// constraints should be published by the ros node OffboardControl
	CollisionConstraints constraints;

	hrt_abstime _last_timeout_warning{0};
	hrt_abstime _time_activated{0};

	/**
	 * Aggregates the sensor data into a internal obstacle map in body frame
	 */
	void _updateObstacleMap();
};
