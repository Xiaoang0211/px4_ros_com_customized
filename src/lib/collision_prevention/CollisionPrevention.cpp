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
 * this script should solve the collision prevention problem in 2 cases,
 * proposed for random exploration:
 * 1. convex or plain: For example, pole and tree, the vehicle velocity 
 *    should keep perpendicular to the bin of the closest distance. chan-
 *    ge the yaw angle in the reverse direction of the closest distance.
 * 2. concave: For example, narrow space with a wall at the end, the v-
 * 	  -ehicle should move to the direction with max distance according
 * 	  to the readings of the 2D LiDAR. The yaw angle should also turn to
 *    that direction.
 */

#include <lib/collision_prevention/CollisionPrevention.hpp>
#include <iostream>

using namespace matrix;
using namespace px4_msgs::msg;
using namespace std::chrono;
using namespace std::chrono_literals;

CollisionPrevention::CollisionPrevention(const CollisionPreventionParameters& params) :
	_params(params),
	logger_(rclcpp::get_logger("CollisionPrevention"))
{
	static_assert(BIN_SIZE >= 5, "BIN_SIZE needs to be at least 5");
	static_assert(360 % BIN_SIZE == 0, "BIN_SIZE should divide 360 evenly");

	// initialize internal obstacle map
	_obstacle_map_body_frame.frame = ObstacleDistance::MAV_FRAME_BODY_FRD;
	_obstacle_map_body_frame.increment = BIN_SIZE;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;

	for (uint32_t i = 0 ; i < BIN_COUNT; i++) {
		_obstacle_map_body_frame.distances[i] = UINT16_MAX;
	}
}

/**
 * @brief set current_obstacle_distance obtained from the ros topic /fmu/in/obstacle_distance
 * 
 * @param msg 
 */
void CollisionPrevention::setObstacleDistance(const ObstacleDistance& msg)
{	
	current_obstacle_distance = msg;
	_obstacle_distance_received = true;
}

/**
 * @brief set current vehicle odometry obtained from ros topic /fmu/out/vehicle_odometry
 * 
 * @param msg 
 */
void CollisionPrevention::setVehicleOdometry(const float yaw, const Vector2f velocity)
{	
    current_yaw = yaw;
	current_vel = velocity;
	_vehicle_odometry_received = true;
}


void CollisionPrevention::getCollisionConstraints(CollisionConstraints& msg)
{
	msg = constraints;
}

/**
 * @brief to let the offboard control node be able to get the current obstacle map
 * 
 * @param obstacle_distance_msg
 */
void CollisionPrevention::getObstacleDistanceFused(ObstacleDistance& msg)
{	
	msg = _obstacle_map_body_frame;
}

/**
 * @brief getting system-wide real time since unix epoch
 * 
 * @return hrt_abstime: uint64_t, nanoseconds
 */
hrt_abstime CollisionPrevention::getTime()
{	
	auto now = system_clock::now();
	return duration_cast<nanoseconds>(now.time_since_epoch()).count();
}

/**
 * @brief returning the time duration given time points (absolute system time)
 * 
 * @param start_time 
 * @return hrt_duration: uint64_t, nanoseconds
 */
hrt_duration CollisionPrevention::getElapsedTime(const hrt_abstime& start_time)
{
	return getTime() - start_time;
}

// public function
bool CollisionPrevention::is_active()
{	
	std::unique_lock<std::shared_mutex> lock(data_mutex);
	bool activated = _params.cp_dist > 0;

	if (activated && !_was_active) {
		_time_activated = getTime();
	}

	_was_active = activated;
	return activated;
}


void CollisionPrevention::_addObstacleSensorData(const ObstacleDistance &obstacle, const float vehicle_yaw)
{
	float vehicle_orientation_deg = math::degrees(_vehicle_yaw);

	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		// Obstacle message arrives in local_origin frame (north aligned)
		// corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < BIN_COUNT; i++) {
			for (int j = 0; (j < 360 / obstacle.increment) && (j < BIN_COUNT); j++) {
				float bin_lower_angle = _wrap_360((float)i * _obstacle_map_body_frame.increment + _obstacle_map_body_frame.angle_offset
								  - (float)_obstacle_map_body_frame.increment / 2.f);
				float bin_upper_angle = _wrap_360((float)i * _obstacle_map_body_frame.increment + _obstacle_map_body_frame.angle_offset
								  + (float)_obstacle_map_body_frame.increment / 2.f);
				float msg_lower_angle = _wrap_360((float)j * obstacle.increment + obstacle.angle_offset - vehicle_orientation_deg -
								  obstacle.increment / 2.f);
				float msg_upper_angle = _wrap_360((float)j * obstacle.increment + obstacle.angle_offset - vehicle_orientation_deg +
								  obstacle.increment / 2.f);

				// if a bin stretches over the 0/360 degree line, adjust the angles
				if (bin_lower_angle > bin_upper_angle) {
					bin_lower_angle -= 360;
				}

				if (msg_lower_angle > msg_upper_angle) {
					msg_lower_angle -= 360;
				}

				// Check for overlaps.
				if ((msg_lower_angle > bin_lower_angle && msg_lower_angle < bin_upper_angle) ||
				    (msg_upper_angle > bin_lower_angle && msg_upper_angle < bin_upper_angle) ||
				    (msg_lower_angle <= bin_lower_angle && msg_upper_angle >= bin_upper_angle) ||
				    (msg_lower_angle >= bin_lower_angle && msg_upper_angle <= bin_upper_angle)) {
					if (obstacle.distances[j] != UINT16_MAX) {
						if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[j] * 0.01f)) {
							_obstacle_map_body_frame.distances[i] = obstacle.distances[j];
							_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
							_data_maxranges[i] = obstacle.max_distance;
							_data_fov[i] = 1;
						}
					}
				}

			}
		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		// Obstacle message arrives in body frame (front aligned)
		// corresponding data index (shift by msg offset)
		for (int i = 0; i < BIN_COUNT; i++) {
			for (int j = 0; j < 360 / obstacle.increment; j++) {
				float bin_lower_angle = _wrap_360((float)i * _obstacle_map_body_frame.increment + _obstacle_map_body_frame.angle_offset
								  - (float)_obstacle_map_body_frame.increment / 2.f);
				float bin_upper_angle = _wrap_360((float)i * _obstacle_map_body_frame.increment + _obstacle_map_body_frame.angle_offset
								  + (float)_obstacle_map_body_frame.increment / 2.f);
				float msg_lower_angle = _wrap_360((float)j * obstacle.increment + obstacle.angle_offset - obstacle.increment / 2.f);
				float msg_upper_angle = _wrap_360((float)j * obstacle.increment + obstacle.angle_offset + obstacle.increment / 2.f);

				// if a bin stretches over the 0/360 degree line, adjust the angles
				if (bin_lower_angle > bin_upper_angle) {
					bin_lower_angle -= 360;
				}

				if (msg_lower_angle > msg_upper_angle) {
					msg_lower_angle -= 360;
				}

				// Check for overlaps.
				if ((msg_lower_angle > bin_lower_angle && msg_lower_angle < bin_upper_angle) ||
					(msg_upper_angle > bin_lower_angle && msg_upper_angle < bin_upper_angle) ||
					(msg_lower_angle <= bin_lower_angle && msg_upper_angle >= bin_upper_angle) ||
					(msg_lower_angle >= bin_lower_angle && msg_upper_angle <= bin_upper_angle)) {
					if (obstacle.distances[j] != UINT16_MAX) {

						if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[j] * 0.01f)) {
							_obstacle_map_body_frame.distances[i] = obstacle.distances[j];
							_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
							_data_maxranges[i] = obstacle.max_distance;
							_data_fov[i] = 1;
						}
					}
				}
			}
		}

	}else {
    	RCLCPP_WARN(logger_, "Obstacle message received in unsupported frame: %d", obstacle.frame);	}
}

bool CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
	//use data from this sensor if:
	//1. this sensor data is in range, the bin contains already valid data and this data is coming from the same or less range sensor
	//2. this sensor data is in range, and the last reading was out of range
	//3. this sensor data is out of range, the last reading was as well and this is the sensor with longest range
	//4. this sensor data is out of range, the last reading was valid and coming from the same sensor

	uint16_t sensor_range_cm = static_cast<uint16_t>(100.0f * sensor_range + 0.5f); //convert to cm

	if (sensor_reading < sensor_range) {
		if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
		     && sensor_range_cm <= _data_maxranges[map_index])
		    || _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]) {

			return true;
		}

	} else {
		if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]
		     && sensor_range_cm >= _data_maxranges[map_index])
		    || (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
			&& sensor_range_cm == _data_maxranges[map_index])) {

			return true;
		}
	}

	return false;
}

void CollisionPrevention::_updateObstacleMap()
{	
	if (_obstacle_distance_received) {

		// safe copy of the latest obstacle messages
		_obstacle_distance = current_obstacle_distance;

		// Update map with obstacle data if the data is not stale
		uint64_t obs_elapse_time = getElapsedTime(_obstacle_distance.timestamp);
		if (obs_elapse_time < RANGE_STREAM_TIMEOUT_US && _obstacle_distance.increment > 0.f) {
			_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, _obstacle_distance.timestamp);

			_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
								_obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
								_obstacle_distance.min_distance);
			_addObstacleSensorData(_obstacle_distance, _vehicle_yaw);
		}
	}
}

void CollisionPrevention::_updateObstacleData()
{
	_obstacle_data_present = false;
	_closest_dist = UINT16_MAX; // in meter
	_closest_dist_dir.setZero();

	_farthest_dist = 0.0; // in meter
	_farthest_dist_dir.setZero();

	for (int i = 0; i < BIN_COUNT; i++) {
		// if the data is stale, reset the bin
		if (getTime() - _data_timestamps[i] > RANGE_STREAM_TIMEOUT_US) {
			_obstacle_map_body_frame.distances[i] = UINT16_MAX;
		}

		float angle = wrap_2pi(_vehicle_yaw + math::radians((float)i * BIN_SIZE +
								_obstacle_map_body_frame.angle_offset));
		const Vector2f bin_direction = {cosf(angle), sinf(angle)};
		const uint16_t bin_distance = _obstacle_map_body_frame.distances[i];

		// check if there is available data and the data of the map is not stale
		if (bin_distance < UINT16_MAX
			&& (getTime() - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
				_obstacle_data_present = true;
		}

		if (bin_distance * 0.01f < _closest_dist) {
			_closest_dist = bin_distance * 0.01f;
			_closest_dist_dir = bin_direction;
		}

		if (bin_distance * 0.01f > _farthest_dist) {
			_farthest_dist = bin_distance * 0.01f;
			_farthest_dist_dir = bin_direction;
		}
	}
}

/**
 * @brief
 * 
 * @param setpoint_dir direction of velocity setpoint
 * @param setpoint_index bin index of velocity direction
 * @param setpoint_yaw setpoint for yaw angle in earth fixed ned frame
 */
void CollisionPrevention::_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index, float &setpoint_yaw)
{	
	_is_concave = _concaveDetection(setpoint_dir);
	if (!_is_concave) {
		// convex & plain case
		// determine new velocity direction
		Vector2f normal_component = _closest_dist_dir * (_setpoint_dir.dot(_closest_dist_dir));
		Vector2f tangential_component = _setpoint_dir - normal_component; // new velocity direction
		tangential_component = tangential_component.unit_or_zero(); 
		if (tangential_component(0) == 0 && tangential_component(1) == 0) {
			// if zero then rotate by 90° (tangent of obstacle)
			setpoint_dir = {_closest_dist_dir(1), -_closest_dist_dir(0)};
		} else {
			// if not zero then move in tangent direction of obstacle
			setpoint_dir = tangential_component;
		}

		// determin new camera direction (yaw angle) in ned frame
		Vector2f camera_dir = -_closest_dist_dir;
		setpoint_yaw = atan2f(camera_dir(1), camera_dir(0));
	} else {
		// concave case
		// update the velocity setpoint direction
		setpoint_dir = -_closest_dist_dir;
		// setpoint_dir = _farthest_dist_dir;
		// update the yaw setpoint (rad)
		setpoint_yaw = atan2f(setpoint_dir(1), setpoint_dir(0));
	}
}


void CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint_vel, const Vector2f &_vehicle_vel, float &setpoint_yaw, const float &_vehicle_yaw)
{	
	const float setpoint_length = setpoint_vel.norm();
	_min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, _params.cp_dist);

	if (_obstacle_data_present) {
		// if the obstacle vehicle velocity direction is smaller or equal to cp_dist, then triggere the 
		// function to let the velocity clock wise rotate for 90°, also change the yaw angle.  
		_setpoint_index = _getDirectionIndexBodyFrame(setpoint_vel);
		// change setpoint direction slightly (max by _params.cp_guide_ang degrees) to help guide through narrow gaps
		_setpoint_dir = setpoint_vel.unit_or_zero(); // setpoint direction, unit vector
		// RCLCPP_INFO(logger_, "Closest obstacle distance: %f", _closest_dist);
		if (_closest_dist <= _params.cp_dist) {
			_adaptSetpointDirection(_setpoint_dir, _setpoint_index, setpoint_yaw);
		}

		if (_checkSetpointDirectionFeasibility()) {
			setpoint_vel = _setpoint_dir * setpoint_length;
		} else {
			setpoint_vel.setZero();
		}
	} else {
		//allow no movement
		setpoint_vel.setZero();
	}
}

float CollisionPrevention::_getObstacleDistance(const Vector2f &direction)
{
	float obstacle_distance = 0.f;
	const float direction_norm = direction.norm();

	if (direction_norm > FLT_EPSILON) {
		Vector2f dir = direction / direction_norm;
		const float sp_angle_body_frame = atan2f(dir(1), dir(0)) - _vehicle_yaw;
		const float sp_angle_with_offset_deg =
			_wrap_360(math::degrees(sp_angle_body_frame) - _obstacle_map_body_frame.angle_offset);
		int dir_index = floor(sp_angle_with_offset_deg / BIN_SIZE);
		dir_index = math::constrain(dir_index, 0, BIN_COUNT - 1);
		obstacle_distance = _obstacle_map_body_frame.distances[dir_index] * 0.01f;
	}
	return obstacle_distance;
}

int CollisionPrevention::_getDirectionIndexBodyFrame(const Vector2f &direction)
{
	const float sp_angle_body_frame = atan2f(direction(1), direction(0)) - _vehicle_yaw;
	const float sp_angle_with_offset_deg = _wrap_360(math::degrees(sp_angle_body_frame) -
							_obstacle_map_body_frame.angle_offset);
	return (int) floor(sp_angle_with_offset_deg / BIN_SIZE);
}

bool CollisionPrevention::_checkSetpointDirectionFeasibility()
{
	bool setpoint_feasible = true;

	for (int i = 0; i < BIN_COUNT; i++) {
		// check if our setpoint is either pointing in a direction where data exists, or if not, wether we are allowed to go where there is no data
		if ((_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == _setpoint_index) && (!_params.cp_go_no_data
				|| (_params.cp_go_no_data && _data_fov[i]))) {
			setpoint_feasible =  false;
		}
	}
	return setpoint_feasible;
}

float CollisionPrevention::_getScale(const float &reference_distance)
{
	// Compute the linear scale
    float scale = (reference_distance - _min_dist_to_keep) / (_params.mpc_xy_vel_max * _params.cp_dist);

    // Square the scale for a smoother velocity reduction near obstacles
    scale = scale > 0 ? powf(scale, 2) : 0.0f;

    // Clamp the scale to [0, 1]
    scale = std::clamp(scale, 0.0f, 1.0f);

    return scale;
}

bool CollisionPrevention::_concaveDetection(const Vector2f velocity_dir)
{
    const float angle_range = math::radians(90.0f); // ±60° in radians
    const int num_points = 12; // Number of points to check
    const float step_size = 2 * angle_range / (num_points - 1); // Angular step size
	const int velocity_bin = _getDirectionIndexBodyFrame(velocity_dir);

    // Transform LiDAR points into the velocity-based coordinate system
    Vector2f right_dir = {velocity_dir(1), -velocity_dir(0)}; // Perpendicular to velocity direction

    // Points at ±60° (endpoints of the line)
    int left_bin = velocity_bin + (int)(math::degrees(-angle_range) / BIN_SIZE + BIN_COUNT) % BIN_COUNT;
    int right_bin = velocity_bin + (int)(math::degrees(angle_range) / BIN_SIZE + BIN_COUNT) % BIN_COUNT;

    Vector2f left_point = _getPointVelocityFrame(left_bin, velocity_dir, right_dir);
    Vector2f right_point = _getPointVelocityFrame(right_bin, velocity_dir, right_dir);

    // Calculate the line connecting the endpoints (y = mx + c)
    float slope = (right_point(1) - left_point(1)) / (right_point(0) - left_point(0));
    float intercept = left_point(1) - slope * left_point(0);

    // Check all points in the range
    int concave_count = 0;
    int non_concave_count = 0;

    for (int i = 0; i < num_points; i++) {
        float angle = -angle_range + i * step_size; // Current angle
        int bin_index = (int)(math::degrees(angle) / BIN_SIZE + BIN_COUNT) % BIN_COUNT;
        Vector2f point = _getPointVelocityFrame(bin_index, velocity_dir, right_dir);

        // Calculate the y-value of the line at the point's x-coordinate
        float line_y = slope * point(0) + intercept;

        // Determine if the point is concave or not
        if (point(1) < line_y) {
            concave_count++;
        } else {
            non_concave_count++;
        }
    }

    // Return true if more concave points than non-concave points
    return concave_count > non_concave_count;
}

/**
 * @brief get the scan point in setpoint velocity coordinate frame
 * 
 * @param bin_index 
 * @param velocity_dir 
 * @param right_dir 
 * @return Vector2f 
 */
Vector2f CollisionPrevention::_getPointVelocityFrame(int bin_index, const Vector2f &velocity_dir, const Vector2f &right_dir)
{
    float bin_distance = _obstacle_map_body_frame.distances[bin_index] * 0.01f; // Convert to meters
    float bin_angle = math::radians(bin_index * BIN_SIZE);

	float bin_angle_ned = wrap_2pi(bin_angle + _vehicle_yaw);
	float x_ned = bin_distance * cosf(bin_angle_ned);
	float y_ned = bin_distance * sinf(bin_angle_ned);

    // transform to velocity-aligned coordinate system
    float x = x_ned * right_dir(0) + y_ned * velocity_dir(0);
    float y = x_ned * right_dir(1) + y_ned * velocity_dir(1);
    return Vector2f(x, y);
}


void CollisionPrevention::modifySetpoint(Vector2f& setpoint_vel, float &setpoint_yaw)
{	
	std::unique_lock<std::shared_mutex> lock(data_mutex);

	// safe copy of vehicle odometry
	if (_vehicle_odometry_received) {
		_vehicle_yaw = current_yaw;
		_vehicle_vel = current_vel;
	}
	
	//calculate movement constraints based on range data
	const Vector2f original_setpoint_vel = setpoint_vel;

	_updateObstacleMap();
	_updateObstacleData();

	// modify this function
	_calculateConstrainedSetpoint(setpoint_vel, _vehicle_vel, setpoint_yaw, _vehicle_yaw);

	// update collision constraints to UORB topic CollisionConstraints
	original_setpoint_vel.copyTo(constraints.original_setpoint.data());
	setpoint_vel.copyTo(constraints.adapted_setpoint.data());
	constraints.timestamp = getTime();

	// reset flags
	_obstacle_distance_received = false;
    _vehicle_odometry_received = false;
}

float CollisionPrevention::_wrap_360(const float f)
{
	return wrap(f, 0.f, 360.f);
}

int CollisionPrevention::_wrap_bin(int i)
{
	i = i % BIN_COUNT;

	while (i < 0) {
		i += BIN_COUNT;
	}

	return i;
}