// CollisionPreventionTest.cpp

#include <gtest/gtest.h>
#include <lib/collision_prevention/CollisionPrevention.hpp>

// Test class derived from CollisionPrevention to override timing functions and expose protected methods
class CollisionPreventionTestNode : public CollisionPrevention {
public:
    CollisionPreventionTestNode()
        : CollisionPrevention()
    {
        _simulated_time = 0;
    }

    // Override timing functions to control time in tests
    hrt_abstime hrt_absolute_time() override {
        return _simulated_time;
    }

    void set_simulated_time(hrt_abstime time) {
        _simulated_time = time;
    }

    void increment_simulated_time(hrt_duration delta_time) {
        _simulated_time += delta_time;
    }

    // Methods to set obstacle data and vehicle attitude data directly
    void setObstacleDistance(const ObstacleDistance &obstacle_distance) {
        _latest_obstacle_distance = obstacle_distance;
        _new_obstacle_distance_received = true;
    }

    void setVehicleAttitude(const VehicleAttitude &vehicle_attitude) {
        _latest_vehicle_attitude = vehicle_attitude;
        _new_vehicle_attitude_received = true;
    }

    // Expose protected methods for testing
    void calculateConstrainedSetpoint(matrix::Vector2f &setpoint, const matrix::Vector2f &curr_pos,
                                      const matrix::Vector2f &curr_vel) {
        _calculateConstrainedSetpoint(setpoint, curr_pos, curr_vel);
    }

private:
    hrt_abstime _simulated_time;
};

class CollisionPreventionTest : public ::testing::Test {
protected:
    CollisionPreventionTest() {
        // Initialize ROS 2
        rclcpp::init(0, nullptr);
        collision_prevention_node = std::make_shared<CollisionPreventionTestNode>();
    }

    ~CollisionPreventionTest() override {
        // Shutdown ROS 2
        rclcpp::shutdown();
    }

    std::shared_ptr<CollisionPreventionTestNode> collision_prevention_node;
};

TEST_F(CollisionPreventionTest, NoObstacleTest) {
    // Set up test with no obstacle data
    collision_prevention_node->set_simulated_time(1000000); // 1 second

    // Create a setpoint
    matrix::Vector2f original_setpoint(1.0f, 0.0f);
    const float max_speed = 1.0f;
    matrix::Vector2f curr_pos(0.0f, 0.0f);
    matrix::Vector2f curr_vel(0.0f, 0.0f);

    // Ensure that no obstacle data is present
    ObstacleDistance obstacle_distance;
    obstacle_distance.timestamp = collision_prevention_node->hrt_absolute_time();
    obstacle_distance.increment = 10.0f; // Degrees per bin
    obstacle_distance.min_distance = 0;
    obstacle_distance.max_distance = 5000; // 50 meters
    obstacle_distance.frame = ObstacleDistance::MAV_FRAME_BODY_FRD;
    std::fill(std::begin(obstacle_distance.distances), std::end(obstacle_distance.distances), UINT16_MAX);
    collision_prevention_node->setObstacleDistance(obstacle_distance);

    // VehicleAttitude message with no rotation
    VehicleAttitude vehicle_attitude;
    vehicle_attitude.timestamp = collision_prevention_node->hrt_absolute_time();
    vehicle_attitude.q = {1.0f, 0.0f, 0.0f, 0.0f};
    collision_prevention_node->setVehicleAttitude(vehicle_attitude);

    // Call modifySetpoint
    collision_prevention_node->modifySetpoint(original_setpoint, max_speed, curr_pos, curr_vel);

    // Expect that original_setpoint remains unchanged, as there are no obstacles
    EXPECT_FLOAT_EQ(original_setpoint(0), 1.0f);
    EXPECT_FLOAT_EQ(original_setpoint(1), 0.0f);
}

TEST_F(CollisionPreventionTest, ObstacleInFrontTest) {
    // Set up test with an obstacle directly in front
    collision_prevention_node->set_simulated_time(1000000); // 1 second

    // Create a setpoint
    matrix::Vector2f original_setpoint(1.0f, 0.0f);
    const float max_speed = 1.0f;
    matrix::Vector2f curr_pos(0.0f, 0.0f);
    matrix::Vector2f curr_vel(0.0f, 0.0f);

    // ObstacleDistance message with an obstacle at 5 meters directly in front (0 degrees)
    ObstacleDistance obstacle_distance;
    obstacle_distance.timestamp = collision_prevention_node->hrt_absolute_time();
    obstacle_distance.increment = 10.0f; // Degrees per bin
    obstacle_distance.min_distance = 20; // 0.2 meters
    obstacle_distance.max_distance = 5000; // 50 meters
    obstacle_distance.frame = ObstacleDistance::MAV_FRAME_BODY_FRD;
    std::fill(std::begin(obstacle_distance.distances), std::end(obstacle_distance.distances), UINT16_MAX);
    // Set obstacle at 0 degrees
    obstacle_distance.distances[0] = 500; // 5 meters (in cm)
    collision_prevention_node->setObstacleDistance(obstacle_distance);

    // VehicleAttitude message with no rotation
    VehicleAttitude vehicle_attitude;
    vehicle_attitude.timestamp = collision_prevention_node->hrt_absolute_time();
    vehicle_attitude.q = {1.0f, 0.0f, 0.0f, 0.0f};
    collision_prevention_node->setVehicleAttitude(vehicle_attitude);

    // Call modifySetpoint
    collision_prevention_node->modifySetpoint(original_setpoint, max_speed, curr_pos, curr_vel);

    // Expect that the setpoint is reduced due to the obstacle
    EXPECT_LT(original_setpoint(0), 1.0f);
    EXPECT_FLOAT_EQ(original_setpoint(1), 0.0f);
}

TEST_F(CollisionPreventionTest, ObstacleOnRightTest) {
    // Set up test with an obstacle to the right
    collision_prevention_node->set_simulated_time(1000000); // 1 second

    // Create a setpoint aiming forward
    matrix::Vector2f original_setpoint(1.0f, 0.0f);
    const float max_speed = 1.0f;
    matrix::Vector2f curr_pos(0.0f, 0.0f);
    matrix::Vector2f curr_vel(0.0f, 0.0f);

    // ObstacleDistance message with an obstacle at 90 degrees (to the right)
    ObstacleDistance obstacle_distance;
    obstacle_distance.timestamp = collision_prevention_node->hrt_absolute_time();
    obstacle_distance.increment = 10.0f; // Degrees per bin
    obstacle_distance.min_distance = 20; // 0.2 meters
    obstacle_distance.max_distance = 5000; // 50 meters
    obstacle_distance.frame = ObstacleDistance::MAV_FRAME_BODY_FRD;
    std::fill(std::begin(obstacle_distance.distances), std::end(obstacle_distance.distances), UINT16_MAX);
    // Set obstacle at 90 degrees (bin index 9)
    obstacle_distance.distances[9] = 500; // 5 meters (in cm)
    collision_prevention_node->setObstacleDistance(obstacle_distance);

    // VehicleAttitude message with no rotation
    VehicleAttitude vehicle_attitude;
    vehicle_attitude.timestamp = collision_prevention_node->hrt_absolute_time();
    vehicle_attitude.q = {1.0f, 0.0f, 0.0f, 0.0f};
    collision_prevention_node->setVehicleAttitude(vehicle_attitude);

    // Call modifySetpoint
    collision_prevention_node->modifySetpoint(original_setpoint, max_speed, curr_pos, curr_vel);

    // Expect that the setpoint remains largely unaffected, as the obstacle is to the side
    EXPECT_FLOAT_EQ(original_setpoint(0), 1.0f);
    EXPECT_FLOAT_EQ(original_setpoint(1), 0.0f);
}

