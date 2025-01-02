
#pragma once

#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/transport/Node.hh>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <cmath>
#include <limits>

class OkvisAdvertiser : public rclcpp::Node
{
public:
    OkvisAdvertiser(const std::string &clockTopic,
                    const std::string &cameraInfoTopic, 
                    const std::string &colorImageTopic, 
                    const std::string &depthImageTopic, 
                    const std::string &laserScanTopic);
    ~OkvisAdvertiser() override = default;

    rmw_qos_profile_t qos_profile;

private:
    cv::Mat color_image_;
    rclcpp::Time color_image_stamp_;
    sensor_msgs::msg::CameraInfo cam0_info_; // camera for colored image
    sensor_msgs::msg::CameraInfo cam1_info_; // camera for depth image
    uint64_t sim_system_offset_{0};

    // transformation from cam0 to cam1, in our specific case there is no offset
    cv::Mat R_ = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat t_ = (cv::Mat_<float>(3, 1) << 0, 0, 0);

    // gazebo node
    std::shared_ptr<gz::transport::Node> gz_node_;

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  color_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  depth_info_pub_;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // subscriber for sensor combined
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;

    // callbacks for topic subscription in gz sim
    void clockCallback(const gz::msgs::Clock &clock);
    void cameraInfoCallback(const gz::msgs::CameraInfo &msg);
    void publishCameraInfo(sensor_msgs::msg::CameraInfo &camera_info_msg, 
                            const gz::msgs::CameraInfo &msg,
                            const std::string &frame_id);
    void colorImageCallback(const gz::msgs::Image &color);
    void depthImageCallback(const gz::msgs::Image &depth);
    void laserScanCallback(const gz::msgs::LaserScan &scan);
};