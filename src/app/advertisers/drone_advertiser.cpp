/**
 * @brief drone topic advertiser example, which 
 * @file drone_advertiser.cpp
 * @addtogroup examples
 * @author Xiaoang Zhang <jesse1008611@gmail.com>
 */

#include <px4_msgs/msg/obstacle_distance.hpp>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <cmath>
#include <limits>


class DroneAdvertiser : public rclcpp::Node
{
public:
	DroneAdvertiser(const std::string &clockTopic,
                    const std::string &cameraInfoTopic, 
                    const std::string &imageTopic, 
                    const std::string &laserScanTopic) 
    : Node("drone_advertiser")
    {
        ros_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/gz_camera/image", 10);
        ros_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/gz_camera/camera_info", 10);
        ros_obstacle_distance_pub_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);

        gz_node_ = std::make_shared<gz::transport::Node>();

        // subscription to clock
        if (!gz_node_->Subscribe(clockTopic, &DroneAdvertiser::clockCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(),"Error subscribing to Gazebo Sim topic: %s", clockTopic.c_str());
        }

        // subscription to camera info
        if (!gz_node_->Subscribe(cameraInfoTopic, &DroneAdvertiser::cameraInfoCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic: %s", cameraInfoTopic.c_str());
        }

        // subscription to image captured by camera
        if (!gz_node_->Subscribe(imageTopic, &DroneAdvertiser::imageCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic: %s", imageTopic.c_str());
        }

        // subscription to 2d lidar, which serves as the distance sensor
        if (!gz_node_->Subscribe(laserScanTopic, &DroneAdvertiser::laserScanCallback, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic [%s]", laserScanTopic.c_str());
        }
    }
private:
    std::shared_ptr<gz::transport::Node> gz_node_;
    std::string cameraInfoTopic;
    std::string imageTopic;
    uint64_t sim_system_offset_{0};

    // ROS 2 publisher: image, camera info, and obstacle distance
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_pub_;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr ros_obstacle_distance_pub_;

    // Initialize ROS 2 CameraInfo
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    /**
     * @brief 
     * 
     * @param clock clock from Gazebo Sim
     */
    void clockCallback(const gz::msgs::Clock &clock) 
    {
        const uint64_t time_us_sim = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);
        const uint64_t time_us_sys = (clock.system().sec() * 1000000) + (clock.system().nsec() / 1000);

        sim_system_offset_ = time_us_sys - time_us_sim;
    }

    void cameraInfoCallback(const gz::msgs::CameraInfo &msg)
    {   
        // populate the ROS 2 CameraInfo message
        // Convert simulation time to system time
        uint64_t timestamp_us_sim = msg.header().stamp().sec() * 1000000 + msg.header().stamp().nsec() / 1000;
        uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

        // Assign system time to the ROS 2 CameraInfo message header
        camera_info_msg_.header.stamp.sec = static_cast<uint32_t>(timestamp_us_sys / 1e6); // Convert to seconds
        camera_info_msg_.header.stamp.nanosec = static_cast<uint32_t>((timestamp_us_sys % static_cast<uint64_t>(1e6)) * 1000); // Convert to nanoseconds

        std::string frame_id;
        for (const auto &data : msg.header().data()) {
            if (data.key() == "frame_id") {
                camera_info_msg_.header.frame_id = data.value(0);
                break;
            }
        }
        camera_info_msg_.width = msg.width();
        camera_info_msg_.height = msg.height();
        camera_info_msg_.distortion_model = "none";

        // Copy distortion coefficients
        camera_info_msg_.d.assign(msg.distortion().k().begin(), msg.distortion().k().end());

        // Copy intrinsic matrix
        for (int i = 0; i < 9; ++i)
        {
            camera_info_msg_.k[i] = msg.intrinsics().k(i);
        }

        // Copy projection matrix
        for (int i = 0; i < 12; ++i) {
            camera_info_msg_.p[i] = msg.projection().p(i);
        }

        // Copy rectification matrix
        for (int i = 0; i < 9; ++i) {
            camera_info_msg_.r[i] = msg.rectification_matrix(i);
        }

        ros_camera_info_pub_->publish(camera_info_msg_);
    }

    void imageCallback(const gz::msgs::Image &msg)
    {
        int cv_type;

        // Determine OpenCV type based on the Gazebo image format
        if (msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8)
        {
            cv_type = CV_8UC3;
        }
        else if (msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8)
        {
            cv_type = CV_8UC1;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported image format!");
            return;
        }

        // Create OpenCV Mat
        cv::Mat image(msg.height(), msg.width(), cv_type, const_cast<char*>(msg.data().data()));

        // Ensure the image is in RGB format
        if (msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8)
        {
            // Convert grayscale to RGB
            cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
        }

        // // Display the image (optional, convert to BGR if you want to display image)
        // cv::imshow("Gazebo Camera Stream", image_bgr);
        // cv::waitKey(1);

        // Create ROS 2 header
        std_msgs::msg::Header header;
        uint64_t timestamp_us_sim = msg.header().stamp().sec() * 1000000 + msg.header().stamp().nsec() / 1000;
        uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

        header.stamp.sec = static_cast<uint32_t>(timestamp_us_sys / 1e6); // Convert to seconds
        header.stamp.nanosec = static_cast<uint32_t>((timestamp_us_sys % static_cast<uint64_t>(1e6)) * 1000); // Convert to nanoseconds
        header.frame_id = "color_image";

        // Convert OpenCV Mat to ROS 2 Image message as RGB
        sensor_msgs::msg::Image::SharedPtr ros_image = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();
        
        // Publish the RGB image
        ros_image_pub_->publish(*ros_image);
    }


    void laserScanCallback(const gz::msgs::LaserScan &scan)
    {
        static constexpr int SECTOR_SIZE_DEG = 5; // PX4 Collision Prevention uses 36 sectors of 10 degrees each

        double angle_min_deg = scan.angle_min() * 180 / M_PI;
        double angle_step_deg = scan.angle_step() * 180 / M_PI;

        int samples_per_sector = std::round(SECTOR_SIZE_DEG / angle_step_deg);
        int number_of_sectors = scan.ranges_size() / samples_per_sector;

        std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

        // Downsample -- take average of samples per sector
        for (int i = 0; i < number_of_sectors; i++) {
            double sum = 0;
            int samples_used_in_sector = 0;

            for (int j = 0; j < samples_per_sector; j++) {

                double distance = scan.ranges()[i * samples_per_sector + j];

                // inf values mean no object
                if (isinf(distance)) {
                    continue;
                }

                sum += distance;
                samples_used_in_sector++;
            }

            // If all samples in a sector are inf then it means the sector is clear
            if (samples_used_in_sector == 0) {
                ds_array[i] = scan.range_max();

            } else {
                ds_array[i] = sum / samples_used_in_sector;
            }
        }

        // Publish to ObstacleDistance
        px4_msgs::msg::ObstacleDistance obs{};

        // Initialize distances with unknown values
        std::fill(std::begin(obs.distances), std::end(obs.distances), UINT16_MAX);
        
        // timestamps for the uorb topic ObstacleDistance is in microseconds
        uint64_t timestamp_us_sim = scan.header().stamp().sec() * 1000000 + scan.header().stamp().nsec() / 1000;
        uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;
        obs.timestamp = timestamp_us_sys;
        obs.frame = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
        obs.sensor_type = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
        obs.min_distance = static_cast<uint16_t>(scan.range_min() * 100); // in cm
        obs.max_distance = static_cast<uint16_t>(scan.range_max() * 100); // in cm
        obs.angle_offset = static_cast<float>(angle_min_deg); // in degrees
        obs.increment = static_cast<float>(SECTOR_SIZE_DEG); // in degrees


        // Map samples in FOV into sectors in ObstacleDistance
        int index = 0;

        // Iterate in reverse because array is FLU and we need FRD
        for (std::vector<double>::reverse_iterator i = ds_array.rbegin(); i != ds_array.rend(); ++i) {

		    uint16_t distance_cm = (*i) * 100.;

            if (distance_cm >= obs.max_distance) {
                obs.distances[index] = obs.max_distance + 1;

            } else if (distance_cm < obs.min_distance) {
                obs.distances[index] = 0;

            } else {
                obs.distances[index] = distance_cm;
            }

            index++;
        }

        // Publish the ObstacleDistance message
        ros_obstacle_distance_pub_->publish(obs);
    }
};

int main(int argc, char *argv[])
{
	std::cout << "Starting drone advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

    // default settings for model and world
    std::string model_name = "x500_depth";
    std::string world_name = "baylands";

    if (argc < 1) {
        std::string input_arg = argv[1];
        size_t underscore_pos = input_arg.find_last_of("_");
        if (underscore_pos != std::string::npos) {
            model_name = input_arg.substr(0, underscore_pos);
            world_name = input_arg.substr(underscore_pos + 1);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("drone_advertiser"), "Invalid argument format. Expected format: <model_name>_<world_name>");
            return 1;
        }
    }
    // Gazebo Sim Topics to be subscribed
    std::string clockTopic = "/world/" + world_name + "/clock";
    std::string cameraInfoTopic = "/camera_info";
    std::string imageTopic = "/rgbd_camera/image";
    // std::string depthImageTopic = "/rgbd_camera/depth_image";
    std::string laserScanTopic = "/world/"+ world_name + "/model/" + model_name + "_0/model/lidar/link/link/sensor/lidar_2d_v2/scan";

    // Instantiate GZROSCameraAdavertiser and start listening
    auto drone_advertiser = std::make_shared<DroneAdvertiser>(clockTopic,
                                                              cameraInfoTopic, 
                                                              imageTopic, 
                                                              laserScanTopic);
	rclcpp::spin(drone_advertiser);
	rclcpp::shutdown();

	return 0;
}
