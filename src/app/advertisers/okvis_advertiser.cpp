/**
 * @brief okvis topic advertiser, which publishes camera, imu, and LiDAR sensor data
 * in Gazebo Sim as ROS2 topic with the names given in okvis2.
 * 
 * @file okvis_advertiser.cpp
 * @addtogroup app
 * @author Xiaoang Zhang <jesse1008611@gmail.com>
 */

#include <app/advertiser/okvis_advertiser.hpp>

OkvisAdvertiser::OkvisAdvertiser(const std::string &clockTopic,
                const std::string &cameraInfoTopic, 
                const std::string &colorImageTopic, 
                const std::string &depthImageTopic, 
                const std::string &laserScanTopic) 
: Node("okvis_advertiser")
{   
    qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/okvis/cam0/image_raw", 10);
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/okvis/cam1/image_raw", 10);
    color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/okvis/cam0_info", 10);
    depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/okvis/cam1_info", 10);
    obstacle_distance_pub_ = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/okvis/imu0", 10);

    gz_node_ = std::make_shared<gz::transport::Node>();


    // subscription to clock topic in the simulated world
    if (!gz_node_->Subscribe(clockTopic, &OkvisAdvertiser::clockCallback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic: %s", clockTopic.c_str());
    }

    // subscription to camera info
    if (!gz_node_->Subscribe(cameraInfoTopic, &OkvisAdvertiser::cameraInfoCallback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic: %s", cameraInfoTopic.c_str());
    }

    // subscription to rgb image 
    if (!gz_node_->Subscribe(colorImageTopic, &OkvisAdvertiser::colorImageCallback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic: %s", colorImageTopic.c_str());
    }

    // subscription to depth image
    if (!gz_node_->Subscribe(depthImageTopic, &OkvisAdvertiser::depthImageCallback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic: %s", depthImageTopic.c_str());
    }

    // subscription to 2d lidar, which serves as the distance sensor
    if (!gz_node_->Subscribe(laserScanTopic, &OkvisAdvertiser::laserScanCallback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to Gazebo Sim topic [%s]", laserScanTopic.c_str());
    }

    // subscription to sensor_combined, which serves as the imu input
    sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
        "/fmu/out/sensor_combined", qos,
        [this](const px4_msgs::msg::SensorCombined::SharedPtr msg) {
            sensor_msgs::msg::Imu imu_msg;

            // Convert timestamp (system time) from microseconds to nanoseconds
            uint64_t timestamp_ns = msg->timestamp * 1000; // Microseconds to nanoseconds
            imu_msg.header.stamp.sec = static_cast<uint32_t>(timestamp_ns / 1e9); // Seconds
            imu_msg.header.stamp.nanosec = static_cast<uint32_t>(timestamp_ns % static_cast<uint64_t>(1e9)); // Nanoseconds

            imu_msg.header.frame_id = "IMU0";

            // Assign accelerometer data
            imu_msg.linear_acceleration.x = msg->accelerometer_m_s2[0];
            imu_msg.linear_acceleration.y = msg->accelerometer_m_s2[1];
            imu_msg.linear_acceleration.z = msg->accelerometer_m_s2[2];

            // Assign gyroscope data
            imu_msg.angular_velocity.x = msg->gyro_rad[0];
            imu_msg.angular_velocity.y = msg->gyro_rad[1];
            imu_msg.angular_velocity.z = msg->gyro_rad[2];

            // Set orientation to 0, as the SensorCombined message does not provide orientation
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 1.0;

            // Publish the converted IMU message
            imu_pub_->publish(imu_msg);
        }
    );
}

void OkvisAdvertiser::clockCallback(const gz::msgs::Clock &clock)
{
	const uint64_t time_us_sim = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);
    const uint64_t time_us_sys = (clock.system().sec() * 1000000) + (clock.system().nsec() / 1000);

    sim_system_offset_ = time_us_sys - time_us_sim;
}

void OkvisAdvertiser::publishCameraInfo(sensor_msgs::msg::CameraInfo &camera_info_msg, 
                                        const gz::msgs::CameraInfo &msg,
                                        const std::string &frame_id) 
{
    // Populate CameraInfo fields from gz::msgs::CameraInfo
    camera_info_msg.header.frame_id = frame_id;
    camera_info_msg.width = msg.width();
    camera_info_msg.height = msg.height();
    camera_info_msg.distortion_model = "none";

    // Copy distortion coefficients
    camera_info_msg.d.assign(msg.distortion().k().begin(), msg.distortion().k().end());

    // Copy intrinsic matrix
    for (int i = 0; i < 9; ++i) {
        camera_info_msg.k[i] = msg.intrinsics().k(i);
    }

    // Copy projection matrix
    for (int i = 0; i < 12; ++i) {
        camera_info_msg.p[i] = msg.projection().p(i);
    }

    // Copy rectification matrix
    for (int i = 0; i < 9; ++i) {
        camera_info_msg.r[i] = msg.rectification_matrix(i);
    }
    
    // // publish to topics w.r.t frame id
    // if (frame_id == "x500_depth_0::OakD-Lite/base_link::IMX214") {
    //     this->color_info_pub_->publish(camera_info_msg);
    //     cam0_info_ = camera_info_msg;
    // } else if (frame_id == "x500_depth_0::OakD-Lite/base_link::StereoOV7251")
    // {
    //     this->depth_info_pub_->publish(camera_info_msg);
    //     cam1_info_ = camera_info_msg;
    // }
    
    if (frame_id == "x500_depth_0::OakD-Lite/base_link::OakD-Lite") {
        // x500_depth's rgb and depth images are already aligned
        this->color_info_pub_->publish(camera_info_msg);
        this->depth_info_pub_->publish(camera_info_msg);
        cam0_info_ = camera_info_msg;
        cam1_info_ = camera_info_msg;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown frame id!");
    }

}


void OkvisAdvertiser::cameraInfoCallback(const gz::msgs::CameraInfo &gz_cam_info)
{   
    sensor_msgs::msg::CameraInfo camera_info_msg;

    // Convert simulation time to system time
    uint64_t timestamp_us_sim = gz_cam_info.header().stamp().sec() * 1000000 + gz_cam_info.header().stamp().nsec() / 1000;
    uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

    // Assign system time to the ROS 2 CameraInfo message header
    camera_info_msg.header.stamp.sec = static_cast<uint32_t>(timestamp_us_sys / 1e6); // Convert to seconds
    camera_info_msg.header.stamp.nanosec = static_cast<uint32_t>((timestamp_us_sys % static_cast<uint64_t>(1e6)) * 1000); // Convert to nanoseconds

    std::string frame_id;
    for (const auto &data : gz_cam_info.header().data()) {
        if (data.key() == "frame_id") {
            frame_id = data.value(0);
            break;
        }
    }
    publishCameraInfo(camera_info_msg, gz_cam_info, frame_id);
}

void OkvisAdvertiser::colorImageCallback(const gz::msgs::Image &color)
{   
    int cv_type;

    if (color.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8)
    {
        cv_type = CV_8UC3;
    } else if (color.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8)
    {
        cv_type = CV_8UC1;
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported image format!");
        return;
    }

    cv::Mat image(color.height(), color.width(), cv_type, const_cast<char*>(color.data().data()));
    
    // Convert RGB to BGR for OpenCV if necessary
    if (color.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
        cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
    }

    color_image_ = image.clone();
    
    // Convert OpenCV Mat to ROS 2 Image message using cv_bridge
    std_msgs::msg::Header header;
    uint64_t timestamp_us_sim = color.header().stamp().sec() * 1000000 + color.header().stamp().nsec() / 1000;
    uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

    header.stamp.sec = static_cast<uint32_t>(timestamp_us_sys / 1e6);
    header.stamp.nanosec = static_cast<uint32_t>((timestamp_us_sys % static_cast<uint64_t>(1e6)) * 1000);
    header.frame_id = "color_image";

    // copy of time rgb image time stamp to the member variable color_image_stamp_
    // color_image_stamp_ = rclcpp::Time(header.stamp.sec, header.stamp.nanosec);
    color_image_stamp_ = rclcpp::Time(color.header().stamp().sec(), color.header().stamp().nsec());

    sensor_msgs::msg::Image::SharedPtr color_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    color_image_pub_->publish(*color_image_msg);
}

// /**
//  * @brief Callback for subscribing the depth image. The depth image should be 
//  * resized so that it is aligned with the RGB image. We resize the depth imag
//  * -e instead of RGB because it has a larger FOV according to the settings in
//  * PX4-Autopilot.
//  * 
//  * @param msg Gazebo Sim message of depth image
//  */
// void OkvisAdvertiser::depthImageCallback(const gz::msgs::Image &depth)
// {   
//     if (depth.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32) {
//         RCLCPP_ERROR(this->get_logger(), "Unsupported image format!");
//         return;
//     }

//     // header time stamp for depth image
//     std_msgs::msg::Header header;

//     uint64_t timestamp_us_sim = depth.header().stamp().sec() * 1000000 + depth.header().stamp().nsec() / 1000;
//     uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

//     header.stamp.sec = static_cast<uint32_t>(timestamp_us_sys / 1e6);
//     header.stamp.nanosec = static_cast<uint32_t>((timestamp_us_sys % static_cast<uint64_t>(1e6)) * 1000);
//     header.frame_id = "depth_image";

//     // check if color and depth images are temporally aligned
//     rclcpp::Time depth_image_stamp(header.stamp.sec, header.stamp.nanosec);
//     auto time_difference = (depth_image_stamp - color_image_stamp_).nanoseconds();
//     // RCLCPP_INFO(this->get_logger(), "Time differece: %.2f ms", time_difference / 1e6);
    
//     if (std::abs(time_difference) > 10 * 1e6) { // we allow max. 10ms time difference between color and depth image
//         RCLCPP_WARN(this->get_logger(),
//                     "Depth image timestamp and color image timestamp are not aligned: "
//                     "time difference = %.2f ms", time_difference / 1e6);
//     }

//     int color_width = cam0_info_.width;
//     int color_height = cam0_info_.height;
//     int depth_width = cam1_info_.width;
//     int depth_height = cam1_info_.height;

//     cv::Mat depth_image(depth_height, depth_width, CV_32FC1, const_cast<char*>(depth.data().data()));
//     depth_image.setTo(30.0, depth_image == std::numeric_limits<float>::infinity()); // Replace inf with 10.0 meters
    
//     // // Normalize for visualization
//     // double minVal, maxVal;
//     // cv::minMaxLoc(depth_image, &minVal, &maxVal);
//     // RCLCPP_INFO(this->get_logger(), "Original Depth min: %.2f, max: %.2f", minVal, maxVal);
//     // cv::Mat depthVisOriginal;
//     // depth_image.convertTo(depthVisOriginal, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
//     // cv::applyColorMap(depthVisOriginal, depthVisOriginal, cv::COLORMAP_VIRIDIS);

//     // // Display the original depth image
//     // cv::imshow("Original Depth", depthVisOriginal);
//     // cv::waitKey(1);


//     // Create maps for remapping
//     cv::Mat map_x(color_height, color_width, CV_32FC1);
//     cv::Mat map_y(color_height, color_width, CV_32FC1);

//     // Initialize the maps with -1 (invalid values)
//     map_x.setTo(-1.0f);
//     map_y.setTo(-1.0f);

//     // Populate the remap matrices
//     for (int v_d = 0; v_d < depth_height; ++v_d) {
//         for (int u_d = 0; u_d < depth_width; ++u_d) {
//             float D = depth_image.at<float>(v_d, u_d);

//             if (D <= 0.0f) {
//                 continue; // Skip invalid depth values
//             }

//             float Xd = (u_d - cam1_info_.k[2]) * D / cam1_info_.k[0];
//             float Yd = (v_d - cam1_info_.k[5]) * D / cam1_info_.k[4];
//             float Zd = D;

//             cv::Mat pt_depth = (cv::Mat_<float>(3, 1) << Xd, Yd, Zd);
//             cv::Mat pt_color = R_ * pt_depth + t_; // Transformation from cam1 to cam0

//             float Xc = pt_color.at<float>(0, 0);
//             float Yc = pt_color.at<float>(1, 0);
//             float Zc = pt_color.at<float>(2, 0);

//             if (Zc <= 0.0f) {
//                 continue; // Skip invalid transformations
//             }

//             float u_c = cam0_info_.k[0] * (Xc / Zc) + cam0_info_.k[2];
//             float v_c = cam0_info_.k[4] * (Yc / Zc) + cam0_info_.k[5];

//             if (u_c >= 0 && u_c < color_width && v_c >= 0 && v_c < color_height) {
//                 map_x.at<float>(static_cast<int>(v_c), static_cast<int>(u_c)) = u_d;
//                 map_y.at<float>(static_cast<int>(v_c), static_cast<int>(u_c)) = v_d;
//             }
//         }
//     }

//     // Use remap to interpolate the depth image
//     cv::Mat aligned_depth_image;
//     cv::remap(depth_image, aligned_depth_image, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0.0f);

//     // visualization:
//     double minVal, maxVal;
//     cv::minMaxLoc(aligned_depth_image, &minVal, &maxVal);
//     RCLCPP_INFO(this->get_logger(), "Depth min: %.2f, max: %.2f", minVal, maxVal);
//     cv::Mat depthVis;
//     aligned_depth_image.convertTo(depthVis, CV_8UC1, 255.0/(maxVal - minVal), -minVal*255.0/(maxVal - minVal));
//     cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_JET);

//     // Show
//     cv::imshow("Aligned Depth", depthVis);
//     cv::imshow("RGB", color_image_);
//     cv::waitKey(1);

//     sensor_msgs::msg::Image::SharedPtr depth_image_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, aligned_depth_image).toImageMsg();
//     depth_image_pub_->publish(*depth_image_msg);
// }

/**
 * @brief Callback for subscribing the depth image. The depth image should be 
 * resized so that it is aligned with the RGB image. We resize the depth imag
 * -e instead of RGB because it has a larger FOV according to the settings in
 * PX4-Autopilot.
 * 
 * @param msg Gazebo Sim message of depth image
 */
void OkvisAdvertiser::depthImageCallback(const gz::msgs::Image &depth)
{   
    if (depth.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32) {
        RCLCPP_ERROR(this->get_logger(), "Unsupported image format!");
        return;
    }

    // header time stamp for depth image
    std_msgs::msg::Header header;

    uint64_t timestamp_us_sim = depth.header().stamp().sec() * 1000000 + depth.header().stamp().nsec() / 1000;
    uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

    header.stamp.sec = static_cast<uint32_t>(timestamp_us_sys / 1e6);
    header.stamp.nanosec = static_cast<uint32_t>((timestamp_us_sys % static_cast<uint64_t>(1e6)) * 1000);
    header.frame_id = "depth_image";

    // check if color and depth images are temporally aligned
    // rclcpp::Time depth_image_stamp(header.stamp.sec, header.stamp.nanosec);
    rclcpp::Time depth_image_stamp(depth.header().stamp().sec(), depth.header().stamp().nsec());
    auto time_difference = (depth_image_stamp - color_image_stamp_).nanoseconds();
    // RCLCPP_INFO(this->get_logger(), "Time differece: %.2f ms", time_difference / 1e6);
    
    if (std::abs(time_difference) > 10 * 1e6) { // we allow max. 10ms time difference between color and depth image
        RCLCPP_WARN(this->get_logger(),
                    "Depth image timestamp and color image timestamp are not aligned: "
                    "time difference = %.2f ms", time_difference / 1e6);
    }

    cv::Mat depth_image(depth.height(), depth.width(), CV_32FC1, const_cast<char*>(depth.data().data()));
    depth_image.setTo(10.0, depth_image == std::numeric_limits<float>::infinity()); // Replace inf with 10.0 meters, refer to the gz sdf file 
    
    sensor_msgs::msg::Image::SharedPtr depth_image_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, depth_image).toImageMsg();
    depth_image_pub_->publish(*depth_image_msg);
}

void OkvisAdvertiser::laserScanCallback(const gz::msgs::LaserScan &scan)
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
    
    // time stamp, sim time
    uint64_t timestamp_us_sim = scan.header().stamp().sec() * 1000000 + scan.header().stamp().nsec() / 1000;
    uint64_t timestamp_us_sys = timestamp_us_sim + sim_system_offset_;

    obs.timestamp = timestamp_us_sys; // system time in microsecods
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
    obstacle_distance_pub_->publish(obs);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting okvis advertiser node..." << std::endl;
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
            RCLCPP_ERROR(rclcpp::get_logger("okvis_advertiser"), "Invalid argument format. Expected format: <model_name>_<world_name>");
            return 1;
        }
    }
    // Gazebo Sim Topics to be subscribed
    std::string clockTopic = "/world/" + world_name + "/clock";
    std::string cameraInfoTopic = "/camera_info";
    std::string colorImageTopic = "/rgbd_camera/image";
    std::string depthImageTopic = "/rgbd_camera/depth_image";
    std::string laserScanTopic = "/world/"+ world_name + "/model/" + model_name + "_0/model/lidar/link/link/sensor/lidar_2d_v2/scan";

    // Instantiate advertiser and start listening
    auto okvis_advertiser = std::make_shared<OkvisAdvertiser>(clockTopic,
                                                              cameraInfoTopic, 
                                                              colorImageTopic, 
                                                              depthImageTopic, 
                                                              laserScanTopic);
	rclcpp::spin(okvis_advertiser);
	rclcpp::shutdown();

	return 0;
}
