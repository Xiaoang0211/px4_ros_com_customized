cmake_minimum_required(VERSION 3.8)

# app/advertisers/drone_advertiser
add_executable(drone_advertiser drone_advertiser.cpp)
ament_target_dependencies(drone_advertiser rclcpp px4_msgs OpenCV cv_bridge)
target_link_libraries(drone_advertiser gz-transport${GZ_TRANSPORT_VER}::core)
install(TARGETS drone_advertiser DESTINATION lib/${PROJECT_NAME})

# app/advertisers/okvis_advertiser
add_executable(okvis_advertiser okvis_advertiser.cpp)
ament_target_dependencies(okvis_advertiser rclcpp px4_msgs OpenCV cv_bridge)
target_link_libraries(okvis_advertiser gz-transport${GZ_TRANSPORT_VER}::core)
install(TARGETS okvis_advertiser DESTINATION lib/${PROJECT_NAME})

