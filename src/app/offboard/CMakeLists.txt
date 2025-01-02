cmake_minimum_required(VERSION 3.8)

# app/offboard/offboard_control
add_executable(offboard_control offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
target_link_libraries(offboard_control mathlib matrix)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# app/offboard/OffboardControl 
add_executable(OffboardControl OffboardControl.cpp)
ament_target_dependencies(OffboardControl rclcpp px4_msgs)
target_link_libraries(OffboardControl mathlib matrix CollisionPrevention RandomExplore)
install(TARGETS OffboardControl DESTINATION lib/${PROJECT_NAME})

# app/offboard/NoMaD_control
add_executable(NoMaD_control NoMaD_control.cpp)
ament_target_dependencies(NoMaD_control rclcpp px4_msgs geometry_msgs)
target_link_libraries(NoMaD_control mathlib matrix CollisionPrevention)
install(TARGETS NoMaD_control DESTINATION lib/${PROJECT_NAME})

# app/offboard/keyboard_control
add_executable(keyboard_control keyboard_control.cpp)
ament_target_dependencies(keyboard_control rclcpp px4_msgs geometry_msgs)
target_link_libraries(keyboard_control mathlib matrix CollisionPrevention KeyboardTeleop)
install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})
