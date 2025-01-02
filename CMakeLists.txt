cmake_minimum_required(VERSION 3.9)
project(px4_ros_com)

set(CMAKE_BUILD_TYPE Debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable testing and find Google Test
include(CTest)
enable_testing()
find_package(GTest REQUIRED)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(gz-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

# Top-level includes
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add subdirectories for libraries
add_subdirectory(src/lib/collision_prevention)
add_subdirectory(src/lib/mathlib)
add_subdirectory(src/lib/matrix)
add_subdirectory(src/lib/random_explore)
add_subdirectory(src/lib/keyboard_teleop)

# Add subdirectories for app
add_subdirectory(src/app/advertisers)
add_subdirectory(src/app/listeners)
add_subdirectory(src/app/offboard)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(
    ament_cmake
    rclcpp
    rosidl_default_runtime
    eigen3_cmake_module
    Eigen3
    px4_msgs
    geometry_msgs
    sensor_msgs
)

# Install the main include directory (this includes general headers if any)
install(DIRECTORY include/
    DESTINATION include/
)

# Install the launch files
install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/launch
    DESTINATION share/px4_ros_com/
)

# ############
# # Testing ##
# ############

# if(BUILD_TESTING)
#     find_package(ament_lint_auto REQUIRED)
#     # These lines disable certain lint checks. Remove them once licenses and copyrights
#     # are in place.
#     set(ament_cmake_copyright_FOUND TRUE)
#     set(ament_cmake_cpplint_FOUND TRUE)
#     ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
