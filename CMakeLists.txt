cmake_minimum_required(VERSION 3.9)
project(px4_ros_com)

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

# find dependencies
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

# Add include directories
# include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/include)

# include subdirectories for the library files
add_subdirectory(src/lib/collision_prevention)
add_subdirectory(src/lib/mathlib)
add_subdirectory(src/lib/matrix)
add_subdirectory(src/lib/random_explore)

#################
# Setup targets #
#################

# examples/advertisers/drone_advertiser
add_executable(drone_advertiser src/examples/advertisers/drone_advertiser.cpp)
ament_target_dependencies(drone_advertiser rclcpp px4_msgs OpenCV cv_bridge)
# Link additional libraries if required
target_link_libraries(drone_advertiser gz-transport${GZ_TRANSPORT_VER}::core)
install(TARGETS drone_advertiser DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
target_link_libraries(offboard_control mathlib matrix)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
add_executable(OffboardControl src/examples/offboard/OffboardControl.cpp)
ament_target_dependencies(OffboardControl rclcpp px4_msgs)
target_link_libraries(OffboardControl mathlib matrix CollisionPrevention RandomExplore)
install(TARGETS OffboardControl DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_include_directories(include)

ament_export_targets(export_CollisionPrevention HAS_LIBRARY_TARGET)
ament_export_targets(export_mathlib HAS_LIBRARY_TARGET)
ament_export_targets(export_matrix HAS_LIBRARY_TARGET)
ament_export_targets(export_RandomExplore HAS_LIBRARY_TARGET)

# Install header files
install(DIRECTORY include/
		DESTINATION include/)

# Install CollisionPrevention library
install(TARGETS CollisionPrevention
	EXPORT export_CollisionPrevention
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install RandomExplore library
install(TARGETS RandomExplore
	EXPORT export_RandomExplore
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install header files for CollisionPrevention
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/collision_prevention/
    DESTINATION include/lib/collision_prevention
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install header files for RandomExplore
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/random_explore/
    DESTINATION include/lib/random_explore
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install header files for OffboardControl
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/examples/offboard/
    DESTINATION include/examples/offboard
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install headers for mathlib
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/mathlib/
    DESTINATION include/lib/mathlib
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install headers for matrix
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/matrix/
    DESTINATION include/lib/matrix
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(TARGETS mathlib EXPORT export_mathlib)
install(TARGETS matrix EXPORT export_matrix)

# ############
# # Testing ##
# ############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
