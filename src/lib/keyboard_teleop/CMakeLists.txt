cmake_minimum_required(VERSION 3.8)

# Define the RandomExplore library
add_library(KeyboardTeleop STATIC KeyboardTeleop.cpp)

# Specify include directories for the library
target_include_directories(KeyboardTeleop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(KeyboardTeleop geometry_msgs rclcpp)

# Add compile options (adjust as needed)
target_compile_options(KeyboardTeleop PRIVATE -Wno-cast-align) # TODO: fix and enable

# Export the library for downstream usage
install(TARGETS KeyboardTeleop
    EXPORT export_KeyboardTeleop
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/lib/keyboard_teleop
)

# Install header files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/keyboard_teleop/
    DESTINATION include/lib/keyboard_teleop
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Export library for downstream use
ament_export_targets(export_KeyboardTeleop HAS_LIBRARY_TARGET)

# Add testing in the future
# if(BUILD_TESTING)
#   # Add unit tests here if needed
# endif()