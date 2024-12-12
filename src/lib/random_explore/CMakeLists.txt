cmake_minimum_required(VERSION 3.8)

# Define the RandomExplore library
add_library(RandomExplore STATIC RandomExplore.cpp)
# Specify include directories for the library
target_include_directories(RandomExplore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(RandomExplore px4_msgs rclcpp)
target_link_libraries(RandomExplore mathlib matrix CollisionPrevention)

# Add compile options (adjust as needed)
target_compile_options(RandomExplore PRIVATE -Wno-cast-align) # TODO: fix and enable

# Export the library for downstream usage
install(TARGETS RandomExplore
    EXPORT export_RandomExplore
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/lib/random_explore
)

# Install header files for this library
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/random_explore/
    DESTINATION include/lib/random_explore
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Export library for downstream use
ament_export_targets(export_RandomExplore HAS_LIBRARY_TARGET)

# Add testing in the future
# if(BUILD_TESTING)
#   # Add unit tests here if needed
# endif()