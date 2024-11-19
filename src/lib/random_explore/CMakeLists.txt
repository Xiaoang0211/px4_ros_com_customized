cmake_minimum_required(VERSION 3.8)

# Define the RandomExplore library
add_library(RandomExplore STATIC RandomExplore.cpp)
# Specify include directories for the library
target_include_directories(RandomExplore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(RandomExplore px4_msgs)
target_link_libraries(RandomExplore mathlib matrix)

# Add compile options (adjust as needed)
target_compile_options(RandomExplore PRIVATE -Wno-cast-align) # TODO: fix and enable