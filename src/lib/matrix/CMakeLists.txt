cmake_minimum_required(VERSION 3.8)

# Define matrix as an interface library since it only contains header files
add_library(matrix INTERFACE)

# Specify include directories for the matrix library
target_include_directories(matrix INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link any dependencies needed by the headers
target_link_libraries(matrix INTERFACE Eigen3::Eigen)

# Include the test subdirectory if testing is enabled
add_subdirectory(test)

