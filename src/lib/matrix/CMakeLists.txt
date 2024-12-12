cmake_minimum_required(VERSION 3.8)

project(matrix)

# Find dependencies
find_package(Eigen3 REQUIRED)

# Define matrix as an interface library since it only contains header files
add_library(matrix INTERFACE)

# Specify include directories for the matrix library
target_include_directories(matrix INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/lib/matrix>
    $<INSTALL_INTERFACE:include/lib/matrix>
)

# Link dependencies needed by the headers
target_link_libraries(matrix INTERFACE Eigen3::Eigen)

# Install headers for matrix
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/matrix/
    DESTINATION include/lib/matrix
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export the matrix target so that other packages can use it
ament_export_targets(export_matrix HAS_LIBRARY_TARGET)

# # If testing is enabled, process the test directory
# if(BUILD_TESTING)

# endif()
