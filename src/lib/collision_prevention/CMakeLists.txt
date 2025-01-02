cmake_minimum_required(VERSION 3.8)

# Define the CollisionPrevention library
add_library(CollisionPrevention STATIC CollisionPrevention.cpp)

# Specify include directories for the library
target_include_directories(CollisionPrevention PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(CollisionPrevention px4_msgs rclcpp)
target_link_libraries(CollisionPrevention mathlib matrix)

# Add compile options (adjust as needed)
target_compile_options(CollisionPrevention PRIVATE -Wno-cast-align) # TODO: fix and enable

# Install the CollisionPrevention library
install(TARGETS CollisionPrevention
    EXPORT export_CollisionPrevention
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/lib/collision_prevention
)

# Install header files for this library
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/collision_prevention/
    DESTINATION include/lib/collision_prevention
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Export the library for downstream usage
ament_export_targets(export_CollisionPrevention HAS_LIBRARY_TARGET)

# # Add testing
# if(BUILD_TESTING)
#     # Find Google Test
#     find_package(GTest REQUIRED)
#     enable_testing()

#     # Define test executable
#     set(TEST_NAME collision_prevention_test)
#     add_executable(${TEST_NAME} CollisionPreventionTest.cpp)

#     # Include directories for the test executable
#     target_include_directories(${TEST_NAME} PRIVATE
#         ${CMAKE_SOURCE_DIR}/include/lib/collision_prevention
#         ${CMAKE_CURRENT_SOURCE_DIR}
#     )

#     # Link the CollisionPrevention library and GTest to the test executable
#     target_link_libraries(${TEST_NAME} PRIVATE CollisionPrevention GTest::GTest GTest::Main)

#     # Add the test
#     add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
# endif()