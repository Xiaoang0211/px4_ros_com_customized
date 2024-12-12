cmake_minimum_required(VERSION 3.8)

add_library(mathlib INTERFACE)

# Specify include directories for mathlib
target_include_directories(mathlib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/lib/mathlib>
    $<INSTALL_INTERFACE:include/lib/mathlib>
)

# Link to Eigen3
find_package(Eigen3 REQUIRED)
target_link_libraries(mathlib INTERFACE Eigen3::Eigen)

# Install headers for mathlib
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/lib/mathlib/
    DESTINATION include/lib/mathlib
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Export the mathlib target for downstream usage
ament_export_targets(export_mathlib HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    # Enable testing
    enable_testing()
    # Find GTest
    find_package(GTest REQUIRED)

    # Define function for adding mathlib tests
    function(add_mathlib_test test_name source_file)
        add_executable(${test_name} ${source_file})
        target_link_libraries(${test_name} PRIVATE mathlib GTest::GTest GTest::Main)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endfunction()

    # Add each test using the add_mathlib_test function
    add_mathlib_test(LowPassFilter2pVector3fTest math/test/LowPassFilter2pVector3fTest.cpp)
    add_mathlib_test(AlphaFilterTest math/test/AlphaFilterTest.cpp)
    add_mathlib_test(MedianFilterTest math/test/MedianFilterTest.cpp)
    add_mathlib_test(NotchFilterTest math/test/NotchFilterTest.cpp)
    add_mathlib_test(second_order_reference_model_test math/test/second_order_reference_model_test.cpp)
    add_mathlib_test(FunctionsTest math/FunctionsTest.cpp)
    add_mathlib_test(UtilitiesTest math/test/UtilitiesTest.cpp)
    add_mathlib_test(WelfordMeanTest math/WelfordMeanTest.cpp)
    add_mathlib_test(WelfordMeanVectorTest math/WelfordMeanVectorTest.cpp)
    add_mathlib_test(MaxDistanceToCircleTest math/MaxDistanceToCircleTest.cpp)
endif()
