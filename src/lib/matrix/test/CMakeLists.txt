# Minimum CMake version required
cmake_minimum_required(VERSION 3.8)

add_compile_options(
    -Wno-double-promotion
    -Wno-float-equal
)

function(add_matrix_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} PRIVATE matrix GTest::GTest GTest::Main)
    target_include_directories(${test_name} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

# Add each test using the add_matrix_test function
add_matrix_test(MatrixAssignmentTest MatrixAssignmentTest.cpp)
add_matrix_test(MatrixAttitudeTest MatrixAttitudeTest.cpp)
add_matrix_test(MatrixCopyToTest MatrixCopyToTest.cpp)
add_matrix_test(MatrixDcm2Test MatrixDcm2Test.cpp)
add_matrix_test(MatrixDualTest MatrixDualTest.cpp)
add_matrix_test(MatrixFilterTest MatrixFilterTest.cpp)
add_matrix_test(MatrixHatveeTest MatrixHatveeTest.cpp)
add_matrix_test(MatrixHelperTest MatrixHelperTest.cpp)
add_matrix_test(MatrixIntegralTest MatrixIntegralTest.cpp)
add_matrix_test(MatrixInverseTest MatrixInverseTest.cpp)
add_matrix_test(MatrixLeastSquaresTest MatrixLeastSquaresTest.cpp)
add_matrix_test(MatrixMultiplicationTest MatrixMultiplicationTest.cpp)
add_matrix_test(MatrixPseudoInverseTest MatrixPseudoInverseTest.cpp)
add_matrix_test(MatrixScalarMultiplicationTest MatrixScalarMultiplicationTest.cpp)
add_matrix_test(MatrixSetIdentityTest MatrixSetIdentityTest.cpp)
add_matrix_test(MatrixSliceTest MatrixSliceTest.cpp)
add_matrix_test(MatrixSparseVectorTest MatrixSparseVectorTest.cpp)
add_matrix_test(MatrixSquareTest MatrixSquareTest.cpp)
add_matrix_test(MatrixTransposeTest MatrixTransposeTest.cpp)
add_matrix_test(MatrixVectorTest MatrixVectorTest.cpp)
add_matrix_test(MatrixUnwrapTest MatrixUnwrapTest.cpp)
add_matrix_test(MatrixUpperRightTriangleTest MatrixUpperRightTriangleTest.cpp)
add_matrix_test(MatrixVector2Test MatrixVector2Test.cpp)
add_matrix_test(MatrixVector3Test MatrixVector3Test.cpp)
add_matrix_test(MatrixVectorAssignmentTest MatrixVectorAssignmentTest.cpp)