cmake_minimum_required(VERSION 3.10)
project(rbps)

set(CMAKE_CXX_STANDARD 17)
# Add these flags to your test executable in CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

# Include directories
include_directories(include)

# Enable Testing
enable_testing()

# Math Tests
add_executable(test_math tests/math3d/test_math.cpp)
add_test(NAME test_math COMMAND test_math)

add_executable(test_mat3 tests/math3d/test_mat3.cpp)
add_test(NAME test_mat3 COMMAND test_mat3)

add_executable(test_quat tests/math3d/test_quat.cpp)
add_test(NAME test_quat COMMAND test_quat)

add_executable(test_vec3 tests/math3d/test_vec3.cpp)
add_test(NAME test_vec3 COMMAND test_vec3)

