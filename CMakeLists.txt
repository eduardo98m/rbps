cmake_minimum_required(VERSION 3.10)
project(rbps)

set(CMAKE_CXX_STANDARD 17)
# Add these flags to your test executable in CMake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

# Include directories
include_directories(include)

# Building the library
set(RBPS_SOURCES
    src/rbps/Body.cpp
    src/rbps/constraints/Constraint.cpp
)

add_library(rbps_lib ${RBPS_SOURCES})
target_include_directories(rbps_lib PUBLIC include)

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


add_executable(test_body tests/rbps/test_body.cpp)
target_link_libraries(test_body PRIVATE rbps_lib)
add_test(NAME test_body COMMAND test_body)

add_executable(test_constraint tests/rbps/test_constraint.cpp)
target_link_libraries(test_constraint PRIVATE rbps_lib)
add_test(NAME test_constraint COMMAND test_constraint)
