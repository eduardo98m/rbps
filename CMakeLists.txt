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
    src/rbps/constraints/Joint.cpp
    src/rbps/constraints/Contact.cpp
)

set(RBC_SOURCES
    src/rbc/narrow_phase.cpp
    src/rbc/gjk/GJK.cpp
    src/rbc/gjk/EPA.cpp
)


add_library(rbps_lib ${RBPS_SOURCES})
target_include_directories(rbps_lib PUBLIC include)

add_library(rbc_lib ${RBC_SOURCES}) 
target_include_directories(rbc_lib PUBLIC include)

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

add_executable(test_vec2i tests/math3d/test_vec2i.cpp)
add_test(NAME test_vec2i COMMAND test_vec2i)

# Collision Tests
add_executable(test_gjk tests/rbc/test_gjk.cpp)
target_link_libraries(test_gjk PRIVATE rbc_lib)
add_test(NAME test_gjk COMMAND test_gjk)

add_executable(test_epa tests/rbc/test_epa.cpp)
target_link_libraries(test_epa PRIVATE rbc_lib)
add_test(NAME test_epa COMMAND test_epa)

add_executable(test_sphere_sphere tests/rbc/analytic/test_sphere_sphere.cpp)
target_link_libraries(test_sphere_sphere PRIVATE rbc_lib)
add_test(NAME test_sphere_sphere COMMAND test_sphere_sphere)

add_executable(test_box_box tests/rbc/analytic/test_box_box.cpp)
target_link_libraries(test_box_box PRIVATE rbc_lib)
add_test(NAME test_box_box COMMAND test_box_box)

add_executable(test_sphere_box tests/rbc/analytic/test_sphere_box.cpp)
target_link_libraries(test_sphere_box PRIVATE rbc_lib)
add_test(NAME test_sphere_box COMMAND test_sphere_box)

add_executable(test_dispatcher tests/rbc/test_dispatcher.cpp)
target_link_libraries(test_dispatcher PRIVATE rbc_lib)
add_test(NAME test_dispatcher COMMAND test_dispatcher)

# Physics Tests

add_executable(test_body tests/rbps/test_body.cpp)
target_link_libraries(test_body PRIVATE rbps_lib)
add_test(NAME test_body COMMAND test_body)

add_executable(test_constraint tests/rbps/test_constraint.cpp)
target_link_libraries(test_constraint PRIVATE rbps_lib)
add_test(NAME test_constraint COMMAND test_constraint)

add_executable(test_joint tests/rbps/test_joint.cpp)
target_link_libraries(test_joint PRIVATE rbps_lib)
add_test(NAME test_joint COMMAND test_joint)

add_executable(test_contact tests/rbps/test_contact.cpp)
target_link_libraries(test_contact PRIVATE rbps_lib)
add_test(NAME test_contact COMMAND test_contact)
