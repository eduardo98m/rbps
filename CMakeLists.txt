cmake_minimum_required(VERSION 3.14)
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
    src/rbps/CollisionPipeline.cpp
    src/rbps/constraints/Constraint.cpp
    src/rbps/constraints/Joint.cpp
    src/rbps/constraints/Contact.cpp
)

set(RBC_SOURCES
    src/rbc/BroadPhase.cpp
    src/rbc/narrow_phase.cpp
    src/rbc/gjk/GJK.cpp
    src/rbc/gjk/EPA.cpp
)

add_library(rbc_lib ${RBC_SOURCES})
target_include_directories(rbc_lib PUBLIC include)


add_library(rbps_lib ${RBPS_SOURCES})
target_include_directories(rbps_lib PUBLIC include)

target_link_libraries(rbps_lib PUBLIC rbc_lib)

#----------------------------------------------------------

# ============================================================================
#  Visualizer optional block — append to your existing CMakeLists.txt
#
#  Usage:
#    cmake -DRBPS_BUILD_VISR=ON ..   → builds the visr target + demo app
#    cmake ..                         → visr not built, zero overhead
#
#  Dependencies (managed here, isolated from your test build):
#    raylib   5.0        — prefer system install, FetchContent fallback
#    imgui    v1.91.8    — always FetchContent (no system package standard)
#    rlImGui  pinned SHA — FetchContent (small bridging lib)
#
#  Why FetchContent over submodules?
#    • Pinned to an exact git tag or commit SHA — fully reproducible.
#    • No .gitmodules to maintain; devs don't need 'git submodule update'.
#    • CMake handles the dependency graph; targets are available immediately.
#    • To update a dep: change the GIT_TAG line and re-run cmake.
# ============================================================================


cmake_minimum_required(VERSION 3.14)
 
option(RBPS_BUILD_VISR "Build the rbps visualizer (requires Raylib + ImGui)" OFF)
 
if(NOT RBPS_BUILD_VISR)
    return()
endif()
 
include(FetchContent)
 
# ── 1. Raylib ─────────────────────────────────────────────────────────────────
find_package(raylib 5.0 QUIET)
 
if(NOT raylib_FOUND)
    message(STATUS "[visr] raylib not found, fetching 5.5...")
    fetchcontent_declare(raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        5.5
        GIT_SHALLOW    TRUE
    )
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
    fetchcontent_makeavailable(raylib)
else()
    message(STATUS "[visr] using system raylib ${raylib_VERSION}")
endif()
 
# ── 2. Dear ImGui ─────────────────────────────────────────────────────────────
#  Must be >= 1.92.1 — rlImGui uses the new Font API introduced there.
#  Pin to v1.92.6 (latest as of 2025-03).
fetchcontent_declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.6
    GIT_SHALLOW    TRUE
)
fetchcontent_makeavailable(imgui)
 
add_library(imgui_rbps STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)
target_include_directories(imgui_rbps PUBLIC ${imgui_SOURCE_DIR})
 
# ── 3. rlImGui ────────────────────────────────────────────────────────────────
#  rlImGui has NO release tags — we pin to a commit SHA.
#  4d8a618 = latest main as of 2025-03, compatible with ImGui >= 1.92.1.
#  To update: grab a new SHA from https://github.com/raylib-extras/rlImGui/commits/main
fetchcontent_declare(rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        4d8a618
    GIT_SHALLOW    FALSE   # shallow clone cannot be used with a commit SHA
)
fetchcontent_makeavailable(rlimgui)
 
add_library(rlimgui_rbps STATIC
    ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)
target_include_directories(rlimgui_rbps PUBLIC ${rlimgui_SOURCE_DIR})
target_link_libraries(rlimgui_rbps PUBLIC raylib imgui_rbps)
 
# ── 4. visr (header-only interface target) ────────────────────────────────────
add_library(visr_lib INTERFACE)
target_include_directories(visr_lib INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(visr_lib INTERFACE
    rbps_lib
    raylib
    imgui_rbps
    rlimgui_rbps
)

# ── 5. Demo application ───────────────────────────────────────────────────────
#Replace src/visr_demo/main.cpp with your own scene setup.
add_executable(visr_demo src/visr_demo/main.cpp)
target_link_libraries(visr_demo PRIVATE visr_lib)

message(STATUS "[visr] visualizer targets built: visr_lib, visr_demo")

#----------------------------------------------------------

# Enable Testing
enable_testing()

# IVC Tests
add_executable(test_ivc tests/ivc/test_ivc.cpp)
add_test(NAME test_ivc COMMAND test_ivc)

# Storage Tests
add_executable(test_soa tests/storage/test_soa.cpp)
add_test(NAME test_soa COMMAND test_soa)

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

add_executable(test_sphere_capsule tests/rbc/analytic/test_sphere_capsule.cpp)
target_link_libraries(test_sphere_capsule PRIVATE rbc_lib)
add_test(NAME test_sphere_capsule COMMAND test_sphere_capsule)

add_executable(test_capsule_capsule tests/rbc/analytic/test_capsule_capsule.cpp)
target_link_libraries(test_capsule_capsule PRIVATE rbc_lib)
add_test(NAME test_capsule_capsule COMMAND test_capsule_capsule)

add_executable(test_plane_sphere tests/rbc/analytic/test_plane_sphere.cpp)
target_link_libraries(test_plane_sphere PRIVATE rbc_lib)
add_test(NAME test_plane_sphere COMMAND test_plane_sphere)

add_executable(test_plane_box tests/rbc/analytic/test_plane_box.cpp)
target_link_libraries(test_plane_box PRIVATE rbc_lib)
add_test(NAME test_plane_box COMMAND test_plane_box)

add_executable(test_plane_capsule tests/rbc/analytic/test_plane_capsule.cpp)
target_link_libraries(test_plane_capsule PRIVATE rbc_lib)
add_test(NAME test_plane_capsule COMMAND test_plane_capsule)

add_executable(test_heightmap_sphere tests/rbc/analytic/test_heightmap_sphere.cpp)
target_link_libraries(test_heightmap_sphere PRIVATE rbc_lib)
add_test(NAME test_heightmap_sphere COMMAND test_heightmap_sphere)

add_executable(test_heightmap_capsule tests/rbc/analytic/test_heightmap_capsule.cpp)
target_link_libraries(test_heightmap_capsule PRIVATE rbc_lib)
add_test(NAME test_heightmap_capsule COMMAND test_heightmap_capsule)


add_executable(test_dispatcher tests/rbc/test_dispatcher.cpp)
target_link_libraries(test_dispatcher PRIVATE rbc_lib)
add_test(NAME test_dispatcher COMMAND test_dispatcher)

add_executable(test_broad_phase tests/rbc/test_broad_phase.cpp)
target_link_libraries(test_broad_phase PRIVATE rbc_lib)
add_test(NAME test_broad_phase COMMAND test_broad_phase)


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

add_executable(test_collision_pipeline tests/rbps/test_collision_pipeline.cpp)
target_link_libraries(test_collision_pipeline PRIVATE rbps_lib)
add_test(NAME test_collision_pipeline COMMAND test_collision_pipeline)

add_executable(test_collision_pipeline_graph_coloring tests/rbps/test_collision_pipeline_graph_coloring.cpp)
target_link_libraries(test_collision_pipeline_graph_coloring PRIVATE rbps_lib)
add_test(NAME test_collision_pipeline_graph_coloring COMMAND test_collision_pipeline_graph_coloring)

# API
add_executable(test_bodyAPI tests/rbps/API/test_bodyAPI.cpp)
target_link_libraries(test_bodyAPI PRIVATE rbps_lib)
add_test(NAME test_bodyAPI COMMAND test_bodyAPI)

add_executable(test_constraintAPI tests/rbps/API/test_constraintAPI.cpp)
target_link_libraries(test_constraintAPI PRIVATE rbps_lib)
add_test(NAME test_constraintAPI COMMAND test_constraintAPI)

add_executable(test_jointAPI tests/rbps/API/test_jointAPI.cpp)
target_link_libraries(test_jointAPI PRIVATE rbps_lib)
add_test(NAME test_jointAPI COMMAND test_jointAPI)

add_executable(test_colliderAPI tests/rbps/API/test_colliderAPI.cpp)
target_link_libraries(test_colliderAPI PRIVATE rbps_lib)
add_test(NAME test_colliderAPI COMMAND test_colliderAPI)

add_executable(test_world tests/rbps/API/test_world.cpp)
target_link_libraries(test_world PRIVATE rbps_lib)
add_test(NAME test_world COMMAND test_world)
