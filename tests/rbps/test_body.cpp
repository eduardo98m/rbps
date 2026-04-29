#include "rbps/Body.hpp"
#include "tests/test_helper.hpp"
#include "tests/rbps/body_helpers.hpp"

using namespace rbps;
using namespace m3d;

// ─── Tests ───────────────────────────────────────────────────────────────────

TEST(update_inertia_tensor_world_identity)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    update_inertia_tensor_world(bc, 0);
    ASSERT_TRUE(bc.inertia_tensor_world[0].is_approx(bc.inertia_tensor[0], 0.001));
    ASSERT_TRUE(bc.inverse_inertia_tensor_world[0].is_approx(bc.inverse_inertia_tensor[0], 0.001));
}

TEST(update_inertia_tensor_world_rotated)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inertia_tensor[0] = smat3(2, 3, 4, 0, 0, 0);
    bc.inverse_inertia_tensor[0] = smat3(0.5, 1.0 / 3.0, 0.25, 0, 0, 0);
    bc.orientation[0] = quat::from_rpy(0, 0, M_PI / 2.0);
    update_inertia_tensor_world(bc, 0);
    ASSERT_NEAR(bc.inertia_tensor_world[0].xx, 3.0, 0.001);
    ASSERT_NEAR(bc.inertia_tensor_world[0].yy, 2.0, 0.001);
    ASSERT_NEAR(bc.inertia_tensor_world[0].zz, 4.0, 0.001);
}

TEST(update_position_and_orientation_no_forces)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.position[0] = vec3{1, 2, 3};
    bc.linear_velocity[0] = vec3{0.5, 0.3, 0.1};
    scalar dt = 0.01;
    update_position_and_orientation(bc, dt);
    ASSERT_NEAR(bc.position[0].x, 1.0 + 0.5 * dt, 0.001);
    ASSERT_NEAR(bc.position[0].y, 2.0 + 0.3 * dt, 0.001);
    ASSERT_NEAR(bc.position[0].z, 3.0 + 0.1 * dt, 0.001);
    ASSERT_NEAR(bc.prev_position[0].x, 1.0, 0.001);
    ASSERT_NEAR(bc.prev_position[0].y, 2.0, 0.001);
}

TEST(update_position_and_orientation_with_forces)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.force[0] = vec3{10, 0, 0};
    bc.mass[0] = 2.0;
    bc.inverse_mass[0] = 0.5;
    scalar dt = 0.01;
    update_position_and_orientation(bc, dt);
    ASSERT_NEAR(bc.linear_velocity[0].x, 0.05, 0.001);
}

TEST(update_position_and_orientation_angular)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.angular_velocity[0] = vec3{0, 0, 1.0};
    scalar dt = 0.01;
    quat initial_orientation = bc.orientation[0];
    update_position_and_orientation(bc, dt);
    ASSERT_FALSE(bc.orientation[0] == initial_orientation);
    scalar mag = std::sqrt(bc.orientation[0].w * bc.orientation[0].w +
                           bc.orientation[0].x * bc.orientation[0].x +
                           bc.orientation[0].y * bc.orientation[0].y +
                           bc.orientation[0].z * bc.orientation[0].z);
    ASSERT_NEAR(mag, 1.0, 0.001);
}

TEST(update_position_and_orientation_static_body)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    bc.force[0] = vec3{100, 100, 100};
    bc.torque[0] = vec3{10, 10, 10};
    vec3 initial_pos = bc.position[0];
    quat initial_orient = bc.orientation[0];
    update_position_and_orientation(bc, 0.01);
    ASSERT_TRUE(bc.position[0] == initial_pos);
    ASSERT_TRUE(bc.orientation[0] == initial_orient);
}

TEST(update_velocities_from_position_change)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.position[0] = vec3{1, 2, 3};
    scalar inv_dt = 1.0 / 0.1;
    update_velocities(bc, inv_dt);
    ASSERT_NEAR(bc.linear_velocity[0].x, 10.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].y, 20.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].z, 30.0, 0.001);
}

TEST(update_velocities_from_orientation_change)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.prev_orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[0] = quat::from_rpy(0, 0, 0.1);
    update_velocities(bc, 1.0 / 0.1);
    ASSERT_TRUE(std::abs(bc.angular_velocity[0].z) > 0.5);
}

TEST(update_velocities_static_body)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.position[0] = vec3{10, 10, 10};
    update_velocities(bc, 10.0);
    ASSERT_NEAR(bc.linear_velocity[0].x, 0.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].y, 0.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].z, 0.0, 0.001);
}

TEST(apply_positional_constraint_impulse_linear)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.mass[0] = 2.0;
    bc.inverse_mass[0] = 0.5;
    bc.position[0] = vec3{0, 0, 0};
    apply_positional_constraint_impulse(bc, 0, vec3{10, 0, 0}, vec3{0, 0, 0});
    ASSERT_NEAR(bc.position[0].x, 5.0, 0.001);
    ASSERT_NEAR(bc.position[0].y, 0.0, 0.001);
}

TEST(apply_positional_constraint_impulse_with_lever)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inverse_mass[0] = 1.0;
    bc.inverse_inertia_tensor[0] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    quat initial_orientation = bc.orientation[0];
    apply_positional_constraint_impulse(bc, 0, vec3{0, 10, 0}, vec3{1, 0, 0});
    ASSERT_NEAR(bc.position[0].y, 10.0, 0.001);
    ASSERT_FALSE(bc.orientation[0] == initial_orientation);
}

TEST(apply_positional_constraint_impulse_static)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    vec3 initial_pos = bc.position[0];
    quat initial_orient = bc.orientation[0];
    apply_positional_constraint_impulse(bc, 0, vec3{100, 100, 100}, vec3{1, 1, 1});
    ASSERT_TRUE(bc.position[0] == initial_pos);
    ASSERT_TRUE(bc.orientation[0] == initial_orient);
}

TEST(apply_rotational_constraint_impulse_base)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    quat initial_orientation = bc.orientation[0];
    apply_rotational_constraint_impulse(bc, 0, vec3{0, 0, 1});
    ASSERT_FALSE(bc.orientation[0] == initial_orientation);
    scalar mag = std::sqrt(bc.orientation[0].w * bc.orientation[0].w +
                           bc.orientation[0].x * bc.orientation[0].x +
                           bc.orientation[0].y * bc.orientation[0].y +
                           bc.orientation[0].z * bc.orientation[0].z);
    ASSERT_NEAR(mag, 1.0, 0.001);
}

TEST(apply_rotational_constraint_impulse_static)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    quat initial_orient = bc.orientation[0];
    apply_rotational_constraint_impulse(bc, 0, vec3{10, 10, 10});
    ASSERT_TRUE(bc.orientation[0] == initial_orient);
}

TEST(apply_positional_velocity_constraint_impulse_base)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inverse_mass[0] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    apply_positional_velocity_constraint_impulse(bc, 0, vec3{5, 0, 0}, vec3{0, 1, 0});
    ASSERT_NEAR(bc.linear_velocity[0].x, 5.0, 0.001);
    ASSERT_NEAR(bc.angular_velocity[0].z, -5.0, 0.001);
}

TEST(apply_positional_velocity_constraint_impulse_static)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    vec3 initial_lin_vel = bc.linear_velocity[0];
    vec3 initial_ang_vel = bc.angular_velocity[0];
    apply_positional_velocity_constraint_impulse(bc, 0, vec3{100, 0, 0}, vec3{1, 1, 1});
    ASSERT_TRUE(bc.linear_velocity[0] == initial_lin_vel);
    ASSERT_TRUE(bc.angular_velocity[0] == initial_ang_vel);
}

TEST(get_positional_generalized_inverse_mass_center)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inverse_mass[0] = 0.5;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    scalar w = get_positional_generalized_inverse_mass(bc, 0, vec3{0, 0, 0}, vec3{1, 0, 0});
    ASSERT_NEAR(w, 0.5, 0.001);
}

TEST(get_positional_generalized_inverse_mass_offset)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inverse_mass[0] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    scalar w = get_positional_generalized_inverse_mass(bc, 0, vec3{1, 0, 0}, vec3{0, 1, 0});
    ASSERT_NEAR(w, 2.0, 0.001);
}

TEST(get_positional_generalized_inverse_mass_static)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    scalar w = get_positional_generalized_inverse_mass(bc, 0, vec3{1, 1, 1}, vec3{0, 1, 0});
    ASSERT_NEAR(w, 0.0, 0.001);
}

TEST(get_rotational_generalized_inverse_mass_base)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.inverse_inertia_tensor[0] = smat3(2, 3, 4, 0, 0, 0);
    scalar w = get_rotational_generalized_inverse_mass(bc, 0, vec3{1, 0, 0});
    ASSERT_NEAR(w, 2.0, 0.001);
}

TEST(get_rotational_generalized_inverse_mass_static)
{
    BodyCollection bc; test::init_test_bodies(bc, 1);
    bc.type[0] = BodyType::STATIC;
    scalar w = get_rotational_generalized_inverse_mass(bc, 0, vec3{0, 0, 1});
    ASSERT_NEAR(w, 0.0, 0.001);
}

TEST(multiple_bodies_integration)
{
    BodyCollection bc; test::init_test_bodies(bc, 3);
    bc.mass[0] = 1.0;
    bc.inverse_mass[0] = 1.0;
    bc.mass[1] = 2.0;
    bc.inverse_mass[1] = 0.5;
    bc.mass[2] = 3.0;
    bc.inverse_mass[2] = 1.0 / 3.0;
    bc.force[0] = vec3{10, 0, 0};
    bc.force[1] = vec3{0, 20, 0};
    bc.force[2] = vec3{0, 0, 30};
    scalar dt = 0.01;
    update_position_and_orientation(bc, dt);
    ASSERT_NEAR(bc.linear_velocity[0].x, 0.1, 0.001);
    ASSERT_NEAR(bc.linear_velocity[1].y, 0.1, 0.001);
    ASSERT_NEAR(bc.linear_velocity[2].z, 0.1, 0.001);
}

TEST_SUITE(
    RUN_TEST(update_inertia_tensor_world_identity),
    RUN_TEST(update_inertia_tensor_world_rotated),
    RUN_TEST(update_position_and_orientation_no_forces),
    RUN_TEST(update_position_and_orientation_with_forces),
    RUN_TEST(update_position_and_orientation_angular),
    RUN_TEST(update_position_and_orientation_static_body),
    RUN_TEST(update_velocities_from_position_change),
    RUN_TEST(update_velocities_from_orientation_change),
    RUN_TEST(update_velocities_static_body),
    RUN_TEST(apply_positional_constraint_impulse_linear),
    RUN_TEST(apply_positional_constraint_impulse_with_lever),
    RUN_TEST(apply_positional_constraint_impulse_static),
    RUN_TEST(apply_rotational_constraint_impulse_base),
    RUN_TEST(apply_rotational_constraint_impulse_static),
    RUN_TEST(apply_positional_velocity_constraint_impulse_base),
    RUN_TEST(apply_positional_velocity_constraint_impulse_static),
    RUN_TEST(get_positional_generalized_inverse_mass_center),
    RUN_TEST(get_positional_generalized_inverse_mass_offset),
    RUN_TEST(get_positional_generalized_inverse_mass_static),
    RUN_TEST(get_rotational_generalized_inverse_mass_base),
    RUN_TEST(get_rotational_generalized_inverse_mass_static),
    RUN_TEST(multiple_bodies_integration))