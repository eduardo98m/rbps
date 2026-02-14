#include "rbps/Body.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;
using namespace m3d;

// Helper function to create a simple body collection
BodyCollection create_test_bodies(size_t n)
{
    BodyCollection bc;
    bc.n_bodies = n;
    bc.force.resize(n, vec3{0, 0, 0});
    bc.torque.resize(n, vec3{0, 0, 0});
    bc.mass.resize(n, 1.0);
    bc.inverse_mass.resize(n, 1.0);
    bc.inertia_tensor.resize(n, smat3(1, 1, 1, 0, 0, 0)); // Identity inertia
    bc.inverse_inertia_tensor.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.inertia_tensor_world.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.inverse_inertia_tensor_world.resize(n, smat3(1, 1, 1, 0, 0, 0));
    bc.type.resize(n, BodyType::DYNAMIC);
    bc.position.resize(n, vec3{0, 0, 0});
    bc.orientation.resize(n, quat(1, 0, 0, 0));
    bc.linear_velocity.resize(n, vec3{0, 0, 0});
    bc.angular_velocity.resize(n, vec3{0, 0, 0});
    bc.prev_position.resize(n, vec3{0, 0, 0});
    bc.prev_orientation.resize(n, quat(1, 0, 0, 0));
    bc.prev_linear_velocity.resize(n, vec3{0, 0, 0});
    bc.prev_angular_velocity.resize(n, vec3{0, 0, 0});
    return bc;
}

TEST(update_inertia_tensor_world_identity)
{
    BodyCollection bc = create_test_bodies(1);
    
    // With identity orientation, world inertia should equal local inertia
    update_inertia_tensor_world(bc, 0);
    
    ASSERT_TRUE(bc.inertia_tensor_world[0].is_approx(bc.inertia_tensor[0], 0.001));
    ASSERT_TRUE(bc.inverse_inertia_tensor_world[0].is_approx(bc.inverse_inertia_tensor[0], 0.001));
}

TEST(update_inertia_tensor_world_rotated)
{
    BodyCollection bc = create_test_bodies(1);
    
    // Set a non-identity inertia tensor
    bc.inertia_tensor[0] = smat3(2, 3, 4, 0, 0, 0);
    bc.inverse_inertia_tensor[0] = smat3(0.5, 1.0/3.0, 0.25, 0, 0, 0);
    
    // Rotate 90 degrees around Z axis
    bc.orientation[0] = quat::from_rpy(0, 0, M_PI / 2.0);
    
    update_inertia_tensor_world(bc, 0);
    
    // World inertia should be rotated
    // After 90° Z rotation: Ixx and Iyy should swap
    ASSERT_NEAR(bc.inertia_tensor_world[0].xx, 3.0, 0.001);
    ASSERT_NEAR(bc.inertia_tensor_world[0].yy, 2.0, 0.001);
    ASSERT_NEAR(bc.inertia_tensor_world[0].zz, 4.0, 0.001);
}

TEST(update_position_and_orientation_no_forces)
{
    BodyCollection bc = create_test_bodies(1);
    bc.position[0] = vec3{1, 2, 3};
    bc.linear_velocity[0] = vec3{0.5, 0.3, 0.1};
    
    scalar dt = 0.01;
    update_position_and_orientation(bc, dt);
    
    // Position should integrate velocity
    ASSERT_NEAR(bc.position[0].x, 1.0 + 0.5 * dt, 0.001);
    ASSERT_NEAR(bc.position[0].y, 2.0 + 0.3 * dt, 0.001);
    ASSERT_NEAR(bc.position[0].z, 3.0 + 0.1 * dt, 0.001);
    
    // Previous position should be stored
    ASSERT_NEAR(bc.prev_position[0].x, 1.0, 0.001);
    ASSERT_NEAR(bc.prev_position[0].y, 2.0, 0.001);
}

TEST(update_position_and_orientation_with_forces)
{
    BodyCollection bc = create_test_bodies(1);
    bc.force[0] = vec3{10, 0, 0};
    bc.mass[0] = 2.0;
    bc.inverse_mass[0] = 0.5;
    
    scalar dt = 0.01;
    update_position_and_orientation(bc, dt);
    
    // Velocity should change: dv = F/m * dt = 10/2 * 0.01 = 0.05
    ASSERT_NEAR(bc.linear_velocity[0].x, 0.05, 0.001);
}

TEST(update_position_and_orientation_angular)
{
    BodyCollection bc = create_test_bodies(1);
    bc.angular_velocity[0] = vec3{0, 0, 1.0}; // Rotate around Z
    
    scalar dt = 0.01;
    quat initial_orientation = bc.orientation[0];
    
    update_position_and_orientation(bc, dt);
    
    // Orientation should have changed
    ASSERT_FALSE(bc.orientation[0] == initial_orientation);
    
    // Quaternion should still be normalized
    scalar mag = std::sqrt(bc.orientation[0].w * bc.orientation[0].w +
                           bc.orientation[0].x * bc.orientation[0].x +
                           bc.orientation[0].y * bc.orientation[0].y +
                           bc.orientation[0].z * bc.orientation[0].z);
    ASSERT_NEAR(mag, 1.0, 0.001);
}

TEST(update_position_and_orientation_static_body)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    bc.force[0] = vec3{100, 100, 100};
    bc.torque[0] = vec3{10, 10, 10};
    
    vec3 initial_pos = bc.position[0];
    quat initial_orient = bc.orientation[0];
    
    update_position_and_orientation(bc, 0.01);
    
    // Static bodies should not move
    ASSERT_TRUE(bc.position[0] == initial_pos);
    ASSERT_TRUE(bc.orientation[0] == initial_orient);
}

TEST(update_velocities_from_position_change)
{
    BodyCollection bc = create_test_bodies(1);
    
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.position[0] = vec3{1, 2, 3};
    
    scalar dt = 0.1;
    scalar inv_dt = 1.0 / dt;
    
    update_velocities(bc, inv_dt);
    
    // Velocity = (pos - prev_pos) / dt
    ASSERT_NEAR(bc.linear_velocity[0].x, 10.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].y, 20.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].z, 30.0, 0.001);
}

TEST(update_velocities_from_orientation_change)
{
    BodyCollection bc = create_test_bodies(1);
    
    bc.prev_orientation[0] = quat(1, 0, 0, 0);
    // Small rotation around Z
    bc.orientation[0] = quat::from_rpy(0, 0, 0.1);
    
    scalar dt = 0.1;
    scalar inv_dt = 1.0 / dt;
    
    update_velocities(bc, inv_dt);
    
    // Angular velocity should be primarily in Z direction
    ASSERT_TRUE(std::abs(bc.angular_velocity[0].z) > 0.5);
}

TEST(update_velocities_static_body)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.position[0] = vec3{10, 10, 10};
    
    update_velocities(bc, 10.0);
    
    // Static bodies velocities should remain zero
    ASSERT_NEAR(bc.linear_velocity[0].x, 0.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].y, 0.0, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].z, 0.0, 0.001);
}

TEST(apply_positional_constraint_impulse_linear)
{
    BodyCollection bc = create_test_bodies(1);
    bc.mass[0] = 2.0;
    bc.inverse_mass[0] = 0.5;
    bc.position[0] = vec3{0, 0, 0};
    
    vec3 impulse{10, 0, 0};
    vec3 r{0, 0, 0}; // Applied at center of mass
    
    apply_positional_constraint_impulse(bc, 0, impulse, r);
    
    // Position change = impulse * inverse_mass
    ASSERT_NEAR(bc.position[0].x, 5.0, 0.001);
    ASSERT_NEAR(bc.position[0].y, 0.0, 0.001);
}

TEST(apply_positional_constraint_impulse_with_lever)
{
    BodyCollection bc = create_test_bodies(1);
    bc.inverse_mass[0] = 1.0;
    bc.inverse_inertia_tensor[0] = smat3(1, 1, 1, 0, 0, 0);
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    
    vec3 impulse{0, 10, 0};
    vec3 r{1, 0, 0}; // Lever arm in X direction
    
    quat initial_orientation = bc.orientation[0];
    
    apply_positional_constraint_impulse(bc, 0, impulse, r);
    
    // Should cause both translation and rotation
    ASSERT_NEAR(bc.position[0].y, 10.0, 0.001);
    ASSERT_FALSE(bc.orientation[0] == initial_orientation);
}

TEST(apply_positional_constraint_impulse_static)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    
    vec3 initial_pos = bc.position[0];
    quat initial_orient = bc.orientation[0];
    
    apply_positional_constraint_impulse(bc, 0, vec3{100, 100, 100}, vec3{1, 1, 1});
    
    // Static body should not move
    ASSERT_TRUE(bc.position[0] == initial_pos);
    ASSERT_TRUE(bc.orientation[0] == initial_orient);
}

TEST(apply_rotational_constraint_impulse_base)
{
    BodyCollection bc = create_test_bodies(1);
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    
    vec3 impulse{0, 0, 1}; // Rotational impulse around Z
    quat initial_orientation = bc.orientation[0];
    
    apply_rotational_constraint_impulse(bc, 0, impulse);
    
    // Orientation should change
    ASSERT_FALSE(bc.orientation[0] == initial_orientation);
    
    // Should still be normalized
    scalar mag = std::sqrt(bc.orientation[0].w * bc.orientation[0].w +
                           bc.orientation[0].x * bc.orientation[0].x +
                           bc.orientation[0].y * bc.orientation[0].y +
                           bc.orientation[0].z * bc.orientation[0].z);
    ASSERT_NEAR(mag, 1.0, 0.001);
}

TEST(apply_rotational_constraint_impulse_static)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    
    quat initial_orient = bc.orientation[0];
    
    apply_rotational_constraint_impulse(bc, 0, vec3{10, 10, 10});
    
    ASSERT_TRUE(bc.orientation[0] == initial_orient);
}

TEST(apply_positional_velocity_constraint_impulse_base)
{
    BodyCollection bc = create_test_bodies(1);
    bc.inverse_mass[0] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    
    vec3 impulse{5, 0, 0};
    vec3 r{0, 1, 0};
    
    apply_positional_velocity_constraint_impulse(bc, 0, impulse, r);
    
    // Linear velocity should change
    ASSERT_NEAR(bc.linear_velocity[0].x, 5.0, 0.001);
    
    // Angular velocity should change (r x impulse)
    // r x impulse = (0,1,0) x (5,0,0) = (0,0,-5)
    ASSERT_NEAR(bc.angular_velocity[0].z, -5.0, 0.001);
}

TEST(apply_positional_velocity_constraint_impulse_static)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    
    vec3 initial_lin_vel = bc.linear_velocity[0];
    vec3 initial_ang_vel = bc.angular_velocity[0];
    
    apply_positional_velocity_constraint_impulse(bc, 0, vec3{100, 0, 0}, vec3{1, 1, 1});
    
    ASSERT_TRUE(bc.linear_velocity[0] == initial_lin_vel);
    ASSERT_TRUE(bc.angular_velocity[0] == initial_ang_vel);
}

TEST(get_positional_generalized_inverse_mass_center)
{
    BodyCollection bc = create_test_bodies(1);
    bc.inverse_mass[0] = 0.5;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    
    vec3 r{0, 0, 0}; // At center of mass
    vec3 n{1, 0, 0}; // Normal direction
    
    scalar w = get_positional_generalized_inverse_mass(bc, 0, r, n);
    
    // Should just be inverse mass (no rotational component)
    ASSERT_NEAR(w, 0.5, 0.001);
}

TEST(get_positional_generalized_inverse_mass_offset)
{
    BodyCollection bc = create_test_bodies(1);
    bc.inverse_mass[0] = 1.0;
    bc.inverse_inertia_tensor_world[0] = smat3(1, 1, 1, 0, 0, 0);
    
    vec3 r{1, 0, 0}; // Offset from center
    vec3 n{0, 1, 0}; // Perpendicular to lever
    
    scalar w = get_positional_generalized_inverse_mass(bc, 0, r, n);
    
    // w = 1/m + (r × n)^T * I^-1 * (r × n)
    // r × n = (1,0,0) × (0,1,0) = (0,0,1)
    // (0,0,1)^T * I * (0,0,1) = 1 (since I_zz = 1)
    // w = 1 + 1 = 2
    ASSERT_NEAR(w, 2.0, 0.001);
}

TEST(get_positional_generalized_inverse_mass_static)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    
    scalar w = get_positional_generalized_inverse_mass(bc, 0, vec3{1, 1, 1}, vec3{0, 1, 0});
    
    ASSERT_NEAR(w, 0.0, 0.001);
}

TEST(get_rotational_generalized_inverse_mass_base)
{
    BodyCollection bc = create_test_bodies(1);
    // Set the LOCAL inverse inertia tensor (not world)
    bc.inverse_inertia_tensor[0] = smat3(2, 3, 4, 0, 0, 0);
    // With identity orientation, world will equal local after update
    
    vec3 n{1, 0, 0}; // Rotation axis
    
    scalar w = get_rotational_generalized_inverse_mass(bc, 0, n);
    
    // w = n^T * I^-1 * n = (1,0,0)^T * diag(2,3,4) * (1,0,0) = 2
    ASSERT_NEAR(w, 2.0, 0.001);
}

TEST(get_rotational_generalized_inverse_mass_static)
{
    BodyCollection bc = create_test_bodies(1);
    bc.type[0] = BodyType::STATIC;
    
    scalar w = get_rotational_generalized_inverse_mass(bc, 0, vec3{0, 0, 1});
    
    ASSERT_NEAR(w, 0.0, 0.001);
}

TEST(multiple_bodies_integration)
{
    BodyCollection bc = create_test_bodies(3);
    
    // Set different masses and forces
    bc.mass[0] = 1.0;
    bc.mass[1] = 2.0;
    bc.mass[2] = 3.0;
    bc.inverse_mass[0] = 1.0;
    bc.inverse_mass[1] = 0.5;
    bc.inverse_mass[2] = 1.0 / 3.0;
    
    bc.force[0] = vec3{10, 0, 0};
    bc.force[1] = vec3{0, 20, 0};
    bc.force[2] = vec3{0, 0, 30};
    
    scalar dt = 0.01;
    update_position_and_orientation(bc, dt);
    
    // Check each body integrated correctly
    ASSERT_NEAR(bc.linear_velocity[0].x, 0.1, 0.001);  // 10 * 1.0 * 0.01
    ASSERT_NEAR(bc.linear_velocity[1].y, 0.1, 0.001);  // 20 * 0.5 * 0.01
    ASSERT_NEAR(bc.linear_velocity[2].z, 0.1, 0.001);  // 30 * (1/3) * 0.01
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
    RUN_TEST(multiple_bodies_integration)
)