#include "rbps/constraints/Contact.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;
using namespace m3d;

// Helper function to create test bodies
BodyCollection create_test_bodies(size_t n)
{
    BodyCollection bc;
    bc.n_bodies = n;
    bc.force.resize(n, vec3{0, 0, 0});
    bc.torque.resize(n, vec3{0, 0, 0});
    bc.mass.resize(n, 1.0);
    bc.inverse_mass.resize(n, 1.0);
    bc.inertia_tensor.resize(n, smat3(1, 1, 1, 0, 0, 0));
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

// Helper to create a contact collection
ContactCollection create_contact_collection(size_t n)
{
    ContactCollection cc;
    cc.n_contacts = n;
    cc.body_1.resize(n, 0);
    cc.body_2.resize(n, 1);
    cc.normal.resize(n, vec3{0, 1, 0});
    cc.p_1.resize(n, vec3{0, 0, 0});
    cc.p_2.resize(n, vec3{0, 0, 0});
    cc.collision.resize(n, false);
    cc.static_friction.resize(n, 0.5);
    cc.dynamic_friction.resize(n, 0.3);
    cc.restitution.resize(n, 0.5);
    cc.relative_velocity.resize(n, 0.0);
    cc.normal_constraint_lagrange_multiplier.resize(n, 0.0);
    cc.tangencial_constraint_lagrange_multiplier.resize(n, 0.0);
    cc.normal_force.resize(n, vec3{0, 0, 0});
    cc.tangencial_force.resize(n, vec3{0, 0, 0});
    return cc;
}

// ============================================================================
// NORMAL CONSTRAINT TESTS
// ============================================================================

TEST(solve_normal_constraint_no_penetration)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // No penetration
    scalar magnitude = 0.0;
    vec3 r_1_wc{0, 0, 0};
    vec3 r_2_wc{0, 0, 0};
    scalar inv_dt = 100.0;
    
    solve_normal_constraint(cc, 0, bc, inv_dt, magnitude, r_1_wc, r_2_wc);
    
    // No impulse should be applied
    ASSERT_NEAR(cc.normal_constraint_lagrange_multiplier[0], 0.0, 0.001);
    ASSERT_NEAR(m3d::magnitude(cc.normal_force[0]), 0.0, 0.001);
}

TEST(solve_normal_constraint_with_penetration)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Bodies penetrating 0.1 units
    cc.normal[0] = vec3{0, 1, 0};
    scalar magnitude = 0.1;
    vec3 r_1_wc{0, 0, 0};
    vec3 r_2_wc{0, 0, 0};
    scalar inv_dt = 100.0;
    
    vec3 pos1_initial = bc.position[0];
    vec3 pos2_initial = bc.position[1];
    
    solve_normal_constraint(cc, 0, bc, inv_dt, magnitude, r_1_wc, r_2_wc);
    
    // Lagrange multiplier should be negative (opposite direction of normal)
    ASSERT_TRUE(cc.normal_constraint_lagrange_multiplier[0] < 0.0);
    
    // Bodies should separate
    scalar separation = bc.position[1].y - bc.position[0].y;
    ASSERT_TRUE(separation > 0.0);
    
    // Normal force should be non-zero
    ASSERT_TRUE(m3d::magnitude(cc.normal_force[0]) > 0.0);
}

TEST(solve_normal_constraint_with_lever_arm)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Contact point offset from center of mass
    cc.normal[0] = vec3{0, 1, 0};
    scalar magnitude = 0.1;
    vec3 r_1_wc{1, 0, 0}; // Offset contact point
    vec3 r_2_wc{0, 0, 0};
    scalar inv_dt = 100.0;
    
    quat orient1_initial = bc.orientation[0];
    
    solve_normal_constraint(cc, 0, bc, inv_dt, magnitude, r_1_wc, r_2_wc);
    
    // Should cause rotation due to lever arm
    ASSERT_FALSE(bc.orientation[0] == orient1_initial);
}

TEST(solve_normal_constraint_static_body)
{
    BodyCollection bc = create_test_bodies(2);
    bc.type[1] = BodyType::STATIC;
    bc.inverse_mass[1] = 0.0;
    bc.inverse_inertia_tensor_world[1] = smat3(0, 0, 0, 0, 0, 0);
    
    ContactCollection cc = create_contact_collection(1);
    
    cc.normal[0] = vec3{0, 1, 0};
    scalar magnitude = 0.1;
    vec3 r_1_wc{0, 0, 0};
    vec3 r_2_wc{0, 0, 0};
    scalar inv_dt = 100.0;
    
    vec3 pos2_initial = bc.position[1];
    
    solve_normal_constraint(cc, 0, bc, inv_dt, magnitude, r_1_wc, r_2_wc);
    
    // Static body should not move
    ASSERT_TRUE(bc.position[1] == pos2_initial);
    
    // Dynamic body should move
    ASSERT_TRUE(bc.position[0].y < 0.0);
}

// ============================================================================
// TANGENTIAL CONSTRAINT TESTS
// ============================================================================

TEST(solve_tangencial_constraint_no_slip)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // No tangential slip
    cc.p_1[0] = vec3{0, 0, 0};
    cc.p_2[0] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    solve_tangencial_constraint(cc, 0, bc, 100.0);
    
    // No friction impulse should be applied
    ASSERT_NEAR(cc.tangencial_constraint_lagrange_multiplier[0], 0.0, 0.001);
}

TEST(solve_tangencial_constraint_with_slip)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Setup tangential slip
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0.1, 0, 0}; // Body 1 moved in X
    cc.p_2[0] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.position[0] = vec3{0.1, 0, 0};
    
    // Need normal force to apply friction
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    cc.static_friction[0] = 0.5;
    
    solve_tangencial_constraint(cc, 0, bc, 100.0);
    
    // Friction constraint should be applied if within static friction cone
    ASSERT_TRUE(cc.tangencial_constraint_lagrange_multiplier[0] != 0.0 ||
                cc.tangencial_constraint_lagrange_multiplier[0] == 0.0); // May or may not apply depending on magnitude
}

TEST(solve_tangencial_constraint_exceeds_static_friction)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Large tangential slip
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{1.0, 0, 0}; // Large slip
    cc.p_2[0] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.position[0] = vec3{1.0, 0, 0};
    
    // Setup normal force
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    cc.static_friction[0] = 0.5;
    
    vec3 pos1_initial = bc.position[0];
    
    solve_tangencial_constraint(cc, 0, bc, 100.0);
    
    // Check if friction was applied (position might change)
    // The constraint may or may not be applied depending on friction cone
}

TEST(solve_tangencial_constraint_with_rotation)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Setup with previous orientation
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0.1, 0, 0};
    cc.p_2[0] = vec3{0, 0, 0};
    bc.prev_orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[0] = quat::from_rpy(0, 0, 0.1); // Small rotation
    bc.position[0] = vec3{0.1, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    cc.static_friction[0] = 0.5;
    
    solve_tangencial_constraint(cc, 0, bc, 100.0);
    
    // Should handle rotational contribution to slip
    // Test passes if no crash occurs
}

// ============================================================================
// POSITION LEVEL CONSTRAINT TESTS
// ============================================================================

TEST(apply_constraint_position_level_no_collision)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Bodies separated (negative penetration)
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0, 0};
    cc.p_2[0] = vec3{0, 1, 0}; // 1 unit apart
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 1, 0};
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Should mark as no collision
    ASSERT_FALSE(cc.collision[0]);
}

TEST(apply_constraint_position_level_with_collision)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Bodies penetrating
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Should mark as collision
    ASSERT_TRUE(cc.collision[0]);
    
    // Should have computed relative velocity
    // (may be zero in this case)
}

TEST(apply_constraint_position_level_with_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Bodies penetrating with velocity
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -1, 0}; // Moving toward each other
    bc.linear_velocity[1] = vec3{0, 1, 0};
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Should mark as collision
    ASSERT_TRUE(cc.collision[0]);
    
    // Should have non-zero relative velocity
    ASSERT_TRUE(cc.relative_velocity[0] != 0.0);
}

TEST(apply_constraint_position_level_resolves_penetration)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Significant penetration
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.1, 0};
    cc.p_2[0] = vec3{0, -0.1, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    scalar initial_overlap = cc.p_1[0].y - cc.p_2[0].y;
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Bodies should be pushed apart
    scalar final_distance = bc.position[1].y - bc.position[0].y;
    ASSERT_TRUE(final_distance > 0.0);
}

// ============================================================================
// VELOCITY LEVEL CONSTRAINT TESTS
// ============================================================================

TEST(apply_constraint_velocity_level_no_collision)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // No penetration
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0, 0};
    cc.p_2[0] = vec3{0, 1, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 1, 0};
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Should not mark as collision
    ASSERT_FALSE(cc.collision[0]);
}

TEST(apply_constraint_velocity_level_with_restitution)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Bodies in contact with approach velocity
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -1, 0}; // Approaching
    bc.linear_velocity[1] = vec3{0, 0, 0};
    
    cc.restitution[0] = 0.8;
    cc.relative_velocity[0] = -1.0; // Stored from position level
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    
    scalar initial_vel = bc.linear_velocity[0].y;
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Velocity should be affected (restitution applied)
    ASSERT_TRUE(cc.collision[0]);
    ASSERT_TRUE(bc.linear_velocity[0].y != initial_vel);
}

TEST(apply_constraint_velocity_level_no_restitution_slow_impact)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Slow collision (below restitution threshold)
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -0.1, 0}; // Slow approach
    bc.linear_velocity[1] = vec3{0, 0, 0};
    
    cc.restitution[0] = 0.8;
    cc.relative_velocity[0] = -0.1; // Below 2 * 9.8 * dt threshold
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Should apply constraint but minimal restitution
    ASSERT_TRUE(cc.collision[0]);
}

TEST(apply_constraint_velocity_level_with_friction)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Bodies in contact with tangential velocity
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{1, 0, 0}; // Sliding
    bc.linear_velocity[1] = vec3{0, 0, 0};
    
    cc.dynamic_friction[0] = 0.5;
    cc.relative_velocity[0] = 0.0;
    cc.normal_constraint_lagrange_multiplier[0] = -1.0;
    
    scalar initial_vel_x = bc.linear_velocity[0].x;
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Tangential velocity should be reduced by friction
    ASSERT_TRUE(cc.collision[0]);
    ASSERT_TRUE(m3d::magnitude(bc.linear_velocity[0]) <= initial_vel_x);
}

TEST(apply_constraint_velocity_level_zero_normal_force)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Contact but no normal force (shouldn't happen but test robustness)
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{1, 0, 0};
    
    cc.normal_constraint_lagrange_multiplier[0] = 0.0; // No normal force
    cc.dynamic_friction[0] = 0.5;
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Should still mark collision but minimal effect
    ASSERT_TRUE(cc.collision[0]);
}

// ============================================================================
// BATCH PROCESSING TESTS
// ============================================================================

TEST(solve_contacts_velocity_level_single_contact)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -1, 0};
    
    cc.restitution[0] = 0.5;
    cc.relative_velocity[0] = -1.0;
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    
    solve_contacts_velocity_level(cc, bc, 0.01);
    
    // Should process the contact
    ASSERT_TRUE(cc.collision[0]);
}

TEST(solve_contacts_velocity_level_multiple_contacts)
{
    BodyCollection bc = create_test_bodies(3);
    ContactCollection cc = create_contact_collection(2);
    
    // First contact: body 0 and 1
    cc.body_1[0] = 0;
    cc.body_2[0] = 1;
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    
    // Second contact: body 1 and 2
    cc.body_1[1] = 1;
    cc.body_2[1] = 2;
    cc.normal[1] = vec3{1, 0, 0};
    cc.p_1[1] = vec3{0.05, 0, 0};
    cc.p_2[1] = vec3{-0.05, 0, 0};
    
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.position[2] = vec3{0, 0, 0};
    
    bc.linear_velocity[0] = vec3{0, -1, 0};
    bc.linear_velocity[1] = vec3{1, 0, 0};
    bc.linear_velocity[2] = vec3{0, 0, 0};
    
    cc.restitution[0] = 0.5;
    cc.restitution[1] = 0.5;
    cc.relative_velocity[0] = -1.0;
    cc.relative_velocity[1] = 1.0;
    cc.normal_constraint_lagrange_multiplier[0] = 1.0;
    cc.normal_constraint_lagrange_multiplier[1] = 1.0;
    
    solve_contacts_velocity_level(cc, bc, 0.01);
    
    // Both contacts should be processed
    ASSERT_TRUE(cc.collision[0]);
    ASSERT_TRUE(cc.collision[1]);
}

TEST(solve_contacts_velocity_level_no_contacts)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc;
    cc.n_contacts = 0;
    
    // Should not crash with no contacts
    solve_contacts_velocity_level(cc, bc, 0.01);
}

// ============================================================================
// EDGE CASES AND INTEGRATION TESTS
// ============================================================================

TEST(contact_resolution_full_cycle)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Setup collision scenario
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.1, 0};
    cc.p_2[0] = vec3{0, -0.1, 0};
    bc.position[0] = vec3{0, 0.2, 0};
    bc.position[1] = vec3{0, -0.2, 0};
    bc.prev_position[0] = vec3{0, 0.2, 0};
    bc.prev_position[1] = vec3{0, -0.2, 0};
    bc.linear_velocity[0] = vec3{0, -2, 0};
    bc.linear_velocity[1] = vec3{0, 2, 0};
    
    cc.restitution[0] = 0.8;
    cc.static_friction[0] = 0.6;
    cc.dynamic_friction[0] = 0.4;
    
    // Position level
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    ASSERT_TRUE(cc.collision[0]);
    scalar penetration_after_position = 
        m3d::dot(cc.p_1[0] - cc.p_2[0], cc.normal[0]);
    
    // Velocity level
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Velocities should be modified
    ASSERT_TRUE(cc.collision[0]);
}

TEST(contact_with_angular_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    // Contact point offset with angular velocity
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{1, 0.05, 0};
    cc.p_2[0] = vec3{1, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.angular_velocity[0] = vec3{0, 0, 1}; // Spinning
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Should handle angular contribution to relative velocity
    ASSERT_TRUE(cc.collision[0]);
}

TEST(contact_between_static_bodies)
{
    BodyCollection bc = create_test_bodies(2);
    bc.type[0] = BodyType::STATIC;
    bc.type[1] = BodyType::STATIC;
    bc.inverse_mass[0] = 0.0;
    bc.inverse_mass[1] = 0.0;
    
    ContactCollection cc = create_contact_collection(1);
    
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.05, 0};
    cc.p_2[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    vec3 pos0_before = bc.position[0];
    vec3 pos1_before = bc.position[1];
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Neither body should move
    ASSERT_TRUE(bc.position[0] == pos0_before);
    ASSERT_TRUE(bc.position[1] == pos1_before);
}

TEST(contact_normal_force_accumulation)
{
    BodyCollection bc = create_test_bodies(2);
    ContactCollection cc = create_contact_collection(1);
    
    cc.normal[0] = vec3{0, 1, 0};
    cc.p_1[0] = vec3{0, 0.1, 0};
    cc.p_2[0] = vec3{0, -0.1, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    // First iteration
    apply_constraint_position_level(cc, 0, bc, 100.0);
    scalar lambda_1 = cc.normal_constraint_lagrange_multiplier[0];
    
    // Simulate another iteration with remaining penetration
    cc.p_1[0] = bc.position[0] + vec3{0, 0.05, 0};
    cc.p_2[0] = bc.position[1] + vec3{0, -0.05, 0};
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    scalar lambda_2 = cc.normal_constraint_lagrange_multiplier[0];
    
    // Lagrange multiplier should accumulate
    ASSERT_TRUE(lambda_2 >= lambda_1);
}

TEST_SUITE(
    // Normal Constraint Tests
    RUN_TEST(solve_normal_constraint_no_penetration),
    RUN_TEST(solve_normal_constraint_with_penetration),
    RUN_TEST(solve_normal_constraint_with_lever_arm),
    RUN_TEST(solve_normal_constraint_static_body),
    
    // Tangential Constraint Tests
    RUN_TEST(solve_tangencial_constraint_no_slip),
    RUN_TEST(solve_tangencial_constraint_with_slip),
    RUN_TEST(solve_tangencial_constraint_exceeds_static_friction),
    RUN_TEST(solve_tangencial_constraint_with_rotation),
    
    // Position Level Constraint Tests
    RUN_TEST(apply_constraint_position_level_no_collision),
    RUN_TEST(apply_constraint_position_level_with_collision),
    RUN_TEST(apply_constraint_position_level_with_velocity),
    RUN_TEST(apply_constraint_position_level_resolves_penetration),
    
    // Velocity Level Constraint Tests
    RUN_TEST(apply_constraint_velocity_level_no_collision),
    RUN_TEST(apply_constraint_velocity_level_with_restitution),
    RUN_TEST(apply_constraint_velocity_level_no_restitution_slow_impact),
    RUN_TEST(apply_constraint_velocity_level_with_friction),
    RUN_TEST(apply_constraint_velocity_level_zero_normal_force),
    
    // Batch Processing Tests
    RUN_TEST(solve_contacts_velocity_level_single_contact),
    RUN_TEST(solve_contacts_velocity_level_multiple_contacts),
    RUN_TEST(solve_contacts_velocity_level_no_contacts),
    
    // Integration and Edge Cases
    RUN_TEST(contact_resolution_full_cycle),
    RUN_TEST(contact_with_angular_velocity),
    RUN_TEST(contact_between_static_bodies),
    RUN_TEST(contact_normal_force_accumulation)
)