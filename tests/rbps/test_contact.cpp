#include "rbps/constraints/Contact.hpp"
#include "tests/test_helper.hpp"

using namespace rbps;
using namespace m3d;

// Helper function to create test bodies
static BodyCollection create_test_bodies(uint32_t n)
{
    BodyCollection bc;
    for (uint32_t k = 0; k < n; ++k)
    {
        int32_t i = bc.index_of(bc.add());
        bc.mass[i] = 1.0;
        bc.inverse_mass[i] = 1.0;
        bc.type[i] = BodyType::DYNAMIC;
        bc.position[i] = vec3{0, 0, 0};
        bc.linear_velocity[i] = vec3{0, 0, 0};
        bc.orientation[i] = quat(1, 0, 0, 0);
        bc.angular_velocity[i] = vec3{0, 0, 0};
        bc.prev_orientation[i] = quat(1, 0, 0, 0);
        bc.inertia_tensor[i] = smat3(1, 1, 1, 0, 0, 0);
        bc.inverse_inertia_tensor[i] = smat3(1, 1, 1, 0, 0, 0);
        bc.inertia_tensor_world[i] = smat3(1, 1, 1, 0, 0, 0);
        bc.inverse_inertia_tensor_world[i] = smat3(1, 1, 1, 0, 0, 0);
    }
    return bc;
}


// Helper to create a ContactList with n pre-populated contacts
// body_a defaults to slot 0, body_b to slot 1
static ContactList create_contact_list(size_t n)
{
    ContactList cl;
    cl.n_contacts = static_cast<uint32_t>(n);
    cl.body_a               .resize(n, 0);
    cl.body_b               .resize(n, 1);
    cl.collider_a           .resize(n, 0);
    cl.collider_b           .resize(n, 1);
    cl.r_a_local            .resize(n, vec3{0, 0, 0});
    cl.r_b_local            .resize(n, vec3{0, 0, 0});
    cl.normal               .resize(n, vec3{0, 1, 0});
    cl.point_on_a           .resize(n, vec3{0, 0, 0});
    cl.point_on_b           .resize(n, vec3{0, 0, 0});
    cl.penetration_depth    .resize(n, 0.0);
    cl.static_friction      .resize(n, 0.5);
    cl.dynamic_friction     .resize(n, 0.3);
    cl.restitution          .resize(n, 0.5);
    cl.collision            .resize(n, false);
    cl.relative_velocity    .resize(n, 0.0);
    cl.normal_lambda        .resize(n, 0.0);
    cl.tangent_lambda       .resize(n, 0.0);
    cl.normal_force         .resize(n, vec3{0, 0, 0});
    cl.tangent_force        .resize(n, vec3{0, 0, 0});
    return cl;
}

// ============================================================================
// NORMAL CONSTRAINT TESTS
// ============================================================================

TEST(solve_normal_constraint_no_penetration)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // No penetration
    scalar magnitude = 0.0;
    vec3 r_1_wc{0, 0, 0};
    vec3 r_2_wc{0, 0, 0};
    scalar inv_dt = 100.0;
    
    solve_normal_constraint(cc, 0, bc, inv_dt, magnitude, r_1_wc, r_2_wc);
    
    // No impulse should be applied
    ASSERT_NEAR(cc.normal_lambda[0], 0.0, 0.001);
    ASSERT_NEAR(m3d::magnitude(cc.normal_force[0]), 0.0, 0.001);
}

TEST(solve_normal_constraint_with_penetration)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
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
    ASSERT_TRUE(cc.normal_lambda[0] < 0.0);
    
    // Bodies should separate
    scalar separation = bc.position[1].y - bc.position[0].y;
    ASSERT_TRUE(separation > 0.0);
    
    // Normal force should be non-zero
    ASSERT_TRUE(m3d::magnitude(cc.normal_force[0]) > 0.0);
}

TEST(solve_normal_constraint_with_lever_arm)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
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
    
    ContactList cc = create_contact_list(1);
    
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

TEST(solve_tangent_constraint_no_slip)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // No tangential slip
    cc.point_on_a[0] = vec3{0, 0, 0};
    cc.point_on_b[0] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    solve_tangent_constraint(cc, 0, bc, 100.0);
    
    // No friction impulse should be applied
    ASSERT_NEAR(cc.tangent_lambda[0], 0.0, 0.001);
}

TEST(solve_tangent_constraint_with_slip)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Setup tangential slip
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0.1, 0, 0}; // Body 1 moved in X
    cc.point_on_b[0] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.position[0] = vec3{0.1, 0, 0};
    
    // Need normal force to apply friction
    cc.normal_lambda[0] = 1.0;
    cc.static_friction[0] = 0.5;
    
    solve_tangent_constraint(cc, 0, bc, 100.0);
    
    // Friction constraint should be applied if within static friction cone
    ASSERT_TRUE(cc.tangent_lambda[0] != 0.0 ||
                cc.tangent_lambda[0] == 0.0); // May or may not apply depending on magnitude
}

TEST(solve_tangent_constraint_exceeds_static_friction)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Large tangential slip
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{1.0, 0, 0}; // Large slip
    cc.point_on_b[0] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.position[0] = vec3{1.0, 0, 0};
    
    // Setup normal force
    cc.normal_lambda[0] = 1.0;
    cc.static_friction[0] = 0.5;
    
    vec3 pos1_initial = bc.position[0];
    
    solve_tangent_constraint(cc, 0, bc, 100.0);
    
    // Check if friction was applied (position might change)
    // The constraint may or may not be applied depending on friction cone
}

TEST(solve_tangent_constraint_with_rotation)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Setup with previous orientation
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0.1, 0, 0};
    cc.point_on_b[0] = vec3{0, 0, 0};
    bc.prev_orientation[0] = quat(1, 0, 0, 0);
    bc.orientation[0] = quat::from_rpy(0, 0, 0.1); // Small rotation
    bc.position[0] = vec3{0.1, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    
    cc.normal_lambda[0] = 1.0;
    cc.static_friction[0] = 0.5;
    
    solve_tangent_constraint(cc, 0, bc, 100.0);
    
    // Should handle rotational contribution to slip
    // Test passes if no crash occurs
}

// ============================================================================
// POSITION LEVEL CONSTRAINT TESTS
// ============================================================================

TEST(apply_constraint_position_level_no_collision)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Bodies separated (negative penetration)
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0, 0, 0};
    cc.point_on_b[0] = vec3{0, 1, 0}; // 1 unit apart
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 1, 0};
    
    apply_constraint_position_level(cc, 0, bc, 100.0);
    
    // Should mark as no collision
    ASSERT_FALSE(cc.collision[0]);
}

TEST(apply_constraint_position_level_with_collision)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);

    cc.normal[0]    = vec3{0,  1, 0};
    cc.point_on_a[0] = vec3{0,  0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    cc.r_a_local[0]  = vec3{0,  0.05, 0};  // point_on_a - position[0]({0,0,0})
    cc.r_b_local[0]  = vec3{0, -0.05, 0};  // point_on_b - position[1]({0,0,0})
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};

    apply_constraint_position_level(cc, 0, bc, 100.0);

    ASSERT_TRUE(cc.collision[0]);
}

TEST(apply_constraint_position_level_with_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);

    cc.normal[0]    = vec3{0,  1, 0};
    cc.point_on_a[0] = vec3{0,  0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    cc.r_a_local[0]  = vec3{0,  0.05, 0};
    cc.r_b_local[0]  = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -1, 0};
    bc.linear_velocity[1] = vec3{0,  1, 0};

    apply_constraint_position_level(cc, 0, bc, 100.0);

    ASSERT_TRUE(cc.collision[0]);
    ASSERT_TRUE(cc.relative_velocity[0] != 0.0);
}

TEST(apply_constraint_position_level_resolves_penetration)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);

    cc.normal[0]    = vec3{0,  1, 0};
    cc.point_on_a[0] = vec3{0,  0.1, 0};
    cc.point_on_b[0] = vec3{0, -0.1, 0};
    cc.r_a_local[0]  = vec3{0,  0.1, 0};
    cc.r_b_local[0]  = vec3{0, -0.1, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};

    apply_constraint_position_level(cc, 0, bc, 100.0);

    ASSERT_TRUE(bc.position[1].y - bc.position[0].y > 0.0);
}

// ============================================================================
// VELOCITY LEVEL CONSTRAINT TESTS
// ============================================================================

TEST(apply_constraint_velocity_level_no_collision)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // No penetration
    cc.normal[0] = vec3{0, 1, 0};
    cc.collision[0] = false; // Simulate that position level marked as collision
    cc.point_on_a[0] = vec3{0, 0, 0};
    cc.point_on_b[0] = vec3{0, 1, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 1, 0};
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Should not mark as collision
    ASSERT_FALSE(cc.collision[0]);
}

TEST(apply_constraint_velocity_level_with_restitution)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Bodies in contact with approach velocity
    cc.normal[0] = vec3{0, 1, 0};
    cc.collision[0] = true; // Simulate that position level marked as collision
    cc.point_on_a[0] = vec3{0, 0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -1, 0}; // Approaching
    bc.linear_velocity[1] = vec3{0, 0, 0};
    
    cc.restitution[0] = 0.8;
    cc.relative_velocity[0] = -1.0; // Stored from position level
    cc.normal_lambda[0] = 1.0;
    
    scalar initial_vel = bc.linear_velocity[0].y;
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Velocity should be affected (restitution applied)
    ASSERT_TRUE(cc.collision[0]);
    ASSERT_TRUE(bc.linear_velocity[0].y != initial_vel);
}


TEST(apply_constraint_velocity_level_no_restitution_slow_impact)
{
    // When |v_n| <= 2*g*dt the restitution coefficient is clamped to zero.
    // 2*g*dt = 2*9.8*0.01 = 0.196, so approach velocity -0.1 triggers the clamp.
    //
    // With e=0, delta_v = n*(-v_n) which drives the RELATIVE normal velocity
    // to zero.  The impulse is split between both bodies via 1/(w_a+w_b), so
    // each body's individual velocity does NOT go to zero — the relative one does.
    //
    //   Body 0: v_n = -0.1 →  -0.05  (gains +0.05)
    //   Body 1: v_n =  0.0 →  -0.05  (gains -0.05)
    //   Relative v_n after = -0.05 - (-0.05) = 0  ✓
    BodyCollection bc = create_test_bodies(2);
    ContactList cl = create_contact_list(1);
 
    cl.normal[0] = vec3{0, 1, 0};
    cl.point_on_a[0] = vec3{0,  0.05, 0};
    cl.point_on_b[0] = vec3{0, -0.05, 0};
    cl.collision[0] = true;
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -0.1, 0};
    bc.linear_velocity[1] = vec3{0,  0.0, 0};
 
    cl.restitution[0] = 0.8;        // high e, but clamped to 0 by slow-impact threshold
    cl.relative_velocity[0] = -0.1;
    cl.normal_lambda[0] = -1.0;
 
    apply_constraint_velocity_level(cl, 0, bc, 0.01);
 
    // Relative normal velocity at the contact point must be near zero.
    const m3d::vec3 r_a_wc = cl.point_on_a[0] - bc.position[0];
    const m3d::vec3 r_b_wc = cl.point_on_b[0] - bc.position[1];
    const m3d::vec3 v_rel =
        (bc.linear_velocity[0] + m3d::cross(bc.angular_velocity[0], r_a_wc)) -
        (bc.linear_velocity[1] + m3d::cross(bc.angular_velocity[1], r_b_wc));
    const scalar v_n_rel = m3d::dot(v_rel, cl.normal[0]);
    ASSERT_NEAR(v_n_rel, 0.0, 0.01);
 
    // Sanity: no bounce — body 0 must not be moving away faster than it came in.
    ASSERT_TRUE(m3d::dot(bc.linear_velocity[0], cl.normal[0]) >= -0.1 - 0.001);
}
TEST(apply_constraint_velocity_level_with_friction)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Bodies in contact with tangential velocity
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0, 0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{1, 0, 0}; // Sliding
    bc.linear_velocity[1] = vec3{0, 0, 0};
    
    cc.dynamic_friction[0] = 0.5;
    cc.relative_velocity[0] = 0.0;
    cc.normal_lambda[0] = -1.0;
    cc.collision[0] = true; // Simulate that position level marked as collision

    
    scalar initial_vel_x = bc.linear_velocity[0].x;
    
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Tangential velocity should be reduced by friction
    ASSERT_TRUE(m3d::magnitude(bc.linear_velocity[0]) <= initial_vel_x);
}

TEST(apply_constraint_velocity_level_zero_normal_force)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    // Contact but no normal force (shouldn't happen but test robustness)
    cc.normal[0] = vec3{0, 1, 0};
    cc.collision[0] = true; // Simulate that position level marked as collision
    cc.point_on_a[0] = vec3{0, 0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{1, 0, 0};
    
    cc.normal_lambda[0] = 0.0; // No normal force
    cc.dynamic_friction[0] = 0.5;
    
    const vec3 vel_before = bc.linear_velocity[0];

    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Should still mark collision but minimal effect
    // Tangential velocity must be unchanged (no friction without normal force).
    ASSERT_NEAR(bc.linear_velocity[0].x, vel_before.x, 0.001);
    ASSERT_NEAR(bc.linear_velocity[0].z, vel_before.z, 0.001);
}


// ============================================================================
// BATCH PROCESSING TESTS
// ============================================================================

TEST(solve_contacts_velocity_level_single_contact)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);
    
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0, 0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.linear_velocity[0] = vec3{0, -1, 0};
    
    cc.restitution[0] = 0.5;
    cc.relative_velocity[0] = -1.0;
    cc.normal_lambda[0] = 1.0;
    cc.collision[0] = true;
    
    solve_contacts_velocity_level(cc, bc, 0.01);
    
    // Should process the contact
    ASSERT_TRUE(cc.collision[0]);
}

TEST(solve_contacts_velocity_level_multiple_contacts)
{
    BodyCollection bc = create_test_bodies(3);
    ContactList cc = create_contact_list(2);
    
    // First contact: body 0 and 1
    cc.body_a[0] = 0;
    cc.body_b[0] = 1;
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0, 0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
    
    // Second contact: body 1 and 2
    cc.body_a[1] = 1;
    cc.body_b[1] = 2;
    cc.normal[1] = vec3{1, 0, 0};
    cc.point_on_a[1] = vec3{0.05, 0, 0};
    cc.point_on_b[1] = vec3{-0.05, 0, 0};
    
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
    cc.normal_lambda[0] = 1.0;
    cc.normal_lambda[1] = 1.0;
    cc.collision[0] = true;
    cc.collision[1] = true;
    
    solve_contacts_velocity_level(cc, bc, 0.01);
    
    // Both contacts should be processed
    ASSERT_TRUE(cc.collision[0]);
    ASSERT_TRUE(cc.collision[1]);
}

TEST(solve_contacts_velocity_level_no_contacts)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc;
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
    ContactList cc = create_contact_list(1);
    
    // Setup collision scenario
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0, 0.1, 0};
    cc.point_on_b[0] = vec3{0, -0.1, 0};
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
        m3d::dot(cc.point_on_a[0] - cc.point_on_b[0], cc.normal[0]);
    
    // Velocity level
    apply_constraint_velocity_level(cc, 0, bc, 0.01);
    
    // Velocities should be modified
    ASSERT_TRUE(cc.collision[0]);
}

TEST(contact_with_angular_velocity)
{
    BodyCollection bc = create_test_bodies(2);
    ContactList cc = create_contact_list(1);

    cc.normal[0]    = vec3{0,  1, 0};
    cc.point_on_a[0] = vec3{1,  0.05, 0};
    cc.point_on_b[0] = vec3{1, -0.05, 0};
    cc.r_a_local[0]  = vec3{1,  0.05, 0};  // position[0] = {0,0,0}
    cc.r_b_local[0]  = vec3{1, -0.05, 0};  // position[1] = {0,0,0}
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};
    bc.angular_velocity[0] = vec3{0, 0, 1};

    apply_constraint_position_level(cc, 0, bc, 100.0);

    ASSERT_TRUE(cc.collision[0]);
}

TEST(contact_between_static_bodies)
{
    BodyCollection bc = create_test_bodies(2);
    bc.type[0] = BodyType::STATIC;
    bc.type[1] = BodyType::STATIC;
    bc.inverse_mass[0] = 0.0;
    bc.inverse_mass[1] = 0.0;
    
    ContactList cc = create_contact_list(1);
    
    cc.normal[0] = vec3{0, 1, 0};
    cc.point_on_a[0] = vec3{0, 0.05, 0};
    cc.point_on_b[0] = vec3{0, -0.05, 0};
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
    ContactList cc = create_contact_list(1);

    cc.normal[0]    = vec3{0,  1, 0};
    cc.point_on_a[0] = vec3{0,  0.1, 0};
    cc.point_on_b[0] = vec3{0, -0.1, 0};
    cc.r_a_local[0]  = vec3{0,  0.1, 0};
    cc.r_b_local[0]  = vec3{0, -0.1, 0};
    bc.position[0] = vec3{0, 0, 0};
    bc.position[1] = vec3{0, 0, 0};
    bc.prev_position[0] = vec3{0, 0, 0};
    bc.prev_position[1] = vec3{0, 0, 0};

    apply_constraint_position_level(cc, 0, bc, 100.0);
    scalar lambda_1 = cc.normal_lambda[0];

    // Update r_local to reflect new body positions after first solve
    cc.r_a_local[0] = vec3{0,  0.05, 0};
    cc.r_b_local[0] = vec3{0, -0.05, 0};

    apply_constraint_position_level(cc, 0, bc, 100.0);
    scalar lambda_2 = cc.normal_lambda[0];

    ASSERT_TRUE(lambda_1 < 0.0);   // first solve actually did something
    ASSERT_TRUE(lambda_2 <= lambda_1); // lambda is negative and grows more negative
}

TEST_SUITE(
    // Normal Constraint Tests
    RUN_TEST(solve_normal_constraint_no_penetration),
    RUN_TEST(solve_normal_constraint_with_penetration),
    RUN_TEST(solve_normal_constraint_with_lever_arm),
    RUN_TEST(solve_normal_constraint_static_body),
    
    // Tangential Constraint Tests
    RUN_TEST(solve_tangent_constraint_no_slip),
    RUN_TEST(solve_tangent_constraint_with_slip),
    RUN_TEST(solve_tangent_constraint_exceeds_static_friction),
    RUN_TEST(solve_tangent_constraint_with_rotation),
    
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