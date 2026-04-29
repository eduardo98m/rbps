#include "rbps/API/World.hpp"
#include "tests/test_helper.hpp"
#include "tests/rbps/pipeline_helpers.hpp"

using namespace rbps;
using namespace m3d;

using test::add_dynamic_sphere;
using test::add_static_box;

// ============================================================================
//  Construction tests
// ============================================================================

// TEST(world_default_construction)
// {
//     World w;
//     ASSERT_NEAR(w.timestep, 1.0 / 60.0, 1e-9);
//     ASSERT_TRUE(w.substeps == 20);
//     ASSERT_TRUE(w.bodies.count() == 0);
// }

TEST(world_parameterized_construction)
{
    World w(1.0 / 120.0, 10);
    ASSERT_NEAR(w.timestep, 1.0 / 120.0, 1e-9);
    ASSERT_TRUE(w.substeps == 10);
}

// ============================================================================
//  Body / collider creation
// ============================================================================

TEST(world_create_body_returns_valid_id)
{
    World w;
    BodyParams bp;
    bp.position = vec3{0, 0, 0};
    bp.mass     = 1.0;
    bp.type     = BodyType::DYNAMIC;
    uint32_t id = w.create_body(bp);

    // Body should exist and be at the requested position
    int32_t slot = w.bodies.index_of(id);
    ASSERT_TRUE(slot >= 0);
    ASSERT_NEAR(w.bodies.position[slot].y, 0.0, 1e-6);
}

TEST(world_create_multiple_bodies)
{
    World w;
    uint32_t a = add_dynamic_sphere(w, vec3{0,  2, 0});
    uint32_t b = add_dynamic_sphere(w, vec3{0, -2, 0});
    ASSERT_TRUE(a != b);
    ASSERT_TRUE(w.bodies.count() == 2);
    ASSERT_TRUE(w.colliders.count() == 2);
}

TEST(world_create_collider_registers_in_broad_phase)
{
    World w;
    add_dynamic_sphere(w, vec3{0, 0, 0});
    // The broad phase should have one entry after collider creation
    ASSERT_TRUE(w.broad_phase_state.objects.size() >= 1);
}

// ============================================================================
//  Empty world step — must not crash
// ============================================================================

TEST(world_step_empty_world)
{
    World w;
    w.step();
    w.step();
    // Pass if no crash or exception
}

TEST(world_step_single_body_no_collider)
{
    World w;
    BodyParams bp;
    bp.position        = vec3{0, 10, 0};
    bp.linear_velocity = vec3{0,  0, 0};
    bp.mass            = 1.0;
    bp.type            = BodyType::DYNAMIC;
    w.create_body(bp);
    // Step without a collider — broad/narrow phase should produce no contacts
    w.step();
}

// ============================================================================
//  Free fall under gravity
// ============================================================================

TEST(world_free_fall_moves_body_downward)
{
    // A body with gravity applied should move downward each frame.
    World w(1.0 / 60.0, 20);
    uint32_t bid  = add_dynamic_sphere(w, vec3{0, 10, 0});
    int32_t  slot = w.bodies.index_of(bid);

    scalar y_before = w.bodies.position[slot].y;
    w.step();
    scalar y_after  = w.bodies.position[slot].y;

    ASSERT_TRUE(y_after < y_before);
}

TEST(world_free_fall_velocity_increases)
{
    World w(1.0 / 60.0, 20);
    uint32_t bid  = add_dynamic_sphere(w, vec3{0, 10, 0});
    int32_t  slot = w.bodies.index_of(bid);

    w.step();
    scalar vy_1 = w.bodies.linear_velocity[slot].y;
    w.step();
    scalar vy_2 = w.bodies.linear_velocity[slot].y;

    // Velocity should become more negative each frame (accelerating down)
    ASSERT_TRUE(vy_2 < vy_1);
}

// ============================================================================
//  Static body must not move
// ============================================================================

TEST(world_static_body_does_not_move)
{
    World w;
    uint32_t gid  = add_static_box(w, vec3{0, 0, 0});
    int32_t  slot = w.bodies.index_of(gid);

    vec3 pos_before = w.bodies.position[slot];

    for (int i = 0; i < 10; ++i) w.step();

    vec3 pos_after = w.bodies.position[slot];
    ASSERT_NEAR(pos_after.x, pos_before.x, 1e-6);
    ASSERT_NEAR(pos_after.y, pos_before.y, 1e-6);
    ASSERT_NEAR(pos_after.z, pos_before.z, 1e-6);
}

// ============================================================================
//  Collision: sphere resting on ground
// ============================================================================

TEST(world_sphere_lands_on_ground)
{
    World w(1.0 / 60.0, 20);
    add_static_box(w, vec3{0, -3, 0});          // ground center at -3, top at -1
    uint32_t bid  = add_dynamic_sphere(w, vec3{0, 1, 0}, 0.5); // start low
    int32_t  slot = w.bodies.index_of(bid);

    for (int i = 0; i < 180; ++i){ 
        //std::cout << "Step " << i << ": pos=" << w.bodies.position[slot] << " vel=" << w.bodies.linear_velocity[slot] << std::endl;
        w.step();};

    scalar speed = m3d::magnitude(w.bodies.linear_velocity[slot]);
    ASSERT_TRUE(speed < 2.0);                    // came to approximate rest
    ASSERT_TRUE(w.bodies.position[slot].y > -4.0); // didn't tunnel through
}

TEST(world_sphere_does_not_tunnel_through_ground)
{
    World w(1.0 / 60.0, 25);
    add_static_box(w, vec3{0, -3, 0});
    uint32_t bid  = add_dynamic_sphere(w, vec3{0, 2, 0}, 0.5);
    int32_t  slot = w.bodies.index_of(bid);

    for (int i = 0; i < 300; ++i){ 
        //std::cout << "Step " << i << ": pos=" << w.bodies.position[slot] << " vel=" << w.bodies.linear_velocity[slot] << std::endl;
        w.step();};

    // Ground center at -3, half-extent 2 → bottom face at -5.
    // Sphere must not go below the bottom of the ground box.
    ASSERT_TRUE(w.bodies.position[slot].y > -5.0);
}

// ============================================================================
//  Two dynamic spheres collide
// ============================================================================

TEST(world_two_spheres_collide_and_separate)
{
    // Two spheres moving toward each other should bounce apart.
    World w(1.0 / 60.0, 20);
    uint32_t a = add_dynamic_sphere(w, vec3{-2, 0, 0}, 0.5, 1.0, vec3{ 5, 0, 0});
    uint32_t b = add_dynamic_sphere(w, vec3{ 2, 0, 0}, 0.5, 1.0, vec3{-5, 0, 0});
    int32_t  sa = w.bodies.index_of(a);
    int32_t  sb = w.bodies.index_of(b);

    // Simulate until they should have collided (≈ 0.3 s)
    for (int i = 0; i < 30; ++i) {
        //std::cout << "Step " << i << ": pos=" << w.bodies.position[sa] << " vel=" << w.bodies.linear_velocity[sa] << std::endl;
        w.step();};

    // After collision, they should be moving away from each other
    scalar vx_a = w.bodies.linear_velocity[sa].x;
    scalar vx_b = w.bodies.linear_velocity[sb].x;
    ASSERT_TRUE(vx_a < 0.0 || vx_b > 0.0); // at least one bounced back
}

TEST(world_collision_conserves_momentum_approximately)
{
    // Linear momentum should be approximately conserved during collision
    // (exact conservation depends on restitution and solver accuracy).
    World w(1.0 / 60.0, 20);
    uint32_t a = add_dynamic_sphere(w, vec3{-2, 0, 0}, 0.5, 1.0, vec3{3, 0, 0});
    uint32_t b = add_dynamic_sphere(w, vec3{ 2, 0, 0}, 0.5, 1.0, vec3{0, 0, 0});
    int32_t  sa = w.bodies.index_of(a);
    int32_t  sb = w.bodies.index_of(b);

    scalar px_before = w.bodies.linear_velocity[sa].x * w.bodies.mass[sa]
                     + w.bodies.linear_velocity[sb].x * w.bodies.mass[sb];

    for (int i = 0; i < 60; ++i) w.step();

    scalar px_after = w.bodies.linear_velocity[sa].x * w.bodies.mass[sa]
                    + w.bodies.linear_velocity[sb].x * w.bodies.mass[sb];

    // Momentum conserved within 20% (XPBD is approximate but shouldn't explode)
    ASSERT_NEAR(px_after, px_before, m3d::abs(px_before) * 0.2 + 0.1);
}

// ============================================================================
//  Contact list lifecycle
// ============================================================================

TEST(world_contacts_cleared_each_frame)
{
    // Contacts are cleared at the start of each frame — count should not
    // accumulate unboundedly across frames.
    World w(1.0 / 60.0, 20);
    add_static_box(w, vec3{0, -1, 0});
    add_dynamic_sphere(w, vec3{0, 0, 0}, 0.5);

    for (int i = 0; i < 10; ++i) w.step();

    // After any number of frames, contacts should reflect only the current
    // frame — not grow without bound.
    ASSERT_TRUE(w.contacts.n_contacts <= 16); // sanity upper bound for 2 objects
}

TEST(world_no_contacts_when_bodies_separated)
{
    World w;
    add_dynamic_sphere(w, vec3{  0, 100, 0});
    add_dynamic_sphere(w, vec3{100,   0, 0});

    w.step();

    ASSERT_TRUE(w.contacts.n_contacts == 0);
}

TEST_SUITE(
    // Construction
    //RUN_TEST(world_default_construction),
    RUN_TEST(world_parameterized_construction),

    // Creation
    RUN_TEST(world_create_body_returns_valid_id),
    RUN_TEST(world_create_multiple_bodies),
    RUN_TEST(world_create_collider_registers_in_broad_phase),

    // Empty / no-crash
    RUN_TEST(world_step_empty_world),
    RUN_TEST(world_step_single_body_no_collider),

    // Free fall
    RUN_TEST(world_free_fall_moves_body_downward),
    RUN_TEST(world_free_fall_velocity_increases),

    // Static body
    RUN_TEST(world_static_body_does_not_move),

    // Collision
    RUN_TEST(world_sphere_lands_on_ground),
    RUN_TEST(world_sphere_does_not_tunnel_through_ground),
    RUN_TEST(world_two_spheres_collide_and_separate),
    RUN_TEST(world_collision_conserves_momentum_approximately),

    // Contact lifecycle
    RUN_TEST(world_contacts_cleared_each_frame),
    RUN_TEST(world_no_contacts_when_bodies_separated)
)