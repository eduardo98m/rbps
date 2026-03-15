#include "tests/test_helper.hpp"
#include "tests/visr/visr_test_helper.hpp"
#include "visr/CommandDispatch.hpp"

using namespace visr;

// ── CmdSetTimestep ────────────────────────────────────────────────────────────

TEST(set_timestep)
{
    rbps::World w;
    w.timestep = 1.0 / 60.0;
    w.substeps = 20;
    dispatch_command(w, CmdSetTimestep{1.0 / 120.0, 40});
    ASSERT_NEAR(w.timestep, 1.0 / 120.0);
    ASSERT_EQ(w.substeps, 40);
}

// ── CmdZeroVelocity ───────────────────────────────────────────────────────────

TEST(zero_velocity)
{
    rbps::World w;
    const uint32_t id   = make_dynamic_body(w);
    const uint32_t slot = w.bodies.index_of(id);
    w.bodies.linear_velocity[slot]  = {10, 5, -3};
    w.bodies.angular_velocity[slot] = {1, 2, 3};
    dispatch_command(w, CmdZeroVelocity{id});
    ASSERT_NEAR(m3d::length(w.bodies.linear_velocity[slot]),  0.0);
    ASSERT_NEAR(m3d::length(w.bodies.angular_velocity[slot]), 0.0);
}

// ── CmdTeleportBody ───────────────────────────────────────────────────────────

TEST(teleport_body_sets_position_and_zeros_velocity)
{
    rbps::World w;
    const uint32_t id   = make_dynamic_body(w);
    const uint32_t slot = w.bodies.index_of(id);
    w.bodies.linear_velocity[slot] = {99, 0, 0};
    dispatch_command(w, CmdTeleportBody{id, {7.0, -3.0, 2.5}, {1, 0, 0, 0}});
    ASSERT_NEAR(w.bodies.position[slot].x,  7.0);
    ASSERT_NEAR(w.bodies.position[slot].y, -3.0);
    ASSERT_NEAR(w.bodies.position[slot].z,  2.5);
    ASSERT_NEAR(m3d::length(w.bodies.linear_velocity[slot]), 0.0);
}

TEST(teleport_updates_prev_position)
{
    rbps::World w;
    const uint32_t id   = make_dynamic_body(w);
    const uint32_t slot = w.bodies.index_of(id);
    dispatch_command(w, CmdTeleportBody{id, {5.0, 5.0, 5.0}, {1, 0, 0, 0}});
    ASSERT_NEAR(w.bodies.prev_position[slot].x, 5.0);
    ASSERT_NEAR(w.bodies.prev_position[slot].y, 5.0);
}

// ── CmdApplyImpulse ───────────────────────────────────────────────────────────

TEST(apply_impulse_increases_velocity)
{
    rbps::World w;
    const uint32_t id   = make_dynamic_body(w, {0,0,0}, 2.0);
    const uint32_t slot = w.bodies.index_of(id);
    const m3d::scalar vy_before = w.bodies.linear_velocity[slot].y;
    dispatch_command(w, CmdApplyImpulse{id, {0, 10, 0}, w.bodies.position[slot]});
    ASSERT_TRUE(w.bodies.linear_velocity[slot].y > vy_before);
}

TEST(apply_impulse_static_body_no_change)
{
    rbps::World w;
    const uint32_t id   = make_static_body(w, {0,-5,0});
    const uint32_t slot = w.bodies.index_of(id);
    const m3d::vec3 v_before = w.bodies.linear_velocity[slot];
    dispatch_command(w, CmdApplyImpulse{id, {0, 999, 0}, {0,-5,0}});
    ASSERT_NEAR(w.bodies.linear_velocity[slot].y, v_before.y);
}

// ── CmdSetJointTarget / CmdSetJointDamping ────────────────────────────────────

TEST(set_joint_target)
{
    rbps::World w;
    const uint32_t pivot = make_static_body (w, {0, 2, 0});
    const uint32_t bob   = make_dynamic_body(w, {1, 2, 0}, 0.5);
    rbps::RevoluteJointParams rjp{};
    rjp.body_1       = pivot;
    rjp.body_2       = bob;
    rjp.aligned_axis = {0, 0, 1};
    rjp.limit_axis   = {1, 0, 0};
    rjp.r_2          = {-1, 0, 0};
    rjp.limited      = true;
    rjp.lower_limit  = -1.0;
    rjp.upper_limit  =  1.0;
    const uint32_t jid   = w.create_revolute_joint(rjp);
    dispatch_command(w, CmdSetJointTarget{jid, 0.75});
    ASSERT_NEAR(w.joints.target_position[w.joints.index_of(jid)], 0.75);
}

TEST(set_joint_damping)
{
    rbps::World w;
    const uint32_t a = make_static_body (w, {0, 2, 0});
    const uint32_t b = make_dynamic_body(w, {1, 2, 0});
    rbps::RevoluteJointParams rjp{};
    rjp.body_1       = a;
    rjp.body_2       = b;
    rjp.aligned_axis = {0, 0, 1};
    rjp.limit_axis   = {1, 0, 0};
    rjp.r_2          = {-1, 0, 0};
    const uint32_t jid = w.create_revolute_joint(rjp);
    dispatch_command(w, CmdSetJointDamping{jid, 12.5});
    ASSERT_NEAR(w.joints.damping[w.joints.index_of(jid)], 12.5);
}

// ── No-op / noop commands don't crash ────────────────────────────────────────

TEST(noop_commands_no_crash)
{
    rbps::World w;
    make_dynamic_body(w);
    dispatch_command(w, CmdSelectBody{0});
    dispatch_command(w, CmdSelectCollider{0});
    dispatch_command(w, CmdClearSelection{});
    dispatch_command(w, CmdTrackQuantity{TrackTarget::BodyLinearSpeed, 0, true});
    dispatch_command(w, CmdSetGravity{{0, -9.81, 0}});
    dispatch_command(w, CmdPause{});
    dispatch_command(w, CmdResume{});
}

TEST(all_command_variants_compile)
{
    rbps::World w;
    make_dynamic_body(w);
    auto d = [&](Command cmd){ dispatch_command(w, cmd); };
    d(CmdPause{});
    d(CmdResume{});
    d(CmdSetTimestep{1.0/60, 20});
    d(CmdSelectBody{0});
    d(CmdSelectCollider{0});
    d(CmdClearSelection{});
    d(CmdZeroVelocity{0});
    d(CmdSetGravity{{0,-9.81,0}});
    d(CmdTrackQuantity{TrackTarget::BodyPositionX, 0, false});
}

TEST_SUITE(
    RUN_TEST(set_timestep),
    RUN_TEST(zero_velocity),
    RUN_TEST(teleport_body_sets_position_and_zeros_velocity),
    RUN_TEST(teleport_updates_prev_position),
    RUN_TEST(apply_impulse_increases_velocity),
    RUN_TEST(apply_impulse_static_body_no_change),
    RUN_TEST(set_joint_target),
    RUN_TEST(set_joint_damping),
    RUN_TEST(noop_commands_no_crash),
    RUN_TEST(all_command_variants_compile)
)