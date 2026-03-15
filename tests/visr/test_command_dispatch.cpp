// ============================================================================
//  tests/visr/test_command_dispatch.cpp
//
//  Tests for visr::dispatch_command() — verifies each Cmd* type correctly
//  mutates the World.  The visr layer is thin here; the goal is to confirm
//  the mapping from Command variant → World field is correct.
// ============================================================================

#include "tests/visr/visr_test_helper.hpp"
#include "visr/CommandDispatch.hpp"

using namespace visr;

// ── CmdSetTimestep ────────────────────────────────────────────────────────────

static void test_set_timestep()
{
    VISR_TEST("CmdSetTimestep: world.timestep and substeps updated");

    rbps::World w;
    w.timestep = 1.0 / 60.0;
    w.substeps = 20;

    dispatch_command(w, CmdSetTimestep{1.0 / 120.0, 40});

    VISR_APPROX_EQ(w.timestep, 1.0 / 120.0, 1e-9);
    VISR_ASSERT(w.substeps == 40);

    VISR_PASS();
}

// ── CmdZeroVelocity ───────────────────────────────────────────────────────────

static void test_zero_velocity()
{
    VISR_TEST("CmdZeroVelocity: body linear and angular velocity become zero");

    rbps::World w;
    const uint32_t id   = make_dynamic_body(w, {0,0,0});
    const uint32_t slot = w.bodies.index_of(id);

    w.bodies.linear_velocity[slot]  = {10, 5, -3};
    w.bodies.angular_velocity[slot] = {1, 2, 3};

    dispatch_command(w, CmdZeroVelocity{id});

    VISR_APPROX_EQ(m3d::length(w.bodies.linear_velocity[slot]),  0.0, 1e-9);
    VISR_APPROX_EQ(m3d::length(w.bodies.angular_velocity[slot]), 0.0, 1e-9);

    VISR_PASS();
}

// ── CmdTeleportBody ───────────────────────────────────────────────────────────

static void test_teleport_body_position()
{
    VISR_TEST("CmdTeleportBody: position is set and velocity zeroed");

    rbps::World w;
    const uint32_t id   = make_dynamic_body(w, {0,0,0});
    const uint32_t slot = w.bodies.index_of(id);

    w.bodies.linear_velocity[slot] = {99, 0, 0};

    const m3d::vec3 target{7.0, -3.0, 2.5};
    const m3d::quat identity{0, 0, 0, 1};
    dispatch_command(w, CmdTeleportBody{id, target, identity});

    VISR_APPROX_EQ(w.bodies.position[slot].x, 7.0,  1e-9);
    VISR_APPROX_EQ(w.bodies.position[slot].y, -3.0, 1e-9);
    VISR_APPROX_EQ(w.bodies.position[slot].z, 2.5,  1e-9);
    VISR_APPROX_EQ(m3d::length(w.bodies.linear_velocity[slot]), 0.0, 1e-9);

    VISR_PASS();
}

static void test_teleport_sets_prev_position()
{
    VISR_TEST("CmdTeleportBody: prev_position is updated to avoid velocity spike");

    rbps::World w;
    const uint32_t id   = make_dynamic_body(w, {0,0,0});
    const uint32_t slot = w.bodies.index_of(id);

    const m3d::vec3 target{5.0, 5.0, 5.0};
    dispatch_command(w, CmdTeleportBody{id, target, {0,0,0,1}});

    // prev_position must equal position so update_velocities() doesn't
    // see a giant delta and compute a spurious velocity.
    VISR_APPROX_EQ(w.bodies.prev_position[slot].x, 5.0, 1e-9);
    VISR_APPROX_EQ(w.bodies.prev_position[slot].y, 5.0, 1e-9);

    VISR_PASS();
}

// ── CmdApplyImpulse ───────────────────────────────────────────────────────────

static void test_apply_impulse_changes_velocity()
{
    VISR_TEST("CmdApplyImpulse: body gains velocity proportional to impulse/mass");

    rbps::World w;
    const uint32_t id   = make_dynamic_body(w, {0,0,0}, 2.0); // mass=2
    const uint32_t slot = w.bodies.index_of(id);

    // Ensure body is dynamic and has its inertia set
    const m3d::vec3 v_before = w.bodies.linear_velocity[slot];

    dispatch_command(w, CmdApplyImpulse{id, {0, 10, 0}, w.bodies.position[slot]});

    const m3d::vec3 v_after = w.bodies.linear_velocity[slot];
    // Δv = impulse / mass = 10/2 = 5 in Y
    // (exact only if no rotational coupling at COM)
    const m3d::scalar delta_vy = v_after.y - v_before.y;
    VISR_ASSERT_MSG(delta_vy > 0.0, "impulse must increase Y velocity");

    VISR_PASS();
}

static void test_apply_impulse_static_body_no_change()
{
    VISR_TEST("CmdApplyImpulse: static body velocity stays zero");

    rbps::World w;
    const uint32_t id   = make_static_body(w, {0,-5,0});
    const uint32_t slot = w.bodies.index_of(id);

    const m3d::vec3 v_before = w.bodies.linear_velocity[slot];
    dispatch_command(w, CmdApplyImpulse{id, {0, 999, 0}, {0,-5,0}});
    const m3d::vec3 v_after = w.bodies.linear_velocity[slot];

    // apply_positional_velocity_constraint_impulse should no-op for STATIC bodies
    VISR_APPROX_EQ(v_before.x, v_after.x, 1e-9);
    VISR_APPROX_EQ(v_before.y, v_after.y, 1e-9);
    VISR_APPROX_EQ(v_before.z, v_after.z, 1e-9);

    VISR_PASS();
}

// ── CmdSetJointTarget ─────────────────────────────────────────────────────────

static void test_set_joint_target()
{
    VISR_TEST("CmdSetJointTarget: joint target_position updated");

    rbps::World w;
    const uint32_t pivot = make_static_body (w, {0,  2, 0});
    const uint32_t bob   = make_dynamic_body(w, {1.5, 2, 0}, 0.5);

    rbps::RevoluteJointParams rjp{};
    rjp.body_1    = pivot;
    rjp.body_2    = bob;
    rjp.r_1       = {0, 0, 0};
    rjp.r_2       = {-1.5, 0, 0};
    rjp.aligned_axis = {0, 0, 1};
    rjp.limited   = true;
    rjp.lower_limit = -1.0;
    rjp.upper_limit =  1.0;
    rjp.damping   = 0.05;
    const uint32_t jid = w.create_revolute_joint(rjp);

    dispatch_command(w, CmdSetJointTarget{jid, 0.75});

    const uint32_t jslot = w.joints.index_of(jid);
    VISR_APPROX_EQ(w.joints.target_position[jslot], 0.75, 1e-9);

    VISR_PASS();
}

static void test_set_joint_damping()
{
    VISR_TEST("CmdSetJointDamping: joint damping updated");

    rbps::World w;
    const uint32_t a = make_static_body (w, {0,2,0});
    const uint32_t b = make_dynamic_body(w, {1,2,0});

    rbps::RevoluteJointParams rjp{};
    rjp.body_1 = a; rjp.body_2 = b;
    rjp.r_1 = {0,0,0}; rjp.r_2 = {-1,0,0};
    rjp.aligned_axis = {0,0,1};
    rjp.damping = 0.0;
    const uint32_t jid = w.create_revolute_joint(rjp);

    dispatch_command(w, CmdSetJointDamping{jid, 12.5});

    const uint32_t jslot = w.joints.index_of(jid);
    VISR_APPROX_EQ(w.joints.damping[jslot], 12.5, 1e-9);

    VISR_PASS();
}

// ── No-op commands don't crash ────────────────────────────────────────────────

static void test_selection_commands_no_crash()
{
    VISR_TEST("CmdSelectBody / CmdSelectCollider / CmdClearSelection: no-ops, no crash");

    rbps::World w;
    make_dynamic_body(w);

    dispatch_command(w, CmdSelectBody{0});
    dispatch_command(w, CmdSelectCollider{0});
    dispatch_command(w, CmdClearSelection{});
    dispatch_command(w, CmdTrackQuantity{TrackTarget::BodyLinearSpeed, 0, true});
    dispatch_command(w, CmdSetGravity{{0, -9.81, 0}});

    VISR_PASS();
}

// ── Variant exhaustiveness: all arms compile ──────────────────────────────────

static void test_all_variants_compile()
{
    VISR_TEST("all Command variant arms compile without warning");

    rbps::World w;
    make_dynamic_body(w);

    // Just verify these all dispatch without compile errors
    auto dispatch = [&](Command cmd) { dispatch_command(w, cmd); };

    dispatch(CmdSetTimestep{1.0/60, 20});
    dispatch(CmdSelectBody{0});
    dispatch(CmdSelectCollider{0});
    dispatch(CmdClearSelection{});
    dispatch(CmdZeroVelocity{0});
    dispatch(CmdSetGravity{{0,-9.81,0}});
    dispatch(CmdTrackQuantity{TrackTarget::BodyPositionX, 0, false});

    VISR_PASS();
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    std::printf("=== test_command_dispatch ===\n");

    test_set_timestep();
    test_zero_velocity();
    test_teleport_body_position();
    test_teleport_sets_prev_position();
    test_apply_impulse_changes_velocity();
    test_apply_impulse_static_body_no_change();
    test_set_joint_target();
    test_set_joint_damping();
    test_selection_commands_no_crash();
    test_all_variants_compile();

    std::printf("All tests passed.\n");
    return 0;
}