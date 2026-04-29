#include "tests/test_helper.hpp"
#include "tests/visr/visr_test_helper.hpp"
#include "visr/DebugChannel.hpp"
#include "visr/InProcessTransport.hpp"

using namespace visr;

static rbps::World make_world()
{
    rbps::World w;
    w.timestep = 1.0 / 60.0;
    w.substeps = 4;
    make_dynamic_body(w, {0, 5, 0}, 1.0);
    return w;
}

// ── frame_index / sim_time ───────────────────────────────────────────────────

TEST(push_increments_frame_index)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    ASSERT_EQ(ch.frame_index(), 0u);
    ch.push(w); ASSERT_EQ(ch.frame_index(), 1u);
    ch.push(w); ASSERT_EQ(ch.frame_index(), 2u);
}

TEST(push_accumulates_sim_time)
{
    rbps::World w = make_world();
    w.timestep = 0.5;
    DebugChannel<NullTransport> ch;
    ch.push(w); ASSERT_NEAR(ch.sim_time(), 0.5);
    ch.push(w); ASSERT_NEAR(ch.sim_time(), 1.0);
}

// ── pause / resume / should_step ─────────────────────────────────────────────

TEST(should_step_true_when_unpaused)
{
    DebugChannel<NullTransport> ch;
    ASSERT_TRUE(ch.should_step());
    ASSERT_TRUE(ch.should_step());
}

TEST(pause_blocks_should_step)
{
    DebugChannel<NullTransport> ch;
    ch.pause();
    ASSERT_TRUE(ch.is_paused());
    ASSERT_FALSE(ch.should_step());
}

TEST(resume_restores_should_step)
{
    DebugChannel<NullTransport> ch;
    ch.pause();
    ch.resume();
    ASSERT_FALSE(ch.is_paused());
    ASSERT_TRUE(ch.should_step());
}

TEST(toggle_pause_flips_state)
{
    DebugChannel<NullTransport> ch;
    ASSERT_FALSE(ch.is_paused());
    ch.toggle_pause(); ASSERT_TRUE(ch.is_paused());
    ch.toggle_pause(); ASSERT_FALSE(ch.is_paused());
}

// ── step_once ────────────────────────────────────────────────────────────────

TEST(step_once_fires_exactly_once)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    ch.pause();
    ch.request_step_once();
    ch.poll(w);
    ASSERT_TRUE(ch.should_step());   // one advance allowed
    ASSERT_FALSE(ch.should_step());  // auto-re-pauses
}

TEST(step_once_noop_when_not_paused)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    ch.request_step_once();
    ch.poll(w);
    ASSERT_TRUE(ch.should_step());
    ASSERT_TRUE(ch.should_step()); // still unpaused
}

TEST(step_once_advances_frame_index_by_one)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    ch.pause();
    ch.request_step_once();
    ch.poll(w);
    const uint64_t before = ch.frame_index();
    if (ch.should_step()) { w.step(); ch.push(w); }
    ASSERT_EQ(ch.frame_index(), before + 1);
    ASSERT_FALSE(ch.should_step());
}

// ── poll + command dispatch ───────────────────────────────────────────────────

TEST(poll_dispatches_set_timestep)
{
    rbps::World w = make_world();
    DebugChannel<InProcessTransport> ch;
    ch.transport.push_command(CmdSetTimestep{1.0 / 120.0, 5});
    ch.poll(w);
    ASSERT_NEAR(w.timestep, 1.0 / 120.0);
    ASSERT_EQ(w.substeps, 5);
}

TEST(poll_dispatches_zero_velocity)
{
    rbps::World w = make_world();
    const uint32_t body_id = get_first_body_id(w);
    w.bodies.linear_velocity[0] = {10, 5, 3};
    DebugChannel<InProcessTransport> ch;
    ch.transport.push_command(CmdZeroVelocity{body_id});
    ch.poll(w);
    const m3d::vec3 &v = w.bodies.linear_velocity[w.bodies.index_of(body_id)];
    ASSERT_NEAR(m3d::length(v), 0.0);
}

TEST(poll_drains_all_commands)
{
    rbps::World w = make_world();
    DebugChannel<InProcessTransport> ch;
    for (int i = 0; i < 5; ++i)
        ch.transport.push_command(CmdSetTimestep{1.0 / 60.0, 20});
    ch.poll(w);
    ASSERT_EQ(ch.transport.pending_command_count(), 0u);
}

// ── tracked quantities ────────────────────────────────────────────────────────

TEST(track_series_ring_buffer_wraps)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    const uint32_t body_id = get_first_body_id(w);
    ch.handle_track_command(CmdTrackQuantity{TrackTarget::BodyPositionY, body_id, true});
    ASSERT_EQ(ch.tracked_series().size(), 1u);
    for (size_t i = 0; i < TRACK_HISTORY_DEPTH + 10; ++i)
        ch.push(w);
    ASSERT_EQ(ch.tracked_series()[0].samples.size(), TRACK_HISTORY_DEPTH);
}

TEST(track_deregister_removes_series)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    const uint32_t body_id = get_first_body_id(w);
    ch.handle_track_command(CmdTrackQuantity{TrackTarget::BodyLinearSpeed, body_id, true});
    ASSERT_EQ(ch.tracked_series().size(), 1u);
    ch.handle_track_command(CmdTrackQuantity{TrackTarget::BodyLinearSpeed, body_id, false});
    ASSERT_TRUE(ch.tracked_series().empty());
}

TEST(track_no_duplicates)
{
    rbps::World w = make_world();
    DebugChannel<NullTransport> ch;
    const uint32_t body_id = get_first_body_id(w);
    ch.handle_track_command(CmdTrackQuantity{TrackTarget::BodyAngularSpeed, body_id, true});
    ch.handle_track_command(CmdTrackQuantity{TrackTarget::BodyAngularSpeed, body_id, true});
    ASSERT_EQ(ch.tracked_series().size(), 1u);
}

TEST(track_position_y_correct_value)
{
    rbps::World w = make_world();
    const uint32_t body_id = get_first_body_id(w);
    DebugChannel<NullTransport> ch;
    ch.handle_track_command(CmdTrackQuantity{TrackTarget::BodyPositionY, body_id, true});
    w.bodies.position[0] = {0, 7.5, 0};
    ch.push(w);
    ASSERT_EQ(ch.tracked_series()[0].samples.size(), 1u);
    ASSERT_NEAR(ch.tracked_series()[0].samples[0].value, 7.5, 1e-4);
}

TEST_SUITE(
    RUN_TEST(push_increments_frame_index),
    RUN_TEST(push_accumulates_sim_time),
    RUN_TEST(should_step_true_when_unpaused),
    RUN_TEST(pause_blocks_should_step),
    RUN_TEST(resume_restores_should_step),
    RUN_TEST(toggle_pause_flips_state),
    RUN_TEST(step_once_fires_exactly_once),
    RUN_TEST(step_once_noop_when_not_paused),
    RUN_TEST(step_once_advances_frame_index_by_one),
    RUN_TEST(poll_dispatches_set_timestep),
    RUN_TEST(poll_dispatches_zero_velocity),
    RUN_TEST(poll_drains_all_commands),
    RUN_TEST(track_series_ring_buffer_wraps),
    RUN_TEST(track_deregister_removes_series),
    RUN_TEST(track_no_duplicates),
    RUN_TEST(track_position_y_correct_value)
)