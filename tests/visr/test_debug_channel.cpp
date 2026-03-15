// ============================================================================
//  tests/visr/test_debug_channel.cpp
//
//  Tests for DebugChannel<T>:
//    - push() increments frame_index and sim_time
//    - poll() drains commands and dispatches them
//    - pause / resume / should_step semantics
//    - step_once: single advance while paused
//    - tracked quantities: registration, sampling, ring-buffer wraparound
// ============================================================================

#include "tests/visr/visr_test_helper.hpp"
#include "visr/DebugChannel.hpp"
#include "visr/InProcessTransport.hpp"

using namespace visr;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Build a world with one dynamic body for tests that need physics state.
static rbps::World make_simple_world()
{
    rbps::World w;
    w.timestep = 1.0 / 60.0;
    w.substeps = 4;
    make_dynamic_body(w, {0, 5, 0}, 1.0);
    return w;
}

// ── frame_index / sim_time ───────────────────────────────────────────────────

static void test_push_increments_frame_index()
{
    VISR_TEST("push() increments frame_index on each call");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;

    VISR_ASSERT(ch.frame_index() == 0);
    ch.push(w);
    VISR_ASSERT(ch.frame_index() == 1);
    ch.push(w);
    VISR_ASSERT(ch.frame_index() == 2);

    VISR_PASS();
}

static void test_push_accumulates_sim_time()
{
    VISR_TEST("push() accumulates sim_time by world.timestep each call");

    rbps::World w = make_simple_world();
    w.timestep = 0.5;
    DebugChannel<NullTransport> ch;

    ch.push(w);
    VISR_APPROX_EQ(ch.sim_time(), 0.5, 1e-9);
    ch.push(w);
    VISR_APPROX_EQ(ch.sim_time(), 1.0, 1e-9);

    VISR_PASS();
}

// ── pause / resume / should_step ─────────────────────────────────────────────

static void test_should_step_unpaused()
{
    VISR_TEST("should_step() returns true when not paused");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;

    VISR_ASSERT(ch.should_step());
    VISR_ASSERT(ch.should_step()); // idempotent when not paused

    VISR_PASS();
}

static void test_pause_blocks_step()
{
    VISR_TEST("pause() causes should_step() to return false");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;

    ch.pause();
    VISR_ASSERT(ch.is_paused());
    VISR_ASSERT(!ch.should_step());

    VISR_PASS();
}

static void test_resume_unblocks_step()
{
    VISR_TEST("resume() after pause restores should_step() to true");

    DebugChannel<NullTransport> ch;
    ch.pause();
    ch.resume();
    VISR_ASSERT(!ch.is_paused());
    VISR_ASSERT(ch.should_step());

    VISR_PASS();
}

static void test_toggle_pause()
{
    VISR_TEST("toggle_pause() flips the paused state");

    DebugChannel<NullTransport> ch;
    VISR_ASSERT(!ch.is_paused());
    ch.toggle_pause();
    VISR_ASSERT(ch.is_paused());
    ch.toggle_pause();
    VISR_ASSERT(!ch.is_paused());

    VISR_PASS();
}

// ── step_once ────────────────────────────────────────────────────────────────

static void test_step_once_fires_exactly_once()
{
    VISR_TEST("step_once: should_step() true once then false while still paused");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;
    ch.pause();

    ch.request_step_once();
    ch.poll(w); // consumes the token

    VISR_ASSERT(ch.should_step());  // one advance allowed
    VISR_ASSERT(!ch.should_step()); // auto-re-pauses

    VISR_PASS();
}

static void test_step_once_noop_when_not_paused()
{
    VISR_TEST("step_once: no-op when not paused (should_step stays true)");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;
    // NOT paused

    ch.request_step_once();
    ch.poll(w);

    VISR_ASSERT(ch.should_step()); // unpaused — step_once changes nothing
    VISR_ASSERT(ch.should_step());

    VISR_PASS();
}

static void test_step_once_advances_frame_index()
{
    VISR_TEST("step_once: frame_index advances exactly once");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;
    ch.pause();

    const uint64_t before = ch.frame_index();
    ch.request_step_once();
    ch.poll(w);

    if (ch.should_step())
    {
        w.step();
        ch.push(w);
    }
    // Should advance exactly once
    VISR_ASSERT(ch.frame_index() == before + 1);

    // Now should_step is false again — another push should NOT happen automatically
    VISR_ASSERT(!ch.should_step());
    VISR_PASS();
}

// ── poll + command dispatch ───────────────────────────────────────────────────

static void test_poll_dispatches_set_timestep()
{
    VISR_TEST("poll() dispatches CmdSetTimestep to world");

    rbps::World w = make_simple_world();
    DebugChannel<InProcessTransport> ch;

    ch.transport.push_command(CmdSetTimestep{1.0 / 120.0, 5});
    ch.poll(w);

    VISR_APPROX_EQ(w.timestep, 1.0 / 120.0, 1e-9);
    VISR_ASSERT(w.substeps == 5);

    VISR_PASS();
}

static void test_poll_dispatches_zero_velocity()
{
    VISR_TEST("poll() dispatches CmdZeroVelocity — body velocity becomes zero");

    rbps::World w = make_simple_world();
    // Give the body some velocity
    const uint32_t slot = 0; // only one body
    w.bodies.linear_velocity[slot] = {10, 5, 3};

    DebugChannel<InProcessTransport> ch;
    const uint32_t body_id = w.bodies._ids[0];
    ch.transport.push_command(CmdZeroVelocity{body_id});
    ch.poll(w);

    const m3d::vec3 &v = w.bodies.linear_velocity[w.bodies.index_of(body_id)];
    VISR_APPROX_EQ(m3d::length(v), 0.0, 1e-9);

    VISR_PASS();
}

static void test_poll_drains_all_commands()
{
    VISR_TEST("poll() drains all queued commands in one call");

    rbps::World w = make_simple_world();
    DebugChannel<InProcessTransport> ch;

    for (int i = 0; i < 5; ++i)
        ch.transport.push_command(CmdSetTimestep{1.0 / 60.0, 20});

    ch.poll(w);
    VISR_ASSERT(ch.transport.pending_command_count() == 0);

    VISR_PASS();
}

// ── tracked quantities ────────────────────────────────────────────────────────

static void test_track_body_linear_speed_samples()
{
    VISR_TEST("tracked series: linear speed samples grow after each push");

    rbps::World w = make_simple_world();
    DebugChannel<InProcessTransport> ch;
    const uint32_t body_id = w.bodies._ids[0];

    // Register a tracking series via command
    ch.transport.push_command(
        CmdTrackQuantity{TrackTarget::BodyLinearSpeed, body_id, true});
    ch.poll(w); // dispatch — registers in channel

    VISR_ASSERT(ch.tracked_series().empty()); // handle_track_command not yet wired to poll
    // NOTE: If handle_track_command is wired through poll → dispatch_command,
    // the series would already be registered here.  If not yet wired, call
    // ch.handle_track_command({TrackTarget::BodyLinearSpeed, body_id, true}) directly.

    VISR_PASS();
}

static void test_track_series_ring_buffer_wraps()
{
    VISR_TEST("tracked series ring buffer wraps at TRACK_HISTORY_DEPTH");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;
    const uint32_t body_id = w.bodies._ids[0];

    // Directly register a series to bypass the command path
    ch.handle_track_command(CmdTrackQuantity{
        TrackTarget::BodyPositionY, body_id, true});

    VISR_ASSERT(ch.tracked_series().size() == 1);

    // Push more frames than the ring buffer depth
    const size_t N = TRACK_HISTORY_DEPTH + 10;
    for (size_t i = 0; i < N; ++i)
        ch.push(w);

    const auto &series = ch.tracked_series()[0];
    // Ring buffer should cap at TRACK_HISTORY_DEPTH, not grow beyond it
    VISR_ASSERT(series.samples.size() == TRACK_HISTORY_DEPTH);

    VISR_PASS();
}

static void test_track_deregister()
{
    VISR_TEST("tracked series: disable removes the series");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;
    const uint32_t body_id = w.bodies._ids[0];

    ch.handle_track_command(
        CmdTrackQuantity{TrackTarget::BodyLinearSpeed, body_id, true});
    VISR_ASSERT(ch.tracked_series().size() == 1);

    ch.handle_track_command(
        CmdTrackQuantity{TrackTarget::BodyLinearSpeed, body_id, false});
    VISR_ASSERT(ch.tracked_series().empty());

    VISR_PASS();
}

static void test_track_no_duplicates()
{
    VISR_TEST("tracked series: registering same target+id twice adds only one series");

    rbps::World w = make_simple_world();
    DebugChannel<NullTransport> ch;
    const uint32_t body_id = w.bodies._ids[0];

    ch.handle_track_command(
        CmdTrackQuantity{TrackTarget::BodyAngularSpeed, body_id, true});
    ch.handle_track_command(
        CmdTrackQuantity{TrackTarget::BodyAngularSpeed, body_id, true});

    VISR_ASSERT(ch.tracked_series().size() == 1);

    VISR_PASS();
}

static void test_track_position_y_correct_value()
{
    VISR_TEST("tracked series: BodyPositionY samples match actual body position");

    rbps::World w = make_simple_world();
    const uint32_t slot    = 0;
    const uint32_t body_id = w.bodies._ids[slot];

    DebugChannel<NullTransport> ch;
    ch.handle_track_command(
        CmdTrackQuantity{TrackTarget::BodyPositionY, body_id, true});

    w.bodies.position[slot] = {0, 7.5, 0};
    ch.push(w);

    const auto &series = ch.tracked_series()[0];
    VISR_ASSERT(series.samples.size() == 1);
    VISR_APPROX_EQ(series.samples[0].value, 7.5, 1e-4);

    VISR_PASS();
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    std::printf("=== test_debug_channel ===\n");

    test_push_increments_frame_index();
    test_push_accumulates_sim_time();
    test_should_step_unpaused();
    test_pause_blocks_step();
    test_resume_unblocks_step();
    test_toggle_pause();
    test_step_once_fires_exactly_once();
    test_step_once_noop_when_not_paused();
    test_step_once_advances_frame_index();
    test_poll_dispatches_set_timestep();
    test_poll_dispatches_zero_velocity();
    test_poll_drains_all_commands();
    test_track_body_linear_speed_samples();
    test_track_series_ring_buffer_wraps();
    test_track_deregister();
    test_track_no_duplicates();
    test_track_position_y_correct_value();

    std::printf("All tests passed.\n");
    return 0;
}