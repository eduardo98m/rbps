// ============================================================================
//  tests/visr/test_transport.cpp
//
//  Tests for NullTransport and InProcessTransport.
//  These tests do NOT need a World — they operate on FrameSnapshot directly.
// ============================================================================

#include "tests/visr/visr_test_helper.hpp"
#include "visr/Transport.hpp"
#include "visr/InProcessTransport.hpp"
#include <thread>
#include <chrono>

using namespace visr;

// ── NullTransport ─────────────────────────────────────────────────────────────

static void test_null_transport_push_no_crash()
{
    VISR_TEST("NullTransport: push_snapshot does not crash");

    NullTransport t;
    FrameSnapshot snap;
    snap.frame_index = 7;
    t.push_snapshot(snap); // must not crash or throw

    VISR_PASS();
}

static void test_null_transport_poll_returns_false()
{
    VISR_TEST("NullTransport: poll_command always returns false");

    NullTransport t;
    Command cmd;
    VISR_ASSERT(!t.poll_command(cmd));
    VISR_ASSERT(!t.poll_command(cmd));

    VISR_PASS();
}

static void test_null_transport_satisfies_trait()
{
    VISR_TEST("NullTransport: satisfies is_debug_transport trait");

    static_assert(is_debug_transport<NullTransport>::value,
                  "NullTransport must satisfy the transport trait");
    VISR_PASS();
}

// ── InProcessTransport ───────────────────────────────────────────────────────

static void test_inprocess_nullptr_before_push()
{
    VISR_TEST("InProcessTransport: latest_snapshot is nullptr before first push");

    InProcessTransport t;
    VISR_ASSERT(t.latest_snapshot() == nullptr);

    VISR_PASS();
}

static void test_inprocess_push_then_read()
{
    VISR_TEST("InProcessTransport: push then latest_snapshot returns correct frame");

    InProcessTransport t;
    FrameSnapshot snap;
    snap.frame_index = 42;
    snap.sim_time = 1.234;

    t.push_snapshot(snap);
    const FrameSnapshot *got = t.latest_snapshot();

    VISR_ASSERT(got != nullptr);
    VISR_ASSERT(got->frame_index == 42);
    VISR_APPROX_EQ(got->sim_time, 1.234, 1e-9);

    VISR_PASS();
}

static void test_inprocess_double_buffer_latest_wins()
{
    VISR_TEST("InProcessTransport: second push replaces first (double buffer)");

    InProcessTransport t;

    FrameSnapshot snap1;
    snap1.frame_index = 1;
    FrameSnapshot snap2;
    snap2.frame_index = 2;

    t.push_snapshot(snap1);
    t.push_snapshot(snap2);

    const FrameSnapshot *got = t.latest_snapshot();
    VISR_ASSERT(got != nullptr);
    VISR_ASSERT(got->frame_index == 2);

    VISR_PASS();
}

static void test_inprocess_many_pushes_stable_pointer()
{
    VISR_TEST("InProcessTransport: pointer from latest_snapshot stays valid after re-push");

    InProcessTransport t;

    // Push to slot 0
    FrameSnapshot a;
    a.frame_index = 10;
    t.push_snapshot(a);

    // Push to slot 1 — slot 0 is now the stale slot but its memory is still valid
    FrameSnapshot b;
    b.frame_index = 11;
    t.push_snapshot(b);

    // The latest should be b (slot 1); push a into slot 0 again
    FrameSnapshot c;
    c.frame_index = 12;
    t.push_snapshot(c);

    VISR_ASSERT(t.latest_snapshot()->frame_index == 12);

    VISR_PASS();
}

static void test_inprocess_command_roundtrip()
{
    VISR_TEST("InProcessTransport: push_command → poll_command round-trip");

    InProcessTransport t;

    // No commands yet
    Command out;
    VISR_ASSERT(!t.poll_command(out));

    // Push a command
    t.push_command(CmdPause{});

    // Should get it back
    VISR_ASSERT(t.poll_command(out));
    VISR_ASSERT(std::holds_alternative<CmdPause>(out));

    // Queue should now be empty
    VISR_ASSERT(!t.poll_command(out));

    VISR_PASS();
}

static void test_inprocess_command_fifo_order()
{
    VISR_TEST("InProcessTransport: commands are drained in FIFO order");

    InProcessTransport t;

    t.push_command(CmdPause{});
    t.push_command(CmdResume{});
    t.push_command(CmdSetTimestep{1.0 / 120.0, 10});

    Command out;
    VISR_ASSERT(t.poll_command(out));
    VISR_ASSERT(std::holds_alternative<CmdPause>(out));

    VISR_ASSERT(t.poll_command(out));
    VISR_ASSERT(std::holds_alternative<CmdResume>(out));

    VISR_ASSERT(t.poll_command(out));
    VISR_ASSERT(std::holds_alternative<CmdSetTimestep>(out));
    VISR_APPROX_EQ(std::get<CmdSetTimestep>(out).timestep, 1.0 / 120.0, 1e-9);

    VISR_ASSERT(!t.poll_command(out)); // empty

    VISR_PASS();
}

static void test_inprocess_multiple_command_types()
{
    VISR_TEST("InProcessTransport: all command types survive round-trip");

    InProcessTransport t;

    t.push_command(CmdSelectBody{77});
    t.push_command(CmdApplyImpulse{3, {1, 2, 3}, {0, 0, 0}});
    t.push_command(CmdZeroVelocity{5});
    t.push_command(CmdSetGravity{{0, -9.81, 0}});

    Command out;
    t.poll_command(out);
    VISR_ASSERT(std::get<CmdSelectBody>(out).id == 77);

    t.poll_command(out);
    VISR_APPROX_EQ(std::get<CmdApplyImpulse>(out).impulse.y, 2.0, 1e-6);

    t.poll_command(out);
    VISR_ASSERT(std::get<CmdZeroVelocity>(out).body_id == 5);

    t.poll_command(out);
    VISR_APPROX_EQ(std::get<CmdSetGravity>(out).gravity.y, -9.81, 1e-6);

    VISR_PASS();
}

static void test_inprocess_pending_count()
{
    VISR_TEST("InProcessTransport: pending_command_count matches queue size");

    InProcessTransport t;
    VISR_ASSERT(t.pending_command_count() == 0);

    t.push_command(CmdPause{});
    VISR_ASSERT(t.pending_command_count() == 1);

    t.push_command(CmdResume{});
    VISR_ASSERT(t.pending_command_count() == 2);

    Command out;
    t.poll_command(out);
    VISR_ASSERT(t.pending_command_count() == 1);

    VISR_PASS();
}

static void test_inprocess_threaded_push_poll()
{
    VISR_TEST("InProcessTransport: concurrent push (writer) + poll (reader) is safe");

    // Basic thread-safety smoke test — not an exhaustive race detector.
    InProcessTransport t;
    std::atomic<bool> done{false};
    std::atomic<uint64_t> last_seen{0};

    // Writer thread: push 1000 snapshots
    std::thread writer([&]()
                       {
        for (uint64_t i = 1; i <= 1000; ++i)
        {
            FrameSnapshot snap;
            snap.frame_index = i;
            t.push_snapshot(snap);
        }
        done.store(true); });

    // Reader thread: spin until done, check snapshots are non-decreasing
    std::thread reader([&]()
                       {
        while (!done.load())
        {
            const FrameSnapshot *s = t.latest_snapshot();
            if (s)
            {
                VISR_ASSERT(s->frame_index >= last_seen.load());
                last_seen.store(s->frame_index);
            }
            std::this_thread::yield();
        } });

    writer.join();
    reader.join();

    // After writer finished, latest should be 1000
    VISR_ASSERT(t.latest_snapshot()->frame_index == 1000);

    VISR_PASS();
}

static void test_inprocess_satisfies_trait()
{
    VISR_TEST("InProcessTransport: satisfies is_debug_transport trait");

    static_assert(is_debug_transport<InProcessTransport>::value,
                  "InProcessTransport must satisfy the transport trait");
    VISR_PASS();
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    std::printf("=== test_transport ===\n");

    test_null_transport_push_no_crash();
    test_null_transport_poll_returns_false();
    test_null_transport_satisfies_trait();

    test_inprocess_nullptr_before_push();
    test_inprocess_push_then_read();
    test_inprocess_double_buffer_latest_wins();
    test_inprocess_many_pushes_stable_pointer();
    test_inprocess_command_roundtrip();
    test_inprocess_command_fifo_order();
    test_inprocess_multiple_command_types();
    test_inprocess_pending_count();
    test_inprocess_threaded_push_poll();
    test_inprocess_satisfies_trait();

    std::printf("All tests passed.\n");
    return 0;
}