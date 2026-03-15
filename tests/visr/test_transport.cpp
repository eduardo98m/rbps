#include "tests/test_helper.hpp"
#include "visr/Transport.hpp"
#include "visr/InProcessTransport.hpp"
#include <thread>

using namespace visr;

// ── NullTransport ─────────────────────────────────────────────────────────────

TEST(null_transport_push_no_crash)
{
    NullTransport t;
    FrameSnapshot snap;
    snap.frame_index = 7;
    t.push_snapshot(snap); // must not throw
}

TEST(null_transport_poll_always_false)
{
    NullTransport t;
    Command cmd;
    ASSERT_FALSE(t.poll_command(cmd));
    ASSERT_FALSE(t.poll_command(cmd));
}

TEST(null_transport_satisfies_trait)
{
    static_assert(is_debug_transport<NullTransport>::value,
                  "NullTransport must satisfy the transport trait");
    ASSERT_TRUE(is_debug_transport<NullTransport>::value);
}

// ── InProcessTransport ───────────────────────────────────────────────────────

TEST(inprocess_nullptr_before_first_push)
{
    InProcessTransport t;
    ASSERT_TRUE(t.latest_snapshot() == nullptr);
}

TEST(inprocess_push_then_read)
{
    InProcessTransport t;
    FrameSnapshot snap;
    snap.frame_index = 42;
    snap.sim_time    = 1.234;
    t.push_snapshot(snap);
    const FrameSnapshot *got = t.latest_snapshot();
    ASSERT_TRUE(got != nullptr);
    ASSERT_EQ(got->frame_index, 42u);
    ASSERT_NEAR(got->sim_time, 1.234);
}

TEST(inprocess_double_buffer_latest_wins)
{
    InProcessTransport t;
    FrameSnapshot s1; s1.frame_index = 1;
    FrameSnapshot s2; s2.frame_index = 2;
    t.push_snapshot(s1);
    t.push_snapshot(s2);
    ASSERT_EQ(t.latest_snapshot()->frame_index, 2u);
}

TEST(inprocess_three_pushes_latest_wins)
{
    InProcessTransport t;
    FrameSnapshot s; 
    s.frame_index = 10; t.push_snapshot(s);
    s.frame_index = 11; t.push_snapshot(s);
    s.frame_index = 12; t.push_snapshot(s);
    ASSERT_EQ(t.latest_snapshot()->frame_index, 12u);
}

TEST(inprocess_command_roundtrip)
{
    InProcessTransport t;
    Command out;
    ASSERT_FALSE(t.poll_command(out));
    t.push_command(CmdPause{});
    ASSERT_TRUE(t.poll_command(out));
    ASSERT_TRUE(std::holds_alternative<CmdPause>(out));
    ASSERT_FALSE(t.poll_command(out));
}

TEST(inprocess_command_fifo_order)
{
    InProcessTransport t;
    t.push_command(CmdPause{});
    t.push_command(CmdResume{});
    t.push_command(CmdSetTimestep{1.0 / 120.0, 10});
    Command out;
    t.poll_command(out); ASSERT_TRUE(std::holds_alternative<CmdPause>(out));
    t.poll_command(out); ASSERT_TRUE(std::holds_alternative<CmdResume>(out));
    t.poll_command(out);
    ASSERT_TRUE(std::holds_alternative<CmdSetTimestep>(out));
    ASSERT_NEAR(std::get<CmdSetTimestep>(out).timestep, 1.0 / 120.0);
    ASSERT_FALSE(t.poll_command(out));
}

TEST(inprocess_all_command_types_roundtrip)
{
    InProcessTransport t;
    t.push_command(CmdSelectBody{77});
    t.push_command(CmdApplyImpulse{3, {1,2,3}, {0,0,0}});
    t.push_command(CmdZeroVelocity{5});
    t.push_command(CmdSetGravity{{0,-9.81,0}});
    Command out;
    t.poll_command(out); ASSERT_EQ(std::get<CmdSelectBody>(out).id, 77u);
    t.poll_command(out); ASSERT_NEAR(std::get<CmdApplyImpulse>(out).impulse.y, 2.0);
    t.poll_command(out); ASSERT_EQ(std::get<CmdZeroVelocity>(out).body_id, 5u);
    t.poll_command(out); ASSERT_NEAR(std::get<CmdSetGravity>(out).gravity.y, -9.81);
}

TEST(inprocess_pending_count)
{
    InProcessTransport t;
    ASSERT_EQ(t.pending_command_count(), 0u);
    t.push_command(CmdPause{});
    ASSERT_EQ(t.pending_command_count(), 1u);
    t.push_command(CmdResume{});
    ASSERT_EQ(t.pending_command_count(), 2u);
    Command out; t.poll_command(out);
    ASSERT_EQ(t.pending_command_count(), 1u);
}

TEST(inprocess_threaded_push_poll_safe)
{
    InProcessTransport t;
    std::atomic<bool> done{false};
    std::atomic<uint64_t> last_seen{0};

    std::thread writer([&]()
    {
        for (uint64_t i = 1; i <= 1000; ++i)
        {
            FrameSnapshot s; s.frame_index = i;
            t.push_snapshot(s);
        }
        done.store(true);
    });

    std::thread reader([&]()
    {
        while (!done.load())
        {
            const FrameSnapshot *s = t.latest_snapshot();
            if (s)
            {
                ASSERT_TRUE(s->frame_index >= last_seen.load());
                last_seen.store(s->frame_index);
            }
            std::this_thread::yield();
        }
    });

    writer.join();
    reader.join();
    ASSERT_EQ(t.latest_snapshot()->frame_index, 1000u);
}

TEST(inprocess_satisfies_trait)
{
    static_assert(is_debug_transport<InProcessTransport>::value,
                  "InProcessTransport must satisfy the transport trait");
    ASSERT_TRUE(is_debug_transport<InProcessTransport>::value);
}

TEST_SUITE(
    RUN_TEST(null_transport_push_no_crash),
    RUN_TEST(null_transport_poll_always_false),
    RUN_TEST(null_transport_satisfies_trait),
    RUN_TEST(inprocess_nullptr_before_first_push),
    RUN_TEST(inprocess_push_then_read),
    RUN_TEST(inprocess_double_buffer_latest_wins),
    RUN_TEST(inprocess_three_pushes_latest_wins),
    RUN_TEST(inprocess_command_roundtrip),
    RUN_TEST(inprocess_command_fifo_order),
    RUN_TEST(inprocess_all_command_types_roundtrip),
    RUN_TEST(inprocess_pending_count),
    RUN_TEST(inprocess_threaded_push_poll_safe),
    RUN_TEST(inprocess_satisfies_trait)
)