#pragma once
#include <cstdint>
#include <vector>
#include <utility>
#include <atomic>
#include <algorithm>
#include "visr/Transport.hpp"
#include "visr/SnapshotBuilder.hpp"
#include "visr/CommandDispatch.hpp"

// ============================================================================
//  visr/DebugChannel.hpp
//
//  ─── Fixes in this version ───────────────────────────────────────────────
//
//  1. poll() bug (v2): CmdTrackQuantity was forwarded to dispatch_command()
//     whose arm is intentionally empty → handle_track_command() never ran →
//     graphs accumulated no data.  Fixed: channel-level commands (Track,
//     Pause, Resume) are now intercepted before dispatch_command().
//
//  2. Sim-time x-axis (v3): Sample now stores sim_time (seconds) in addition
//     to frame index.  The graph panel uses sim_time for the x-axis so the
//     time scale is physically meaningful and consistent with the simulation.
//
//  3. Pause-while-sampling (v3): push() now takes a did_step flag.
//     When did_step=false (simulation paused) the render snapshot is still
//     published (so the 3-D view stays live) but sample_tracked() and the
//     sim_time / frame_index counters are NOT advanced.  This means the
//     x-axis of every graph flatlines correctly while paused instead of
//     recording identical values over and over against wall-clock time.
//
//  VisrApp usage pattern (see VisrApp.hpp):
//    const bool stepped = channel.should_step();
//    if (stepped) world.step();
//    channel.push(world, stepped);          ← stepped flag propagated here
// ============================================================================

namespace visr
{
    static constexpr size_t TRACK_HISTORY_DEPTH = 512;

    // -------------------------------------------------------------------------
    //  TrackedSeries — ring buffer of (sim_time, value) samples
    // -------------------------------------------------------------------------
    struct TrackedSeries
    {
        TrackTarget target;
        uint32_t    id;

        struct Sample
        {
            uint64_t frame;
            double   sim_time;   // simulation seconds at sample point
            float    value;
        };

        std::vector<Sample> samples;   // ring — oldest at head_ % size when full
        size_t head_ = 0;

        void push(uint64_t frame, double sim_time, float value)
        {
            if (samples.size() < TRACK_HISTORY_DEPTH)
                samples.push_back({frame, sim_time, value});
            else
                samples[head_++ % TRACK_HISTORY_DEPTH] = {frame, sim_time, value};
        }

        // Convenience: value of the most-recently added sample (or 0).
        float last_value() const
        {
            if (samples.empty()) return 0.0f;
            const size_t n = samples.size();
            const size_t idx = (n < TRACK_HISTORY_DEPTH)
                ? n - 1
                : (head_ > 0 ? (head_ - 1) % n : n - 1);
            return samples[idx].value;
        }

        double last_sim_time() const
        {
            if (samples.empty()) return 0.0;
            const size_t n = samples.size();
            const size_t idx = (n < TRACK_HISTORY_DEPTH)
                ? n - 1
                : (head_ > 0 ? (head_ - 1) % n : n - 1);
            return samples[idx].sim_time;
        }
    };

    // -------------------------------------------------------------------------
    //  DebugChannel<Transport>
    // -------------------------------------------------------------------------
    template<typename Transport = NullTransport>
    struct DebugChannel
    {
        Transport transport;
        static_assert(is_debug_transport<Transport>::value,
                      "Transport must implement push_snapshot() and poll_command()");

        uint64_t frame_index_ = 0;
        double   sim_time_    = 0.0;
        bool     paused_      = false;
        bool     step_once_pending_   = false;
        std::atomic<bool> step_once_requested_{false};
        std::vector<TrackedSeries> tracked_series_;

        explicit DebugChannel() = default;
        explicit DebugChannel(Transport t) : transport(std::move(t)) {}

        // -----------------------------------------------------------------
        //  Physics-thread API
        // -----------------------------------------------------------------

        /// Publish the world state to the render thread.
        ///
        /// @param world    Current physics world.
        /// @param did_step True when world.step() was just called this tick.
        ///                 When false (paused or throttled):
        ///                   • The render snapshot IS pushed (so the 3-D view
        ///                     keeps updating, e.g. for contact inspection).
        ///                   • sample_tracked() is NOT called — graphs do not
        ///                     accumulate flat duplicate samples.
        ///                   • sim_time_ and frame_index_ are NOT advanced.
        void push(const rbps::World &world, bool did_step = true)
        {
            // Always build + ship the snapshot so the render thread sees
            // the current body positions regardless of pause state.
            auto snap = build_snapshot(world, frame_index_, sim_time_);
            transport.push_snapshot(snap);

            if (did_step)
            {
                // Only record a graph sample when physics actually advanced.
                sample_tracked(snap);
                sim_time_   += world.timestep;
                ++frame_index_;
            }
        }

        /// Drain the command queue. Channel-level commands are handled here;
        /// world-mutation commands are forwarded to dispatch_command().
        void poll(rbps::World &world)
        {
            bool do_step_once = step_once_requested_.exchange(
                false, std::memory_order_acq_rel);
            if (do_step_once && paused_)
                step_once_pending_ = true;

            Command cmd;
            while (transport.poll_command(cmd))
            {
                // Channel-level: do NOT forward to dispatch_command().
                if (auto *t = std::get_if<CmdTrackQuantity>(&cmd))
                    { handle_track_command(*t); continue; }
                if (std::get_if<CmdPause>(&cmd))
                    { paused_ = true;  continue; }
                if (std::get_if<CmdResume>(&cmd))
                    { paused_ = false; continue; }

                // Everything else mutates World.
                dispatch_command(world, cmd);
            }
        }

        /// Returns true when physics should step this tick.
        /// Has the side effect of consuming a pending step-once token.
        bool should_step()
        {
            if (!paused_) return true;
            if (step_once_pending_)
            {
                step_once_pending_ = false;
                return true;
            }
            return false;
        }

        // -----------------------------------------------------------------
        //  Pause / step-once controls (render-thread safe)
        // -----------------------------------------------------------------
        void pause()        { paused_ = true;      }
        void resume()       { paused_ = false;     }
        void toggle_pause() { paused_ = !paused_;  }

        void request_step_once()
        {
            step_once_requested_.store(true, std::memory_order_release);
        }

        bool     is_paused()    const { return paused_;      }
        uint64_t frame_index()  const { return frame_index_; }
        double   sim_time()     const { return sim_time_;    }

        // -----------------------------------------------------------------
        //  Tracked quantities (render-thread readable after each push)
        // -----------------------------------------------------------------

        // Returns a COPY of the tracked series vector.
        //
        // Returning by const-reference was the segfault root cause:
        //   1. draw_graph_panel captured:  const auto &all_series = ch.tracked_series();
        //   2. During ImGui processing the user clicked a tracking button.
        //   3. If InProcessTransport::push_command is synchronous, or if
        //      poll() is called before the frame ends (e.g. in draw_sim_control_panel),
        //      handle_track_command() runs → push_back() reallocates the vector →
        //      all_series becomes a dangling reference → segfault on next access.
        //
        // Returning by value costs one small vector copy per frame (≤10 elements)
        // and is correct regardless of threading model or transport implementation.
        std::vector<TrackedSeries> tracked_series() const
        {
            return tracked_series_;
        }

        void sample_tracked(const FrameSnapshot &snap)
        {
            for (auto &series : tracked_series_)
            {
                float value = 0.0f;
                switch (series.target)
                {
                case TrackTarget::BodyLinearSpeed:
                    for (const auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = (float)m3d::length(b.linear_velocity);
                    break;
                case TrackTarget::BodyAngularSpeed:
                    for (const auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = (float)m3d::length(b.angular_velocity);
                    break;
                case TrackTarget::BodyPositionX:
                    for (const auto &b : snap.bodies)
                        if (b.id == series.id) value = (float)b.position.x;
                    break;
                case TrackTarget::BodyPositionY:
                    for (const auto &b : snap.bodies)
                        if (b.id == series.id) value = (float)b.position.y;
                    break;
                case TrackTarget::BodyPositionZ:
                    for (const auto &b : snap.bodies)
                        if (b.id == series.id) value = (float)b.position.z;
                    break;
                case TrackTarget::ContactNormalLambda:
                    for (const auto &ct : snap.contacts)
                        if (ct.collider_a == series.id || ct.collider_b == series.id)
                            value += (float)ct.normal_lambda;
                    break;
                case TrackTarget::ContactPenetrationDepth:
                    for (const auto &ct : snap.contacts)
                        if (ct.collider_a == series.id || ct.collider_b == series.id)
                            value = std::max(value, (float)ct.penetration_depth);
                    break;
                case TrackTarget::JointPosition:
                    for (const auto &j : snap.joints)
                        if (j.id == series.id)
                            value = (float)j.current_position;
                    break;
                case TrackTarget::ConstraintLambda:
                    for (const auto &c : snap.constraints)
                        if (c.id == series.id)
                            value = (float)c.lambda;
                    break;
                default: break;
                }
                // Store sim_time (seconds) as the x-axis value — physically
                // meaningful and independent of frame rate / substep count.
                series.push(snap.frame_index, snap.sim_time, value);
            }
        }

        void handle_track_command(const CmdTrackQuantity &cmd)
        {
            if (!cmd.enabled)
            {
                auto it = std::remove_if(tracked_series_.begin(), tracked_series_.end(),
                    [&](const TrackedSeries &s)
                    { return s.target == cmd.target && s.id == cmd.id; });
                tracked_series_.erase(it, tracked_series_.end());
                return;
            }
            for (const auto &s : tracked_series_)
                if (s.target == cmd.target && s.id == cmd.id) return;

            TrackedSeries ts;
            ts.target = cmd.target;
            ts.id     = cmd.id;
            ts.samples.reserve(TRACK_HISTORY_DEPTH);
            tracked_series_.push_back(std::move(ts));
        }
    };

} // namespace visr