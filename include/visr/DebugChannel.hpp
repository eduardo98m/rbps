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
//  DebugChannel<Transport> is the single glue point between rbps::World
//  and the visualizer transport layer.
//
//  BUG FIX (v2): poll() now handles CmdTrackQuantity, CmdPause, and
//  CmdResume BEFORE forwarding to dispatch_command().
//
//  Previously all three went through dispatch_command() which has empty
//  arms for them — so:
//    • CmdTrackQuantity: handle_track_command() was never called → graphs
//      never accumulated any data → "graphs not working"
//    • CmdPause / CmdResume: paused_ was never set via the transport path
//      (the panel calls channel.pause() directly so the button was fine,
//       but a remote/scripted pause would silently fail)
// ============================================================================

namespace visr
{
    static constexpr size_t TRACK_HISTORY_DEPTH = 512;

    // -------------------------------------------------------------------------
    //  TrackedSeries — ring buffer of (frame_index, value) pairs
    // -------------------------------------------------------------------------
    struct TrackedSeries
    {
        TrackTarget target;
        uint32_t id;

        struct Sample
        {
            uint64_t frame;
            float value;
        };
        std::vector<Sample> samples; // ring — oldest at (head_ % size) when full
        size_t head_ = 0;

        void push(uint64_t frame, float value)
        {
            if (samples.size() < TRACK_HISTORY_DEPTH)
                samples.push_back({frame, value});
            else
                samples[head_++ % TRACK_HISTORY_DEPTH] = {frame, value};
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
        bool     step_once_pending_ = false;
        std::atomic<bool> step_once_requested_{false};
        std::vector<TrackedSeries> tracked_series_;

        explicit DebugChannel() = default;
        explicit DebugChannel(Transport t) : transport(std::move(t)) {}

        // -----------------------------------------------------------------
        //  Physics-thread API
        // -----------------------------------------------------------------

        void push(const rbps::World &world)
        {
            auto snap = build_snapshot(world, frame_index_, sim_time_);
            sample_tracked(snap);
            transport.push_snapshot(snap);
            sim_time_ += world.timestep;
            ++frame_index_;
        }

        /// Drain the command queue and apply each command to the world.
        ///
        /// Commands that mutate DebugChannel state (pause, track) are handled
        /// HERE and never forwarded to dispatch_command() — which would silently
        /// drop them (the arms in CommandDispatch.hpp are intentionally empty).
        void poll(rbps::World &world)
        {
            bool do_step_once = step_once_requested_.exchange(
                false, std::memory_order_acq_rel);
            if (do_step_once && paused_)
                step_once_pending_ = true;

            Command cmd;
            while (transport.poll_command(cmd))
            {
                // ── Channel-level commands: handle here, do NOT forward ───
                if (auto *t = std::get_if<CmdTrackQuantity>(&cmd))
                    { handle_track_command(*t); continue; }

                if (std::get_if<CmdPause>(&cmd))
                    { paused_ = true;  continue; }

                if (std::get_if<CmdResume>(&cmd))
                    { paused_ = false; continue; }

                // ── Everything else mutates World ────────────────────────
                dispatch_command(world, cmd);
            }
        }

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
        void pause()        { paused_ = true;   }
        void resume()       { paused_ = false;  }
        void toggle_pause() { paused_ = !paused_; }

        void request_step_once()
        {
            step_once_requested_.store(true, std::memory_order_release);
        }

        bool is_paused()     const { return paused_;       }
        uint64_t frame_index() const { return frame_index_; }
        double   sim_time()    const { return sim_time_;    }

        // -----------------------------------------------------------------
        //  Tracked quantities
        // -----------------------------------------------------------------
        const std::vector<TrackedSeries> &tracked_series() const
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
                series.push(snap.frame_index, value);
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
                if (s.target == cmd.target && s.id == cmd.id) return; // already tracked

            TrackedSeries ts;
            ts.target = cmd.target;
            ts.id     = cmd.id;
            ts.samples.reserve(TRACK_HISTORY_DEPTH);
            tracked_series_.push_back(std::move(ts));
        }
    };

} // namespace visr