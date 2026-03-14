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
//  and the visualizer transport layer.  It is NOT part of World — you own
//  it in your main loop and call push / poll around world.step().
//
//  Template parameter:
//    Transport — any type satisfying the visr::DebugTransport concept.
//    Defaults to NullTransport so the type can be used unconditionally
//    in release builds at zero cost.
//
//  Pause semantics:
//    Pause/resume is handled HERE, not inside World.  The main loop
//    checks channel.is_paused() before calling world.step():
//
//      while (running) {
//          channel.poll(world);           // drain + dispatch commands
//          if (!channel.is_paused())
//              world.step();
//          channel.push(world);           // snapshot the post-step state
//      }
//
//  Quantity tracking:
//    CmdTrackQuantity registers a (target, id) pair.  Each push() appends
//    the current value to an internal circular buffer (tracked_series_).
//    The render thread reads these via tracked_series() to drive live graphs.
//    History depth is configurable via TRACK_HISTORY_DEPTH.
//
//  Step-once:
//    Call request_step_once() from the render thread to advance exactly
//    one frame while paused.  poll() consumes the request and returns
//    true from should_step() once, then auto-re-pauses.
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
        std::vector<Sample> samples; // ring — oldest at (head_ % size)
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
        static_assert(is_debug_transport<Transport>::value, "Transport must implement push_snapshot() and poll_command()");

        uint64_t frame_index_ = 0;
        double sim_time_ = 0.0;
        bool paused_ = false;
        bool step_once_pending_ = false;
        std::atomic<bool> step_once_requested_{false};
        std::vector<TrackedSeries> tracked_series_;


        // -----------------------------------------------------------------
        explicit DebugChannel() = default;

        explicit DebugChannel(Transport t)
            : transport(std::move(t)) {}

        // -----------------------------------------------------------------
        //  Physics-thread API
        // -----------------------------------------------------------------

        /// Push the current world state as a snapshot and sample any
        /// tracked quantities.  Call AFTER world.step() (or unconditionally).
        void push(const rbps::World &world)
        {
            auto snap = build_snapshot(world, frame_index_, sim_time_);
            sample_tracked(snap);
            transport.push_snapshot(snap);
            sim_time_ += world.timestep;
            ++frame_index_;
        }

        /// Drain the command queue and apply each command to the world.
        /// Also handles pause / step-once internally so they never reach
        /// dispatch_command().  Call BEFORE world.step().
        void poll(rbps::World &world)
        {
            // Consume the step-once token set by the render thread.
            bool do_step_once = step_once_requested_.exchange(
                false, std::memory_order_acq_rel);
            if (do_step_once && paused_)
                step_once_pending_ = true;

            Command cmd;
            while (transport.poll_command(cmd))
                dispatch_command(world, cmd);
        }

        /// Returns true when the physics loop should call world.step().
        /// Handles both the normal unpaused case and the step-once case.
        bool should_step()
        {
            if (!paused_)
                return true;

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

        void pause() { paused_ = true; }
        void resume() { paused_ = false; }
        void toggle_pause() { paused_ = !paused_; }

        /// Advance exactly one frame while paused.  No-op if not paused.
        void request_step_once()
        {
            step_once_requested_.store(true, std::memory_order_release);
        }

        bool is_paused() const { return paused_; }

        uint64_t frame_index() const { return frame_index_; }
        double sim_time() const { return sim_time_; }

        // -----------------------------------------------------------------
        //  Tracked quantities (render-thread readable after each push)
        // -----------------------------------------------------------------

        const std::vector<TrackedSeries> &tracked_series() const
        {
            return tracked_series_;
        }


        // -----------------------------------------------------------------
        void sample_tracked(const FrameSnapshot &snap)
        {
            for (auto &series : tracked_series_)
            {
                float value = 0.0f;
                switch (series.target)
                {
                case TrackTarget::BodyLinearSpeed:
                    for (auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = m3d::length(b.linear_velocity);
                    break;

                case TrackTarget::BodyAngularSpeed:
                    for (auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = m3d::length(b.angular_velocity);
                    break;

                case TrackTarget::BodyPositionX:
                    for (auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = b.position.x;
                    break;
                case TrackTarget::BodyPositionY:
                    for (auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = b.position.y;
                    break;
                case TrackTarget::BodyPositionZ:
                    for (auto &b : snap.bodies)
                        if (b.id == series.id)
                            value = b.position.z;
                    break;

                case TrackTarget::ContactNormalLambda:
                    for (auto &ct : snap.contacts)
                        if (ct.collider_a == series.id ||
                            ct.collider_b == series.id)
                            value += ct.normal_lambda;
                    break;

                case TrackTarget::ContactPenetrationDepth:
                    for (auto &ct : snap.contacts)
                        if (ct.collider_a == series.id ||
                            ct.collider_b == series.id)
                            value = std::max(value, static_cast<float>(ct.penetration_depth));
                    break;

                case TrackTarget::JointPosition:
                    for (auto &j : snap.joints)
                        if (j.id == series.id)
                            value = j.current_position;
                    break;

                case TrackTarget::ConstraintLambda:
                    for (auto &c : snap.constraints)
                        if (c.id == series.id)
                            value = c.lambda;
                    break;

                default:
                    break;
                }
                series.push(snap.frame_index, value);
            }
        }

        // Called internally when a CmdTrackQuantity is received.
        // (CommandDispatch routes these back here rather than to World.)
        void handle_track_command(const CmdTrackQuantity &cmd)
        {
            if (!cmd.enabled)
            {
                auto it = std::remove_if(tracked_series_.begin(), tracked_series_.end(),
                                         [&](const TrackedSeries &s)
                                         {
                                             return s.target == cmd.target && s.id == cmd.id;
                                         });
                tracked_series_.erase(it, tracked_series_.end());
                return;
            }

            for (auto &s : tracked_series_)
                if (s.target == cmd.target && s.id == cmd.id)
                    return; // already tracked

            TrackedSeries ts;
            ts.target = cmd.target;
            ts.id = cmd.id;
            ts.samples.reserve(TRACK_HISTORY_DEPTH);
            tracked_series_.push_back(std::move(ts));
        }
    };

} // namespace visr