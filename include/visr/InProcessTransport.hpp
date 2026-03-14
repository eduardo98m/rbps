#pragma once
#include <array>
#include <atomic>
#include <mutex>
#include <deque>
#include "visr/Transport.hpp"

// ============================================================================
//  visr/InProcessTransport.hpp
//
//  Same-process, two-thread transport (Option B from the design doc).
//
//  ┌──────────────────┐  atomic slot swap  ┌──────────────────────┐
//  │  Physics thread  │ ─────────────────► │  Render thread       │
//  │  push_snapshot() │   slots[0/1]       │  latest_snapshot()   │
//  │                  │ ◄───────────────── │  push_command()      │
//  └──────────────────┘  mutex+deque       └──────────────────────┘
//
//  Snapshot side (physics → render):
//    - Two FrameSnapshot slots (ping-pong buffer).
//    - Physics writes to the "back" slot, then atomically publishes it.
//    - Render reads from whichever slot is marked "ready" — never blocks.
//    - The render thread gets the LATEST complete frame, not every frame.
//      Missed frames are fine; we're a debugger, not a replay recorder.
//
//  Command side (render → physics):
//    - std::deque guarded by a std::mutex.
//    - Render thread pushes; physics thread pops once per frame.
//    - Command queue is small (user interactions) so mutex cost is negligible.
//    - For higher throughput, swap for a lock-free SPSC ring.
//
//  Thread safety contract:
//    push_snapshot()  — call from physics thread only
//    latest_snapshot() — call from render thread only
//    push_command()  — call from render thread only
//    poll_command()  — call from physics thread only
// ============================================================================

namespace visr
{
    struct InProcessTransport
    {
        // Snapshot double-buffer
        std::array<FrameSnapshot, 2> slots_{};
        std::atomic<int> ready_slot_{-1}; // -1 = nothing yet
        int write_slot_{0};

        // Command queue
        mutable std::mutex cmd_mutex_;
        std::deque<Command> cmd_queue_;

        // ── Physics-thread API ────────────────────────────────────────────

        /// Publish a new frame.  Swaps the write slot and makes it visible
        /// to the render thread in one atomic store.
        void push_snapshot(const FrameSnapshot &snap)
        {
            slots_[write_slot_] = snap;
            ready_slot_.store(write_slot_, std::memory_order_release);
            write_slot_ ^= 1; // flip: 0→1, 1→0
        }

        /// Drain one command from the queue.  Returns false when empty.
        bool poll_command(Command &out)
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            if (cmd_queue_.empty())
                return false;
            out = cmd_queue_.front();
            cmd_queue_.pop_front();
            return true;
        }

        // ── Render-thread API ─────────────────────────────────────────────

        /// Read the latest published snapshot.  Never blocks.
        /// Returns nullptr if no frame has been pushed yet.
        const FrameSnapshot *latest_snapshot() const
        {
            int r = ready_slot_.load(std::memory_order_acquire);
            if (r < 0)
                return nullptr; // nothing published yet
            return &slots_[r];
        }

        /// Push a command from the render thread toward the physics thread.
        void push_command(Command cmd)
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            cmd_queue_.push_back(std::move(cmd));
        }

        /// How many commands are queued (informational, render thread only).
        size_t pending_command_count() const
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            return cmd_queue_.size();
        }
    };

    static_assert(is_debug_transport<InProcessTransport>::value,
            "InProcessTransport must satisfy the DebugTransport contract");

} // namespace visr