#pragma once
#include <array>
#include <atomic>
#include <mutex>
#include <deque>
#include "visr/Transport.hpp"

/**
 * @file InProcessTransport.hpp
 * @brief Same-process, two-thread transport between physics and render.
 * @ingroup visr
 *
 * @par Architecture
 * @code
 * ┌──────────────────┐  atomic slot swap  ┌──────────────────────┐
 * │  Physics thread  │ ─────────────────► │  Render thread       │
 * │  push_snapshot() │   slots[0/1]       │  latest_snapshot()   │
 * │                  │ ◄───────────────── │  push_command()      │
 * └──────────────────┘  mutex+deque       └──────────────────────┘
 * @endcode
 *
 * - **Snapshot** (physics → render): ping-pong buffer of two
 *   `FrameSnapshot` slots. Physics writes to the back slot then
 *   atomically publishes it. Render reads from whichever slot is marked
 *   "ready" — never blocks. The render thread gets the LATEST complete
 *   frame, not every frame; missed frames are fine — this is a debugger,
 *   not a replay recorder.
 * - **Command** (render → physics): `std::deque` guarded by a mutex.
 *   Render pushes, physics drains once per tick. Queue is small enough
 *   (user interactions) that mutex cost is negligible; swap for a
 *   lock-free SPSC ring if throughput becomes a problem.
 *
 * @par Thread-safety contract
 * - `push_snapshot()`   — physics thread only.
 * - `latest_snapshot()` — render thread only.
 * - `push_command()`    — render thread only.
 * - `poll_command()`    — physics thread only.
 */

namespace visr
{
    /**
     * @brief Lock-free snapshot publish + mutex-guarded command queue.
     * @ingroup visr
     */
    struct InProcessTransport
    {
        std::array<FrameSnapshot, 2> slots_{}; ///< Ping-pong snapshot buffer.
        std::atomic<int> ready_slot_{-1};      ///< Index of the readable slot, or `-1` before the first publish.
        int write_slot_{0};                    ///< Slot the physics thread is currently writing to.

        mutable std::mutex cmd_mutex_;         ///< Guards `cmd_queue_`.
        std::deque<Command> cmd_queue_;        ///< Pending commands (FIFO).

        // ── Physics-thread API ────────────────────────────────────────────

        /**
         * @brief Publish a new frame.
         *
         * Copies into the back slot, atomically advertises it, then flips
         * `write_slot_` so the next call writes to the other slot.
         */
        void push_snapshot(const FrameSnapshot &snap)
        {
            slots_[write_slot_] = snap;
            ready_slot_.store(write_slot_, std::memory_order_release);
            write_slot_ ^= 1; // flip: 0→1, 1→0
        }

        /**
         * @brief Drain one command from the queue.
         * @return `false` when the queue is empty; otherwise fills `out` and returns `true`.
         */
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

        /**
         * @brief Read the latest published snapshot. Never blocks.
         * @return Pointer to the readable slot, or `nullptr` if no frame
         *         has been published yet.
         */
        const FrameSnapshot *latest_snapshot() const
        {
            int r = ready_slot_.load(std::memory_order_acquire);
            if (r < 0)
                return nullptr; // nothing published yet
            return &slots_[r];
        }

        /** @brief Push a command from the render thread toward the physics thread. */
        void push_command(Command cmd)
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            cmd_queue_.push_back(std::move(cmd));
        }

        /** @brief Informational: how many commands are queued. Safe from any thread. */
        size_t pending_command_count() const
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            return cmd_queue_.size();
        }
    };

    static_assert(is_debug_transport<InProcessTransport>::value,
            "InProcessTransport must satisfy the DebugTransport contract");

} // namespace visr