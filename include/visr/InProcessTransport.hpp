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
 * │  push_snapshot() │   slots[0/1/2]     │  latest_snapshot()   │
 * │                  │ ◄───────────────── │  push_command()      │
 * └──────────────────┘  mutex+deque       └──────────────────────┘
 * @endcode
 *
 * - **Snapshot** (physics → render): triple buffer of three `FrameSnapshot`
 *   slots. At any moment each slot has exactly one owner:
 *   - `write_slot_`     — physics is writing into this slot.
 *   - `render_slot_`    — render is reading from this slot for the whole frame.
 *   - `published_slot_` — the handoff slot: physics just finished writing it,
 *                         render has not yet claimed it.
 *
 *   Physics writes into `write_slot_`, then atomically swaps it into
 *   `published_slot_`, recovering the previously published slot (now
 *   abandoned by render) as its next write target. Render calls
 *   `latest_snapshot()` once per frame: it atomically claims `published_slot_`
 *   (if one exists) as its new `render_slot_`, implicitly releasing the old
 *   one back into the pool. The render thread then holds `render_slot_`
 *   exclusively for the entire frame — physics can never touch it until
 *   the next `latest_snapshot()` call hands it back. Missed frames are
 *   fine — this is a debugger, not a replay recorder.
 *
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
        std::array<FrameSnapshot, 3> slots_{}; ///< Triple buffer.
        std::atomic<int> published_slot_{-1};  ///< Index of the readable slot, or `-1` before the first publish.
        int write_slot_{0};                    ///< Slot the physics thread is currently writing to.
        int render_slot_{-1};                  ///< Index of the slot the render thread is currently reading from; used for sanity checks.

        mutable std::mutex cmd_mutex_;  ///< Guards `cmd_queue_`.
        std::deque<Command> cmd_queue_; ///< Pending commands (FIFO).

        // ── Physics-thread API ────────────────────────────────────────────

        /**
         * @brief Publish a completed frame to the render thread.
         *
         * Writes `snap` into `write_slot_`, then atomically swaps it into
         * `published_slot_`, recovering whichever slot the render thread just
         * abandoned as the next write target. If no slot has been published yet
         * (first call), the remaining unused slot is located by elimination and
         * adopted as the next write target instead.
         *
         * After this call the three slots have distinct owners:
         * - `write_slot_`     — physics (ready for the next frame).
         * - `published_slot_` — the handoff slot (render will claim it next frame).
         * - `render_slot_`    — render (untouched by physics until handed back).
         */
        void push_snapshot(const FrameSnapshot &snap)
        {
            slots_[write_slot_] = snap;
            // Publish write_slot_, get back whatever slot render just released
            int prev = published_slot_.exchange(write_slot_, std::memory_order_acq_rel);
            // prev is now safe for physics to write into next:
            //   - if -1 (first publish, or render hasn't consumed yet), use the
            //     remaining unused slot (the one that is neither write nor render)
            //   - otherwise prev is the slot render just handed back
            if (prev < 0)
            {
                // Find the slot that is neither write_slot_ nor render_slot_
                for (int i = 0; i < 3; ++i)
                    if (i != write_slot_ && i != render_slot_)
                    {
                        write_slot_ = i;
                        break;
                    }
            }
            else
            {
                write_slot_ = prev;
            }
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
         * @brief Claim the latest published snapshot for this frame. Never blocks.
         *
         * Atomically takes ownership of `published_slot_`, adopting it as the new
         * `render_slot_` and implicitly releasing the old one back into the pool
         * for physics to reclaim. If no new frame has been published since the
         * last call, `render_slot_` is kept as-is — the render thread continues
         * reading the previous frame rather than stalling.
         *
         * Must be called exactly once per render frame and never mid-frame.
         * The returned pointer is valid and exclusively owned by the render thread
         * until the next call to this function.
         *
         * @return Pointer to the render-owned slot, or `nullptr` if no frame
         *         has been published yet.
         */
        const FrameSnapshot *latest_snapshot()
        {
            int published = published_slot_.exchange(-1, std::memory_order_acq_rel);
            if (published >= 0)
                render_slot_ = published; // take the new one, release the old back into the pool

            return (render_slot_ >= 0) ? &slots_[render_slot_] : nullptr;
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