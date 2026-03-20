#pragma once
#include <thread>
#include <atomic>
#include <functional>
#include <vector>
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>

#ifdef PI
#  undef PI
#endif

#include "rbps/API/World.hpp"
#include "visr/DebugChannel.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/systems/CameraSystem.hpp"
#include "visr/systems/SelectionSystem.hpp"
#include "visr/systems/RenderSystem.hpp"

// ============================================================================
//  visr/VisrApp.hpp
//
//  Wires the three systems together into a runnable app.
//
//  Physics loop (v3 change):
//    The result of channel.should_step() is stored in a local bool `stepped`
//    and forwarded to channel.push(world, stepped).
//
//    This is required for two reasons:
//      1. should_step() has a side effect — it consumes a pending step-once
//         token (sets step_once_pending_=false).  Calling it twice would lose
//         the token.
//      2. push() uses the flag to decide whether to call sample_tracked().
//         When stepped=false (paused) the render snapshot is still published
//         but graph samples are not appended, so the x-axis stays still.
//
//  Keyboard shortcuts:
//    Space — pause / resume
//    ESC   — deselect all  (handled in SelectionSystem)
// ============================================================================

namespace visr
{
    struct VisrApp
    {
        rbps::World                      world;
        DebugChannel<InProcessTransport> channel;

        CameraSystem    camera_sys;
        SelectionSystem selection_sys;
        RenderSystem    render_sys;

        const char *title    = "rbps visualizer";
        int         screen_w = 1440;
        int         screen_h = 900;

        std::vector<std::function<void()>> extra_guis;

        void run()
        {
            std::atomic<bool> running{true};

            // ── Physics thread ────────────────────────────────────────────
            std::thread phys([this, &running]()
            {
                while (running.load(std::memory_order_relaxed))
                {
                    channel.poll(world);

                    // Store the result — should_step() consumes a step-once
                    // token so it must only be called once per tick.
                    const bool stepped = channel.should_step();
                    if (stepped)
                        world.step();

                    // push(world, stepped):
                    //   stepped=true  → publish snapshot + advance sim_time
                    //                   + record graph samples
                    //   stepped=false → publish snapshot only (render stays
                    //                   live for inspection, graphs flatline)
                    channel.push(world, stepped);
                }
            });

            // ── Render thread (main thread on macOS / Windows) ────────────
            InitWindow(screen_w, screen_h, title);
            SetTargetFPS(60);
            render_sys.init();

            Camera3D camera = CameraSystem::make_default();
            ui::SelectionState sel{};

            while (!WindowShouldClose())
            {
                // Space → pause / resume
                if (IsKeyPressed(KEY_SPACE))
                    channel.toggle_pause();

                camera_sys.update(camera);

                const FrameSnapshot *snap = channel.transport.latest_snapshot();
                if (snap && !ImGui::GetIO().WantCaptureMouse)
                    selection_sys.update(*snap, camera, sel, channel.transport);

                render_sys.update(channel, channel.transport, snap,
                                  camera, sel, camera_sys, extra_guis);
            }

            running.store(false, std::memory_order_relaxed);
            render_sys.shutdown();
            CloseWindow();
            phys.join();
        }
    };

} // namespace visr