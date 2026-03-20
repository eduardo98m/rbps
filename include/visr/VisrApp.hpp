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
//  Minimal usage:
//    visr::VisrApp app;
//    // populate app.world ...
//    app.run();
//
//  Keyboard shortcuts (handled in run()):
//    Space   — pause / resume
//    ESC     — deselect all   (handled in SelectionSystem)
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
                    if (channel.should_step())
                        world.step();
                    channel.push(world);
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
                // ── Space → pause / resume ────────────────────────────────
                // Checked before ImGui so it works even when a text field
                // has focus (we only act on a single press, not hold).
                if (IsKeyPressed(KEY_SPACE))
                    channel.toggle_pause();

                // ── Systems update ────────────────────────────────────────
                camera_sys.update(camera);   // non-const: speed slider writes back

                const FrameSnapshot *snap = channel.transport.latest_snapshot();
                if (snap && !ImGui::GetIO().WantCaptureMouse)
                    selection_sys.update(*snap, camera, sel, channel.transport);

                // ── Render ────────────────────────────────────────────────
                // Pass camera_sys so RenderSystem can call draw_panel() in
                // the ImGui pass without needing extra_guis.
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