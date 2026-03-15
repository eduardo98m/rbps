#pragma once
#include <thread>
#include <atomic>
#include <functional>
#include <vector>
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>

// PI guard must come before any math3d header
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
//  Physics thread and render thread are cleanly separated; systems are
//  modular structs you can configure before calling run().
//
//  Minimal usage:
//    visr::VisrApp app;
//    // populate app.world ...
//    app.run();
//
//  With extra ImGui panels:
//    app.extra_guis.push_back([&](){ ImGui::Begin("Mine"); ...; ImGui::End(); });
//    app.run();
// ============================================================================

namespace visr
{
    struct VisrApp
    {
        // ── Scene & channel ───────────────────────────────────────────────
        rbps::World                      world;
        DebugChannel<InProcessTransport> channel;

        // ── Systems ───────────────────────────────────────────────────────
        CameraSystem    camera_sys;
        SelectionSystem selection_sys;
        RenderSystem    render_sys;

        // ── Window config ─────────────────────────────────────────────────
        const char *title    = "rbps visualizer";
        int         screen_w = 1440;
        int         screen_h = 900;

        // ── Inject extra ImGui panels ─────────────────────────────────────
        std::vector<std::function<void()>> extra_guis;

        // -----------------------------------------------------------------
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

            // ── Render thread (must be main thread on macOS / Windows) ────
            InitWindow(screen_w, screen_h, title);
            SetTargetFPS(60);
            render_sys.init();

            Camera3D camera = CameraSystem::make_default();
            ui::SelectionState sel{};

            while (!WindowShouldClose())
            {
                // Systems update
                camera_sys.update(camera);
                const FrameSnapshot *snap = channel.transport.latest_snapshot();
                if (snap && !ImGui::GetIO().WantCaptureMouse)
                    selection_sys.update(*snap, camera, sel, channel.transport);

                // Render
                render_sys.update(channel, channel.transport, snap,
                                  camera, sel, extra_guis);
            }

            running.store(false, std::memory_order_relaxed);
            render_sys.shutdown();
            CloseWindow();
            phys.join();
        }
    };

} // namespace visr