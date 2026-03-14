#pragma once
// ============================================================================
//  visr/VisrApp.hpp
//
//  Reference main-loop showing Option B (in-process, two-thread).
//  Copy and adapt; this is not a required base class.
//
//  Physics thread:   owns World + DebugChannel,  runs step loop
//  Render  thread:   owns Raylib window + ImGui,  reads snapshots
//
//  The two threads share only InProcessTransport (embedded in the channel).
//  Every other object is thread-local.
//
//  Usage:
//    visr::VisrApp app("My Sim", 1280, 720);
//    // populate app.world ...
//    app.run();    // blocks until window is closed
// ============================================================================

#include <thread>
#include <atomic>
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>             // rlImGui bridging header (github.com/raylib-extras/rlImGui)

#include "rbps/API/World.hpp"
#include "visr/DebugChannel.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/ui/Panels.hpp"
#include "visr/ui/RaylibDraw.hpp"

namespace visr
{
    struct VisrApp
    {
        rbps::World world;
        DebugChannel<InProcessTransport> channel;

        const char *title;
        int screen_w, screen_h;

        explicit VisrApp(const char *title_ = "rbps visualizer",
                         int w = 1280, int h = 720)
            : title(title_), screen_w(w), screen_h(h)
        {}

        // -----------------------------------------------------------------
        void run()
        {
            std::atomic<bool> running{true};

            // ── Physics thread ────────────────────────────────────────────
            std::thread phys_thread([this, &running]()
            {
                while (running.load(std::memory_order_relaxed))
                {
                    channel.poll(world);        // drain commands first

                    if (channel.should_step())
                        world.step();

                    channel.push(world);        // snapshot post-step state

                    // Optionally yield / sleep to cap physics rate.
                    // std::this_thread::sleep_for(std::chrono::microseconds(100));
                }
            });

            // ── Render thread (main thread) ───────────────────────────────
            InitWindow(screen_w, screen_h, title);
            SetTargetFPS(60);
            rlImGuiSetup(true);     // initialise Dear ImGui for raylib

            Camera3D camera{};
            camera.position   = {5, 5, 10};
            camera.target     = {0, 0, 0};
            camera.up         = {0, 1, 0};
            camera.fovy       = 45.0f;
            camera.projection = CAMERA_PERSPECTIVE;

            ui::SelectionState sel{};
            draw::DrawFlags      draw_flags{};

            while (!WindowShouldClose())
            {
                UpdateCamera(&camera, CAMERA_ORBITAL);

                // Get the latest snapshot (may be nullptr for the first few frames).
                const FrameSnapshot *snap =
                    channel.transport.latest_snapshot();

                BeginDrawing();
                ClearBackground(DARKGRAY);

                BeginMode3D(camera);
                if (snap)
                    draw::draw_scene(*snap, draw_flags, sel.body_id);
                EndMode3D();

                // ImGui
                rlImGuiBegin();
                if (snap)
                    ui::draw_all(channel, channel.transport, *snap, sel);
                rlImGuiEnd();

                DrawFPS(10, 10);
                EndDrawing();
            }

            running.store(false, std::memory_order_relaxed);
            rlImGuiShutdown();
            CloseWindow();

            phys_thread.join();
        }
    };

} // namespace visr