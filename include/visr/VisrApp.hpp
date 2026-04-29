#pragma once
#include <thread>
#include <atomic>
#include <functional>
#include <vector>
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>

#ifdef PI
#undef PI
#endif

#include "rbps/API/World.hpp"
#include "visr/DebugChannel.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/systems/CameraSystem.hpp"
#include "visr/systems/SelectionSystem.hpp"
#include "visr/systems/RenderSystem.hpp"

/**
 * @defgroup visr visr — Visualizer
 * @brief Optional raylib + ImGui visualizer for the RBPS engine.
 *
 * Built only when CMake is configured with `-DRBPS_BUILD_VISR=ON`. The
 * visualizer runs a producer/consumer split:
 * - **Physics thread** owns the `World`, runs `step()`, and publishes a
 *   `FrameSnapshot` (POD copy of the simulation state) to the transport.
 * - **Render thread** consumes the latest snapshot, runs the camera /
 *   selection / render systems, and pushes user-input `Command`s back
 *   through the transport for the physics thread to apply.
 *
 * The transport is templated so the visualizer can be wired to a network
 * link or shared-memory channel just as easily as the default
 * `InProcessTransport`. `VisrApp` ties everything together — most users
 * just instantiate it, call `run()`, and never touch the lower layers.
 */

/**
 * @file VisrApp.hpp
 * @brief Top-level visualizer app: owns the World, thread, and render systems.
 * @ingroup visr
 *
 * @par Physics loop note
 * `channel.should_step()` has a side effect — it consumes a pending
 * step-once token. Cache the result before calling
 * `channel.push(world, stepped)`, which uses the flag to decide whether
 * to advance the graph x-axis.
 *
 * @par Keyboard shortcuts
 * - `Space` — pause / resume.
 * - `ESC`   — deselect all (handled in `SelectionSystem`).
 */

namespace visr
{
    /**
     * @brief Runnable visualizer app — owns a `World`, the systems, and the threads.
     *
     * The default constructor produces an empty world. Populate it the
     * normal way (`world.create_body`, `world.create_collider`, …) before
     * calling `run()`. Custom panels can be appended to `extra_guis`;
     * each entry is invoked once per render frame inside `rlImGuiBegin / End`.
     *
     * @ingroup visr
     */
    struct VisrApp
    {
        rbps::World world;                          ///< Physics world the user populates before `run()`.
        DebugChannel<InProcessTransport> channel;   ///< Snapshot/command channel between physics and render threads.

        CameraSystem camera_sys;       ///< WASD + RMB-drag free-look camera.
        SelectionSystem selection_sys; ///< LMB ray-pick + ESC deselect.
        RenderSystem render_sys;       ///< Per-frame raylib + ImGui pass.

        const char *title = "rbps visualizer"; ///< Window title.
        int screen_w = 1440;                   ///< Window width in pixels.
        int screen_h = 900;                    ///< Window height in pixels.

        /**
         * @brief Optional ImGui panel callbacks invoked each frame.
         *
         * Each entry runs inside `rlImGuiBegin / rlImGuiEnd`, so it can
         * issue any `ImGui::*` call directly. Use to add domain-specific
         * panels without modifying the core `RenderSystem`.
         */
        std::vector<std::function<void()>> extra_guis;

        /**
         * @brief Run the visualizer until the window is closed.
         *
         * Spawns a physics thread, opens the raylib window, and pumps the
         * render loop on the calling thread. Returns when the user closes
         * the window.
         */
        void run()
        {
            std::atomic<bool> running{true};

            // ── Physics thread ────────────────────────────────────────────
            // std::thread phys([this, &running]()
            // {
            //     while (running.load(std::memory_order_relaxed))
            //     {
            //         channel.poll(world);

            //         // Store the result — should_step() consumes a step-once
            //         // token so it must only be called once per tick.
            //         const bool stepped = channel.should_step();
            //         if (stepped)
            //             world.step();

            //         // push(world, stepped):
            //         //   stepped=true  → publish snapshot + advance sim_time
            //         //                   + record graph samples
            //         //   stepped=false → publish snapshot only (render stays
            //         //                   live for inspection, graphs flatline)
            //         channel.push(world, stepped);
            //     }
            // });

            std::thread phys([this, &running]()
                             {
                using namespace std::chrono;
                
                // Track the time of the previous loop iteration
                auto prev_time = high_resolution_clock::now();
                double accumulator = 0.0;
                
                // We get the fixed timestep you set in main.cpp
                const double dt = world.timestep; 

                while (running.load(std::memory_order_relaxed))
                {
                    auto current_time = high_resolution_clock::now();
                    duration<double> elapsed = current_time - prev_time;
                    prev_time = current_time;
                    
                    accumulator += elapsed.count();
                    
                    // Only step the physics when we have accumulated enough real time
                    while (accumulator >= dt)
                    {
                        channel.poll(world);

                        const bool stepped = channel.should_step();
                        if (stepped)
                        {
                            world.step();
                        }

                        channel.push(world, stepped);
                        
                        // Consume the time we just simulated
                        accumulator -= dt;
                    }
                    
                    // Optional: yield the thread briefly to prevent 100% CPU usage
                    std::this_thread::yield(); 
            } });

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