#pragma once
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>
#include <functional>
#include "visr/Snapshot.hpp"
#include "visr/ui/RaylibDraw.hpp"
#include "visr/ui/Panels.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

// ============================================================================
//  visr/systems/RenderSystem.hpp
//
//  Owns the render pass: 3-D scene + ImGui panels in the right order.
//  Mirrors the compact RenderingSystem pattern from the reference code:
//
//    BeginDrawing()
//      ClearBackground
//      BeginMode3D  →  draw scene
//      EndMode3D
//      rlImGuiBegin →  draw panels + user GUIs
//      rlImGuiEnd
//    EndDrawing()
//
//  Extra user GUIs are injected via a list of void() lambdas so callers
//  can add demo-specific panels without subclassing.
// ============================================================================

namespace visr
{
    struct RenderSystem
    {
        draw::DrawFlags flags{};

        // Call between InitWindow and the main loop.
        void init()
        {
            rlImGuiSetup(true);
        }

        // Call once per frame from the render thread.
        template<typename Transport>
        void update(DebugChannel<Transport> &channel,
                    InProcessTransport      &transport,
                    const FrameSnapshot     *snap,          // may be nullptr
                    const Camera3D          &camera,
                    ui::SelectionState      &sel,
                    const std::vector<std::function<void()>> &extra_guis = {})
        {
            BeginDrawing();
            ClearBackground({ 30, 30, 35, 255 });

            // ── 3-D pass ──────────────────────────────────────────────────
            BeginMode3D(camera);
            DrawGrid(20, 1.0f);
            if (snap)
                draw::draw_scene(*snap, flags, sel.body_id);
            EndMode3D();

            // ── ImGui pass ────────────────────────────────────────────────
            rlImGuiBegin();
            if (snap)
                ui::draw_all(channel, transport, *snap, sel);

            for (auto &gui : extra_guis)
                gui();

            rlImGuiEnd();

            DrawFPS(10, 10);
            EndDrawing();
        }

        void shutdown()
        {
            rlImGuiShutdown();
        }
    };

} // namespace visr