#pragma once
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>
#include <functional>
#include "visr/Snapshot.hpp"
#include "visr/ui/RaylibDraw.hpp"
#include "visr/ui/Panels.hpp"
#include "visr/systems/CameraSystem.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

// ============================================================================
//  visr/systems/RenderSystem.hpp
//
//  Owns the render pass: 3-D scene + ImGui panels.
//
//  Frame flow:
//    BeginDrawing()
//      ClearBackground
//      BeginMode3D  →  draw_scene (colliders, contacts, joints, axes)
//      EndMode3D
//      rlImGuiBegin
//        camera_sys.draw_panel()        ← speed / sensitivity sliders
//        ui::draw_all(...)              ← sim control, bodies, contacts…
//        extra user GUIs
//      rlImGuiEnd
//    EndDrawing()
//
//  update() takes CameraSystem& so it can call draw_panel() inside the
//  ImGui pass without requiring the caller to push it through extra_guis.
// ============================================================================

namespace visr
{
    struct RenderSystem
    {
        draw::DrawFlags flags{};
        Color           bg_color = { 30, 30, 35, 255 };

        void init() { rlImGuiSetup(true); }

        template<typename Transport>
        void update(DebugChannel<Transport> &channel,
                    InProcessTransport      &transport,
                    const FrameSnapshot     *snap,      // may be nullptr on first frame
                    const Camera3D          &camera,
                    ui::SelectionState      &sel,
                    CameraSystem            &camera_sys,
                    const std::vector<std::function<void()>> &extra_guis = {})
        {
            BeginDrawing();
            ClearBackground(bg_color);

            // ── 3-D pass ──────────────────────────────────────────────────
            BeginMode3D(camera);
            DrawGrid(20, 1.0f);
            if (snap)
                draw::draw_scene(*snap, flags,
                                 sel.body_id,
                                 sel.contact_idx,
                                 sel.joint_id);
            EndMode3D();

            // ── ImGui pass ────────────────────────────────────────────────
            rlImGuiBegin();

            camera_sys.draw_panel();       // speed & sensitivity

            if (snap)
                ui::draw_all(channel, transport, *snap, sel);

            for (auto &gui : extra_guis)
                gui();

            rlImGuiEnd();

            DrawFPS(10, 10);
            EndDrawing();
        }

        void shutdown() { rlImGuiShutdown(); }
    };

} // namespace visr