#pragma once
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>
#include <implot.h>
#include <functional>
#include <ctime>
#include <cstdio>
#include "visr/Snapshot.hpp"
#include "visr/ui/RaylibDraw.hpp"
#include "visr/ui/Panels.hpp"
#include "visr/systems/CameraSystem.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

// ============================================================================
//  visr/systems/RenderSystem.hpp
//
//  Owns the full render pass including ImGui + ImPlot.
//
//  Frame flow:
//    BeginDrawing()
//      ClearBackground
//      BeginMode3D  →  draw_scene
//      EndMode3D
//      rlImGuiBegin
//        camera_sys.draw_panel()
//        ui::draw_all(...)       ← sets visr::ui::g_screenshot_requested
//      rlImGuiEnd
//    EndDrawing()                ← framebuffer is now complete
//    [if screenshot requested]   ← LoadImageFromScreen → ExportImage
//
//  Screenshot timing:
//    LoadImageFromScreen() reads the OpenGL read buffer.  Calling it
//    immediately after EndDrawing() (before the next BeginDrawing) is
//    the correct window — the full composited frame is in the buffer.
// ============================================================================

namespace visr
{
    struct RenderSystem
    {
        draw::DrawFlags flags{};
        Color           bg_color = { 30, 30, 35, 255 };

        // Last export status shown as a timed toast in the graph panel.
        // Written here (by RenderSystem), read in Panels via g_export_status.
        std::string last_export_msg;
        double      last_export_time = -10.0;  // GetTime() value

        void init()
        {
            rlImGuiSetup(true);
            ImPlot::CreateContext();   // must come AFTER rlImGuiSetup (ImGui context exists)
        }

        template<typename Transport>
        void update(DebugChannel<Transport> &channel,
                    InProcessTransport      &transport,
                    const FrameSnapshot     *snap,
                    const Camera3D          &camera,
                    ui::SelectionState      &sel,
                    CameraSystem            &camera_sys,
                    const std::vector<std::function<void()>> &extra_guis = {})
        {
            // Reset the export-request flag every frame; the panel will set it
            // if the user pressed "Save PNG" this frame.
            ui::g_screenshot_requested = false;

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

            // ── ImGui + ImPlot pass ───────────────────────────────────────
            rlImGuiBegin();

            camera_sys.draw_panel();

            if (snap)
                ui::draw_all(channel, transport, *snap, sel);

            for (auto &gui : extra_guis)
                gui();

            rlImGuiEnd();

            DrawFPS(10, 10);
            EndDrawing();

            // ── Post-frame screenshot ─────────────────────────────────────
            // The framebuffer is complete after EndDrawing().
            // LoadImageFromScreen() reads the current GL read buffer — this is
            // the correct place to call it.
            if (ui::g_screenshot_requested)
            {
                char fname[80];
                const time_t now = time(nullptr);
                strftime(fname, sizeof(fname),
                         "visr_export_%Y%m%d_%H%M%S.png", localtime(&now));

                Image img = LoadImageFromScreen();
                const bool ok = ExportImage(img, fname);
                UnloadImage(img);

                if (ok)
                {
                    ui::g_export_status = std::string("Saved: ") + fname;
                    last_export_msg  = ui::g_export_status;
                }
                else
                {
                    ui::g_export_status = "Export FAILED";
                }
                last_export_time = GetTime();
                ui::g_export_status_time = last_export_time;
            }
        }

        void shutdown()
        {
            ImPlot::DestroyContext();
            rlImGuiShutdown();
        }
    };

} // namespace visr