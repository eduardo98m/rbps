#pragma once
#include <raylib.h>
#include <imgui.h>
#include <rlImGui.h>
#include <implot.h>
#include <functional>
#include <ctime>
#include <cstdio>
#include <cstring>
#include "visr/Snapshot.hpp"
#include "visr/ui/RaylibDraw.hpp"
#include "visr/ui/Panels.hpp"
#include "visr/systems/CameraSystem.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

/**
 * @file RenderSystem.hpp
 * @brief Per-frame raylib + ImGui + ImPlot pass.
 * @ingroup visr
 *
 * @par Frame flow
 * @code
 *  BeginDrawing()
 *    ClearBackground
 *    BeginMode3D  →  draw_scene
 *    EndMode3D
 *    rlImGuiBegin
 *      camera_sys.draw_panel()
 *      ui::draw_all(...)   ← may set ui::g_plot_export_request.pending
 *    rlImGuiEnd
 *  EndDrawing()            ← framebuffer is complete
 *
 *  [if export pending]
 *    LoadImageFromScreen() ← reads complete framebuffer
 *    ImageCrop()           ← crop to saved plot screen-rect
 *    ImageResize()         ← optional upscale
 *    ExportImage()         ← write PNG
 *    ui::g_export_status   ← set result string for next-frame toast
 * @endcode
 *
 * @par Why crop after EndDrawing()?
 * `LoadImageFromScreen` reads the GL read buffer, which is only fully
 * composited after `EndDrawing` (which calls `SwapBuffers` on most
 * platforms). Reading earlier captures a partial frame missing the
 * ImGui overlay. Same-frame, post-`EndDrawing` is the cleanest window.
 */

namespace visr
{
    /**
     * @brief Owns the per-frame render pass and the post-frame plot-export hook.
     * @ingroup visr
     */
    struct RenderSystem
    {
        draw::DrawFlags flags{};                     ///< Flags controlling which overlays the scene draws.
        Color bg_color = { 30, 30, 35, 255 };        ///< Background clear colour.

        /** @brief Set up rlImGui + ImPlot contexts. Call after `InitWindow`. */
        void init()
        {
            rlImGuiSetup(true);
            ImPlot::CreateContext();   // must follow rlImGuiSetup (needs ImGui context)
        }

        /**
         * @brief Render one frame: 3-D scene → ImGui overlay → optional plot export.
         *
         * @tparam Transport Any type satisfying `is_debug_transport`.
         */
        template<typename Transport>
        void update(DebugChannel<Transport> &channel,
                    InProcessTransport      &transport,
                    const FrameSnapshot     *snap,
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
                                 sel.body_id, sel.contact_idx, sel.joint_id);
            EndMode3D();

            // ── ImGui + ImPlot pass ───────────────────────────────────────
            rlImGuiBegin();
            camera_sys.draw_panel();
            if (snap)
                ui::draw_all(channel, transport, *snap, sel);
            for (auto &gui : extra_guis) gui();
            rlImGuiEnd();

            DrawFPS(10, 10);
            EndDrawing();   // ← framebuffer complete after this line

            // ── Plot export (post-frame) ───────────────────────────────────
            // Process after EndDrawing so the full composited frame is
            // available in the GL read buffer.
            _process_export();
        }

        /** @brief Tear down rlImGui + ImPlot contexts. Call before `CloseWindow`. */
        void shutdown()
        {
            ImPlot::DestroyContext();
            rlImGuiShutdown();
        }

    private:

        void _process_export()
        {
            auto &req = ui::g_plot_export_request;
            if (!req.pending) return;
            req.pending = false;

            // Build filename (append .png if missing)
            char fname[160];
            const char *name = req.filename[0] ? req.filename : "visr_plot";
            if (!std::strstr(name, ".png"))
                std::snprintf(fname, sizeof(fname), "%s.png", name);
            else
                std::strncpy(fname, name, sizeof(fname) - 1);

            // Capture full framebuffer
            Image full = LoadImageFromScreen();

            // Crop to the saved plot screen-rect.
            // ImPlot returns screen coords in ImGui space (points); on HiDPI
            // screens these may need to be scaled by GetIO().DisplayFramebufferScale.
            // We use floor/ceil to stay within valid pixel bounds.
            const ImGuiIO &io    = ImGui::GetIO();
            const float    scale = io.DisplayFramebufferScale.x; // usually 1 or 2

            Rectangle crop;
            crop.x      = std::floor(req.pos.x  * scale);
            crop.y      = std::floor(req.pos.y  * scale);
            crop.width  = std::ceil (req.size.x * scale);
            crop.height = std::ceil (req.size.y * scale);

            // Clamp to image bounds
            if (crop.x < 0) crop.x = 0;
            if (crop.y < 0) crop.y = 0;
            if (crop.x + crop.width  > full.width)  crop.width  = full.width  - crop.x;
            if (crop.y + crop.height > full.height)  crop.height = full.height - crop.y;

            ImageCrop(&full, crop);

            // Optional upscale (nearest-neighbour is fine for a plot)
            if (req.upscale > 1)
                ImageResize(&full,
                            full.width  * req.upscale,
                            full.height * req.upscale);

            const bool ok = ExportImage(full, fname);
            UnloadImage(full);

            ui::g_export_status      = ok ? (std::string("Saved: ") + fname) : "Export FAILED";
            ui::g_export_status_time = GetTime();
        }
    };

} // namespace visr