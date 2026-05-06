// Collision Debugger — synchronous, two-shape narrowphase visualizer.
//
// Runs no physics. Every render frame: construct two rbc::Shape values
// from the GUI state, run GJK -> EPA -> manifold-generation against
// each other while capturing every intermediate, then render the
// pipeline state with toggleable layers.

#include <raylib.h>
#include <rlImGui.h>
#include <imgui.h>

#ifdef PI
#  undef PI
#endif

#include "visr/systems/CameraSystem.hpp"

#include "State.hpp"
#include "ShapeFactory.hpp"
#include "PipelineRun.hpp"
#include "PipelineDraw.hpp"
#include "Panels.hpp"

int main()
{
    constexpr int screen_w = 1440;
    constexpr int screen_h = 900;

    InitWindow(screen_w, screen_h, "Collision Debugger");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    visr::CameraSystem camera_sys;
    Camera3D camera = visr::CameraSystem::make_default();

    cdbg::DebuggerState state;
    // Sensible starting layout: two boxes, slightly overlapping along Y.
    state.params_a.kind   = cdbg::Kind_Box;
    state.params_b.kind   = cdbg::Kind_Box;
    state.pose_a.position = m3d::vec3{0.0, 0.0, 0.0};
    state.pose_b.position = m3d::vec3{0.0, 0.95, 0.0};

    cdbg::PipelineResult result;

    while (!WindowShouldClose())
    {
        // ── Camera (skip when ImGui has the mouse) ─────────────────────
        if (!ImGui::GetIO().WantCaptureMouse &&
            !ImGui::GetIO().WantCaptureKeyboard)
            camera_sys.update(camera);

        // ── Build shapes + transforms from current state ───────────────
        const rbc::Shape shape_a = cdbg::make_shape(state.params_a);
        const rbc::Shape shape_b = cdbg::make_shape(state.params_b);
        const m3d::tf    tf_a    = cdbg::pose_to_tf(state.pose_a);
        const m3d::tf    tf_b    = cdbg::pose_to_tf(state.pose_b);

        // ── Run pipeline (live, or on demand) ──────────────────────────
        if (state.live_recompute || state.recompute_now)
        {
            cdbg::run_pipeline(shape_a, tf_a, shape_b, tf_b, result);
            state.recompute_now = false;
        }

        // ── Render ─────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground(Color{18, 18, 22, 255});

        BeginMode3D(camera);
        cdbg::draw_pipeline(shape_a, tf_a, shape_b, tf_b, result, state.viz);
        EndMode3D();

        rlImGuiBegin();
        cdbg::draw_inputs_panel(state);
        cdbg::draw_viz_panel(state.viz);
        cdbg::draw_readout_panel(result);
        camera_sys.draw_panel();
        rlImGuiEnd();

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
