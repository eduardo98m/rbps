// Collision Debugger — synchronous, two-shape narrowphase visualizer.
//
// Runs no physics. Every render frame: construct two rbc::Shape values
// from the GUI state, run GJK -> EPA -> manifold-generation against
// each other while capturing every intermediate, then render the
// pipeline state with toggleable layers.
//
// Crash recovery: before each call to run_pipeline, if the inputs have
// changed we write `scenarios/_last_input.scn` and fflush. On clean
// shutdown we touch `scenarios/_last_input.ok`. If on startup the .scn
// exists without the .ok marker, the previous run died mid-pipeline —
// the scenarios panel offers to replay the trigger.

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
#include "Scenario.hpp"
#include "ScenarioPanels.hpp"

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
    state.params_a.kind   = cdbg::Kind_Box;
    state.params_b.kind   = cdbg::Kind_Box;
    state.pose_a.position = m3d::vec3{0.0, 0.0, 0.0};
    state.pose_b.position = m3d::vec3{0.0, 0.95, 0.0};

    // ── Crash detection: scn-without-ok means previous run faulted ──────
    cdbg::ensure_dir(cdbg::kScenarioDir);
    if (cdbg::file_exists(cdbg::kLastInputPath) &&
        !cdbg::file_exists(cdbg::kCleanShutdownPath))
    {
        state.crash_recovery_pending = true;
        state.live_recompute         = false;  // don't fault on first frame
    }
    // The .ok marker is now stale (or never existed). Remove it so the next
    // crash is detectable; we'll recreate it on clean shutdown.
    cdbg::delete_file(cdbg::kCleanShutdownPath);

    // Track the previous input snapshot so we can detect changes and only
    // write scenarios/_last_input.scn when something actually moved.
    cdbg::DebuggerState prev_inputs = state;
    bool first_frame = true;

    cdbg::PipelineResult result;

    while (!WindowShouldClose())
    {
        if (!ImGui::GetIO().WantCaptureMouse &&
            !ImGui::GetIO().WantCaptureKeyboard)
            camera_sys.update(camera);

        const rbc::Shape shape_a = cdbg::make_shape(state.params_a);
        const rbc::Shape shape_b = cdbg::make_shape(state.params_b);
        const m3d::tf    tf_a    = cdbg::pose_to_tf(state.pose_a);
        const m3d::tf    tf_b    = cdbg::pose_to_tf(state.pose_b);

        // ── Auto-save BEFORE invoking the pipeline ─────────────────────
        // If run_pipeline crashes, this is the file we want on disk.
        const bool inputs_changed = first_frame ||
                                    !cdbg::inputs_equal(state, prev_inputs);
        if (inputs_changed)
        {
            cdbg::save_scenario(cdbg::kLastInputPath, state);
            prev_inputs = state;
        }
        first_frame = false;

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
        cdbg::draw_scenario_panel(state, result);
        camera_sys.draw_panel();
        rlImGuiEnd();

        EndDrawing();
    }

    // ── Clean shutdown: write .ok marker so next launch knows we lived ──
    cdbg::write_marker(cdbg::kCleanShutdownPath);

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
