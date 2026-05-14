/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "RBPS", "index.html", [
    [ "RBPS — Rigid Body Physics Simulator", "index.html", "index" ],
    [ "VISR : VIsualization System for RBPS", "md__2home_2runner_2work_2rbps_2rbps_2include_2visr_2Readme.html", null ],
    [ "Convex Hull — Implementation Notes", "feature_convex_hull_impl.html", [
      [ "Convex Hull — Implementation Notes", "feature_convex_hull_impl.html#autotoc_md41", [
        [ "Recap", "feature_convex_hull_impl.html#autotoc_md42", null ],
        [ "Task list", "feature_convex_hull_impl.html#autotoc_md43", null ],
        [ "File-by-file touch list", "feature_convex_hull_impl.html#autotoc_md44", [
          [ "<tt>rbc</tt> (collision core)", "feature_convex_hull_impl.html#autotoc_md45", null ],
          [ "Tests", "feature_convex_hull_impl.html#autotoc_md46", null ],
          [ "<tt>visr</tt> (visualization)", "feature_convex_hull_impl.html#autotoc_md47", null ]
        ] ],
        [ "ConvexHull contract (header sketch)", "feature_convex_hull_impl.html#autotoc_md48", null ],
        [ "Inertia / volume fallback contract", "feature_convex_hull_impl.html#autotoc_md49", null ],
        [ "Plane fast path", "feature_convex_hull_impl.html#autotoc_md50", null ],
        [ "Visualizer integration", "feature_convex_hull_impl.html#autotoc_md51", null ],
        [ "Usage example", "feature_convex_hull_impl.html#autotoc_md52", null ],
        [ "Open questions resolved during planning", "feature_convex_hull_impl.html#autotoc_md53", null ],
        [ "Out of scope (won't ship in this PR)", "feature_convex_hull_impl.html#autotoc_md54", null ]
      ] ]
    ] ],
    [ "Planned Features", "planned_features.html", [
      [ "Planned Features", "planned_features.html#autotoc_md76", [
        [ "Index", "planned_features.html#autotoc_md77", null ],
        [ "Doc template", "planned_features.html#autotoc_md78", null ]
      ] ],
      [ "Visualizer Refactor", "feature_visualizer_refactor.html", [
        [ "Visualizer Refactor", "feature_visualizer_refactor.html#autotoc_md125", [
          [ "Current State", "feature_visualizer_refactor.html#autotoc_md126", null ],
          [ "Motivation", "feature_visualizer_refactor.html#autotoc_md127", null ],
          [ "Proposed Approach", "feature_visualizer_refactor.html#autotoc_md128", null ],
          [ "Risks / Open Questions", "feature_visualizer_refactor.html#autotoc_md129", null ]
        ] ]
      ] ],
      [ "Heightmap Collider Rewrite", "feature_heightmap.html", [
        [ "Heightmap Collider Rewrite", "feature_heightmap.html#autotoc_md55", [
          [ "Current State", "feature_heightmap.html#autotoc_md56", null ],
          [ "Motivation", "feature_heightmap.html#autotoc_md57", null ],
          [ "Proposed Approach", "feature_heightmap.html#autotoc_md58", null ],
          [ "Pair coverage", "feature_heightmap.html#autotoc_md59", null ],
          [ "Risks / Open Questions", "feature_heightmap.html#autotoc_md60", null ]
        ] ]
      ] ],
      [ "Scene File Format", "feature_scene_format.html", [
        [ "Scene File Format", "feature_scene_format.html#autotoc_md105", [
          [ "Current State", "feature_scene_format.html#autotoc_md106", null ],
          [ "Motivation", "feature_scene_format.html#autotoc_md107", null ],
          [ "Proposed Approach", "feature_scene_format.html#autotoc_md108", null ],
          [ "Risks / Open Questions", "feature_scene_format.html#autotoc_md109", null ]
        ] ]
      ] ],
      [ "Test-App Runtime Interactivity", "feature_test_app_interactivity.html", [
        [ "Test-App Runtime Interactivity", "feature_test_app_interactivity.html#autotoc_md115", [
          [ "Current State", "feature_test_app_interactivity.html#autotoc_md116", null ],
          [ "Motivation", "feature_test_app_interactivity.html#autotoc_md117", null ],
          [ "Proposed Approach", "feature_test_app_interactivity.html#autotoc_md118", null ],
          [ "Risks / Open Questions", "feature_test_app_interactivity.html#autotoc_md119", null ]
        ] ]
      ] ],
      [ "UI Overhaul", "feature_ui_overhaul.html", [
        [ "UI Overhaul", "feature_ui_overhaul.html#autotoc_md120", [
          [ "Current State", "feature_ui_overhaul.html#autotoc_md121", null ],
          [ "Motivation", "feature_ui_overhaul.html#autotoc_md122", null ],
          [ "Proposed Approach", "feature_ui_overhaul.html#autotoc_md123", null ],
          [ "Risks / Open Questions", "feature_ui_overhaul.html#autotoc_md124", null ]
        ] ]
      ] ],
      [ "OpenMP Parallelization", "feature_openmp.html", [
        [ "OpenMP Parallelization", "feature_openmp.html#autotoc_md66", [
          [ "Current State", "feature_openmp.html#autotoc_md67", null ],
          [ "Motivation", "feature_openmp.html#autotoc_md68", null ],
          [ "Proposed Approach", "feature_openmp.html#autotoc_md69", null ],
          [ "Risks / Open Questions", "feature_openmp.html#autotoc_md70", null ]
        ] ]
      ] ],
      [ "Pluggable Broad Phase + BVH", "feature_broadphase_bvh.html", [
        [ "Pluggable Broad Phase + BVH Backend", "feature_broadphase_bvh.html#autotoc_md16", [
          [ "Current State", "feature_broadphase_bvh.html#autotoc_md17", null ],
          [ "Motivation", "feature_broadphase_bvh.html#autotoc_md18", null ],
          [ "Proposed Approach", "feature_broadphase_bvh.html#autotoc_md19", null ],
          [ "Risks / Open Questions", "feature_broadphase_bvh.html#autotoc_md20", null ]
        ] ]
      ] ],
      [ "Contact-Manifold Generalisation", "feature_contact_generation.html", [
        [ "Contact-Manifold Generalisation", "feature_contact_generation.html#autotoc_md31", [
          [ "Current State", "feature_contact_generation.html#autotoc_md32", null ],
          [ "Motivation", "feature_contact_generation.html#autotoc_md33", null ],
          [ "Proposed Approach", "feature_contact_generation.html#autotoc_md34", null ],
          [ "Risks / Open Questions", "feature_contact_generation.html#autotoc_md35", null ]
        ] ]
      ] ],
      [ "Rendering Pipeline Upgrade", "feature_rendering.html", [
        [ "Rendering Pipeline Upgrade", "feature_rendering.html#autotoc_md79", [
          [ "Current State", "feature_rendering.html#autotoc_md80", null ],
          [ "Motivation", "feature_rendering.html#autotoc_md81", null ],
          [ "Proposed Approach", "feature_rendering.html#autotoc_md82", null ],
          [ "Risks / Open Questions", "feature_rendering.html#autotoc_md83", null ]
        ] ]
      ] ],
      [ "GPU Camera + LiDAR Sensors", "feature_sensors_gpu.html", [
        [ "GPU Camera + LiDAR Sensors", "feature_sensors_gpu.html#autotoc_md110", [
          [ "Current State", "feature_sensors_gpu.html#autotoc_md111", null ],
          [ "Motivation", "feature_sensors_gpu.html#autotoc_md112", null ],
          [ "Proposed Approach", "feature_sensors_gpu.html#autotoc_md113", null ],
          [ "Risks / Open Questions", "feature_sensors_gpu.html#autotoc_md114", null ]
        ] ]
      ] ],
      [ "ROS 2 Bridge", "feature_ros2.html", [
        [ "ROS 2 Bridge", "feature_ros2.html#autotoc_md95", [
          [ "Current State", "feature_ros2.html#autotoc_md96", null ],
          [ "Motivation", "feature_ros2.html#autotoc_md97", null ],
          [ "Proposed Approach", "feature_ros2.html#autotoc_md98", null ],
          [ "Risks / Open Questions", "feature_ros2.html#autotoc_md99", null ]
        ] ]
      ] ],
      [ "Articulated Bodies (Maximal-Coordinate XPBD)", "feature_articulated_bodies_xpbd.html", [
        [ "Articulated Bodies (Maximal-Coordinate XPBD)", "feature_articulated_bodies_xpbd.html#autotoc_md6", [
          [ "Current State", "feature_articulated_bodies_xpbd.html#autotoc_md7", null ],
          [ "Motivation", "feature_articulated_bodies_xpbd.html#autotoc_md8", null ],
          [ "Proposed Approach", "feature_articulated_bodies_xpbd.html#autotoc_md9", null ],
          [ "Risks / Open Questions", "feature_articulated_bodies_xpbd.html#autotoc_md10", null ]
        ] ]
      ] ],
      [ "World Snapshot / Restore", "feature_world_snapshot_restore.html", [
        [ "World Snapshot / Restore", "feature_world_snapshot_restore.html#autotoc_md130", [
          [ "Current State", "feature_world_snapshot_restore.html#autotoc_md131", null ],
          [ "Motivation", "feature_world_snapshot_restore.html#autotoc_md132", null ],
          [ "Proposed Approach", "feature_world_snapshot_restore.html#autotoc_md133", null ],
          [ "Risks / Open Questions", "feature_world_snapshot_restore.html#autotoc_md134", null ]
        ] ]
      ] ],
      [ "Python Bindings (pybind11)", "feature_python_bindings.html", [
        [ "Python Bindings (pybind11)", "feature_python_bindings.html#autotoc_md71", [
          [ "Current State", "feature_python_bindings.html#autotoc_md72", null ],
          [ "Motivation", "feature_python_bindings.html#autotoc_md73", null ],
          [ "Proposed Approach", "feature_python_bindings.html#autotoc_md74", null ],
          [ "Risks / Open Questions", "feature_python_bindings.html#autotoc_md75", null ]
        ] ]
      ] ],
      [ "Benchmark Harness", "feature_benchmark_harness.html", [
        [ "Benchmark Harness", "feature_benchmark_harness.html#autotoc_md11", [
          [ "Current State", "feature_benchmark_harness.html#autotoc_md12", null ],
          [ "Motivation", "feature_benchmark_harness.html#autotoc_md13", null ],
          [ "Proposed Approach", "feature_benchmark_harness.html#autotoc_md14", null ],
          [ "Risks / Open Questions", "feature_benchmark_harness.html#autotoc_md15", null ]
        ] ]
      ] ],
      [ "Actuator Models", "feature_actuator_models.html", [
        [ "Actuator Models", "feature_actuator_models.html#autotoc_md1", [
          [ "Current State", "feature_actuator_models.html#autotoc_md2", null ],
          [ "Motivation", "feature_actuator_models.html#autotoc_md3", null ],
          [ "Proposed Approach", "feature_actuator_models.html#autotoc_md4", null ],
          [ "Risks / Open Questions", "feature_actuator_models.html#autotoc_md5", null ]
        ] ]
      ] ],
      [ "Sample-Based MPC (CEM / MPPI)", "feature_sample_mpc.html", [
        [ "Sample-Based MPC (CEM / MPPI)", "feature_sample_mpc.html#autotoc_md100", [
          [ "Current State", "feature_sample_mpc.html#autotoc_md101", null ],
          [ "Motivation", "feature_sample_mpc.html#autotoc_md102", null ],
          [ "Proposed Approach", "feature_sample_mpc.html#autotoc_md103", null ],
          [ "Risks / Open Questions", "feature_sample_mpc.html#autotoc_md104", null ]
        ] ]
      ] ],
      [ "Multi-World Support", "feature_multi_world.html", [
        [ "Multi-World Support", "feature_multi_world.html#autotoc_md61", [
          [ "Current State", "feature_multi_world.html#autotoc_md62", null ],
          [ "Motivation", "feature_multi_world.html#autotoc_md63", null ],
          [ "Proposed Approach", "feature_multi_world.html#autotoc_md64", null ],
          [ "Risks / Open Questions", "feature_multi_world.html#autotoc_md65", null ]
        ] ]
      ] ],
      [ "Convex Hull Collider", "feature_convex_hull.html", [
        [ "Convex Hull Collider", "feature_convex_hull.html#autotoc_md36", [
          [ "Current State", "feature_convex_hull.html#autotoc_md37", null ],
          [ "Motivation", "feature_convex_hull.html#autotoc_md38", null ],
          [ "Proposed Approach", "feature_convex_hull.html#autotoc_md39", null ],
          [ "Risks / Open Questions", "feature_convex_hull.html#autotoc_md40", null ]
        ] ]
      ] ],
      [ "Collider Completeness", "feature_collider_completeness.html", [
        [ "Collider Completeness", "feature_collider_completeness.html#autotoc_md21", [
          [ "Current State", "feature_collider_completeness.html#autotoc_md22", null ],
          [ "Motivation", "feature_collider_completeness.html#autotoc_md23", null ],
          [ "Proposed Approach", "feature_collider_completeness.html#autotoc_md24", [
            [ "1. Plane / Heightmap broadphase unification", "feature_collider_completeness.html#autotoc_md25", null ],
            [ "2. Compound colliders", "feature_collider_completeness.html#autotoc_md26", null ],
            [ "3. Cylinder primitive", "feature_collider_completeness.html#autotoc_md27", null ],
            [ "4. CCD — scoping only, not implementation", "feature_collider_completeness.html#autotoc_md28", null ],
            [ "5. Remaining stubbed Heightmap pairs", "feature_collider_completeness.html#autotoc_md29", null ]
          ] ],
          [ "Risks / Open Questions", "feature_collider_completeness.html#autotoc_md30", null ]
        ] ]
      ] ],
      [ "Roadmap", "feature_roadmap.html", [
        [ "Roadmap", "feature_roadmap.html#autotoc_md84", [
          [ "How features connect", "feature_roadmap.html#autotoc_md85", null ],
          [ "Prerequisite table", "feature_roadmap.html#autotoc_md86", null ],
          [ "Levels", "feature_roadmap.html#autotoc_md87", [
            [ "Level 0 — Foundations", "feature_roadmap.html#autotoc_md88", null ],
            [ "Level 1", "feature_roadmap.html#autotoc_md89", null ],
            [ "Level 2", "feature_roadmap.html#autotoc_md90", null ],
            [ "Level 3", "feature_roadmap.html#autotoc_md91", null ],
            [ "Level 4", "feature_roadmap.html#autotoc_md92", null ]
          ] ],
          [ "Foundation gaps before level-1+ work begins", "feature_roadmap.html#autotoc_md93", null ],
          [ "Conventions", "feature_roadmap.html#autotoc_md94", null ]
        ] ]
      ] ]
    ] ],
    [ "Documentation Style Guide", "md__2home_2runner_2work_2rbps_2rbps_2docs_2STYLE.html", [
      [ "Comment form", "md__2home_2runner_2work_2rbps_2rbps_2docs_2STYLE.html#autotoc_md141", null ],
      [ "Required tags", "md__2home_2runner_2work_2rbps_2rbps_2docs_2STYLE.html#autotoc_md142", null ],
      [ "Optional tags", "md__2home_2runner_2work_2rbps_2rbps_2docs_2STYLE.html#autotoc_md143", null ]
    ] ],
    [ "Topics", "topics.html", "topics" ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", null ],
        [ "Variables", "functions_vars.html", null ],
        [ "Related Symbols", "functions_rela.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ],
    [ "Examples", "examples.html", "examples" ]
  ] ]
];

var NAVTREEINDEX =
[
"AABB_8hpp.html",
"functions_h.html",
"index.html",
"structrbc_1_1MeshData.html#ac6342d793526834e248a4ece4b2db828"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';