@page planned_features Planned Features

# Planned Features

Short (≤ 1 page) design notes for upcoming RBPS features. Each doc is
self-contained: current state in the repo, motivation, a concrete
implementation sketch, and the open questions worth resolving before
work begins. They are intentionally terse — enough to brief a contributor
or an automated agent so they can start implementing without re-deriving
everything from scratch.

Each feature lives in its own folder under `docs/design/<slug>/` so that
images, sub-RFCs, prototype notes, and benchmark results can grow next
to the design doc as the work progresses.

## Index

| # | Feature | Page |
|---|---|---|
| 1  | Visualizer refactor                   | @subpage feature_visualizer_refactor |
| 2  | Heightmap collider rewrite            | @subpage feature_heightmap |
| 3  | Scene file format (URDF / MJCF / USD) | @subpage feature_scene_format |
| 4  | Test-app runtime interactivity        | @subpage feature_test_app_interactivity |
| 5  | UI overhaul                           | @subpage feature_ui_overhaul |
| 6  | OpenMP parallelization                | @subpage feature_openmp |
| 7  | Pluggable broad phase (BVH)           | @subpage feature_broadphase_bvh |
| 8  | Contact-manifold generalisation       | @subpage feature_contact_generation |
| 9  | Rendering pipeline upgrade            | @subpage feature_rendering |
| 10 | GPU camera + LiDAR sensors            | @subpage feature_sensors_gpu |
| 11 | ROS 2 bridge                          | @subpage feature_ros2 |
| 12 | Articulated bodies (XPBD)             | @subpage feature_articulated_bodies_xpbd |
| 13 | World snapshot / restore              | @subpage feature_world_snapshot_restore |
| 14 | Python bindings (pybind11)            | @subpage feature_python_bindings |
| 15 | Benchmark harness                     | @subpage feature_benchmark_harness |
| 16 | Actuator models                       | @subpage feature_actuator_models |
| 17 | Sample-based MPC (CEM / MPPI)         | @subpage feature_sample_mpc |
| 18 | Multi-world support                   | @subpage feature_multi_world |
| 19 | Convex hull collider                  | @subpage feature_convex_hull |
| 20 | Collider completeness                 | @subpage feature_collider_completeness |
| –  | Roadmap (dependencies between the above) | @subpage feature_roadmap |

The **Roadmap** page links every feature above into a topological
dependency graph; start there if you don't know which feature to pick
up first. Rendering the embedded graph requires `graphviz`
(install `dot` and rebuild the docs).

## Doc template

```
@page feature_<slug> <Title>

# <Title>

## Current State
Where it lives today (file paths, what works, what's stubbed).

## Motivation
Why this matters, what it unblocks.

## Proposed Approach
Concrete implementation sketch — files to touch, key types/functions
to add. Reuse existing utilities where possible.

## Risks / Open Questions
Bullet list, ≤ 4 items.
```

When a feature ships, leave its design doc in place and add an
**Outcome** section noting the commit / PR and any deviations from the
plan.
