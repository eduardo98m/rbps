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
