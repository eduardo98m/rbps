@page feature_roadmap Roadmap

# Roadmap

Each entry in [docs/design/](../) describes one planned feature. They do not
say which feature **must ship before** another can start. This page makes
those edges explicit so contributors can pick up work in the right order
and parallelise where the graph allows.

The dependency graph below also surfaces a small set of foundation gaps
(notably the missing `ConvexHull` collider, the stubbed Heightmap pairs,
and the inconsistent Plane / Heightmap broadphase integration) that must
be closed before the level-1+ features that implicitly rely on them.
Those gaps have their own design docs:

- [convex-hull](../convex-hull/convex-hull.md) — first-class convex-mesh shape
- [collider-completeness](../collider-completeness/collider-completeness.md) — Plane/Heightmap broadphase, compound colliders, cylinder, CCD
- [heightmap](../heightmap/heightmap.md) — finishing Box–HM and Mesh–HM

## How features connect

@dotfile dependencies.dot "Feature dependency graph"

The graph is also tracked in textual form below; the table is the source
of truth, [dependencies.dot](dependencies.dot) is its rendered counterpart.
Update both when prerequisites change.

## Prerequisite table

| Level | Feature | Direct prerequisites |
|---|---|---|
| 0 | [broadphase-bvh](../broadphase-bvh/broadphase-bvh.md) | — |
| 0 | [contact-generation](../contact-generation/contact-generation.md) | — |
| 0 | [world-snapshot-restore](../world-snapshot-restore/world-snapshot-restore.md) | — |
| 0 | [openmp-parallelization](../openmp-parallelization/openmp-parallelization.md) | — |
| 0 | [ui-overhaul](../ui-overhaul/ui-overhaul.md) | — |
| 0 | [rendering](../rendering/rendering.md) | — |
| 0 | [sensors-gpu](../sensors-gpu/sensors-gpu.md) | — |
| 0 | [convex-hull](../convex-hull/convex-hull.md) | — |
| 1 | [heightmap](../heightmap/heightmap.md) | contact-generation |
| 1 | [articulated-bodies-xpbd](../articulated-bodies-xpbd/articulated-bodies-xpbd.md) | contact-generation |
| 1 | [visualizer-refactor](../visualizer-refactor/visualizer-refactor.md) | ui-overhaul |
| 1 | [multi-world](../multi-world/multi-world.md) | openmp-parallelization, world-snapshot-restore |
| 1 | [collider-completeness](../collider-completeness/collider-completeness.md) | contact-generation, broadphase-bvh |
| 2 | [scene-format](../scene-format/scene-format.md) | articulated-bodies-xpbd, heightmap, contact-generation |
| 2 | [benchmark-harness](../benchmark-harness/benchmark-harness.md) | contact-generation, articulated-bodies-xpbd, heightmap |
| 3 | [actuator-models](../actuator-models/actuator-models.md) | articulated-bodies-xpbd, scene-format |
| 3 | [test-app-interactivity](../test-app-interactivity/test-app-interactivity.md) | visualizer-refactor, scene-format, articulated-bodies-xpbd, rendering |
| 3 | [python-bindings](../python-bindings/python-bindings.md) | scene-format, articulated-bodies-xpbd, world-snapshot-restore |
| 3 | [ros2-integration](../ros2-integration/ros2-integration.md) | scene-format |
| 4 | [sample-mpc](../sample-mpc/sample-mpc.md) | python-bindings, multi-world, world-snapshot-restore, articulated-bodies-xpbd |

**Reading rule.** A feature can start once every feature listed in its
prerequisite cell has shipped (i.e. its design doc has an `Outcome`
section per the convention in [the design index](../README.md)). Within a
level the order is unconstrained — pick based on contributor availability
and any externally imposed priorities.

## Levels

### Level 0 — Foundations
No prerequisites. Any of these can start immediately and several can run
concurrently because they touch disjoint parts of the codebase.

- **broadphase-bvh** — pluggable BVH backend alongside SAP.
- **contact-generation** — replace the disc fallback with real face / edge / vertex manifolds.
- **world-snapshot-restore** — structural + fast-path snapshot/restore API.
- **openmp-parallelization** — pragmas on the existing parallel-friendly loops.
- **ui-overhaul** — ImGui docking, input gating, layout persistence.
- **rendering** — Raylib v5.5 features (instancing, shadows, PBR-lite).
- **sensors-gpu** — CPU ray-cast fallback and (optionally) the GPU compute path.
- **convex-hull** — new shape variant; reuses existing GJK/EPA. See [convex-hull](../convex-hull/convex-hull.md).

### Level 1
- **heightmap** — finish Box–HM and Mesh–HM via the new contact-feature path.
- **articulated-bodies-xpbd** — multi-link chains, floating base, joint limits.
- **visualizer-refactor** — header-only → library, panel registry.
- **multi-world** — N independent worlds stepping in parallel.
- **collider-completeness** — Plane/HM broadphase unification, compound colliders, cylinder, CCD scoping. See [collider-completeness](../collider-completeness/collider-completeness.md).

### Level 2
- **scene-format** — URDF parser; the first feature that requires articulated bodies + a complete collision surface.
- **benchmark-harness** — pile / stack / pendulum / vehicle scenarios with MuJoCo and PyBullet baselines.

### Level 3
- **actuator-models** — PD gains, torque / velocity limits on joints.
- **test-app-interactivity** — runtime spawn / remove / save-load.
- **python-bindings** — `pip install rbps` with numpy zero-copy + Gymnasium wrapper.
- **ros2-integration** — `Ros2Transport` publishing TF / Odometry / JointState.

### Level 4
- **sample-mpc** — CEM / MPPI loops on top of multi-world and snapshot/restore.

## Foundation gaps before level-1+ work begins

The collision layer has three known gaps that the existing design docs
either don't cover or only mention in passing. All three are level-0 or
level-1 work and should be tracked alongside the rest of the foundation:

| Gap | Owning doc | Why it gates downstream work |
|---|---|---|
| `ConvexHull` shape missing from the variant | [convex-hull](../convex-hull/convex-hull.md) | scene-format imports user meshes; without `ConvexHull` they must be encoded as `Mesh` (`is_gjk_convex == false`), which defeats the GJK fast path. |
| `Box–HM` and `Mesh–HM` stubbed | [heightmap](../heightmap/heightmap.md) | benchmark-harness's vehicle scenario and any outdoor robot scene need real Box / Mesh contact on terrain. |
| Plane bypasses SAP, Heightmap registration ad-hoc, several pairs return `false` | [collider-completeness](../collider-completeness/collider-completeness.md) | Once broadphase-bvh introduces the pluggable broadphase, the unbounded shapes need a uniform integration path or the dispatcher table stays riddled with special cases. |

## Conventions

- **The table is the source of truth.** The rendered graph in
  [dependencies.dot](dependencies.dot) reflects the same edges. When a
  prerequisite changes, update both.
- **Strict edges only.** "Nice to have" relations (e.g. broadphase-bvh
  helps sensors-gpu raycast scaling, openmp helps benchmark-harness
  numbers) are not edges — sensors-gpu and benchmark-harness can ship
  without them.
- **Level numbers are not priorities.** They are the longest path from a
  level-0 root to the feature. Within a level, pick freely.
- **Doxygen build needs `graphviz`.** `HAVE_DOT=YES` in
  [Doxyfile.in](../../../Doxyfile.in) — install `dot` on contributor
  machines. The `.dot` file is also readable on its own; the rendered
  SVG is a convenience.
