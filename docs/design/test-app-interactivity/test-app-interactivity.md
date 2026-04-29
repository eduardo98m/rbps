@page feature_test_app_interactivity Test-App Runtime Interactivity

# Test-App Runtime Interactivity

## Current State
`visr_demo` ([src/visr_demo/main.cpp](../../../src/visr_demo/main.cpp))
builds the scene in C++ before calling `VisrApp::run()`. To change the
scene the user must recompile. The `Command` channel
([include/visr/Command.hpp](../../../include/visr/Command.hpp)) supports
runtime impulses, teleports, and joint targets, but there is no
"create body / collider" command — only mutation of existing entities.

## Motivation
A physics tester is most useful when the user can iterate: drop a new
shape, attach a joint, tweak a parameter, see the result. Recompile
cycles kill that loop. Once this lands the demo becomes a real
sandbox / benchmarking surface.

## Proposed Approach
1. Extend the `Command` variant in
   [Command.hpp](../../../include/visr/Command.hpp) with creation /
   destruction commands:
   - `CmdSpawnBody { BodyParams; ShapeSnap; m3d::vec3 spawn_offset; }`
   - `CmdRemoveBody { uint32_t body_id; }`
   - `CmdSpawnJoint { JointType; uint32_t body_a, body_b; ... }`
2. Add arms to `dispatch_command` in
   [CommandDispatch.hpp](../../../include/visr/CommandDispatch.hpp). For
   spawn commands convert `ShapeSnap → rbc::Shape` (inverse of
   [SnapshotBuilder.hpp](../../../include/visr/SnapshotBuilder.hpp)'s
   `ShapeToSnap`) and call `world.create_body` / `world.create_collider`.
3. New panel `draw_spawn_panel` in
   [ui/Panels.hpp](../../../include/visr/ui/Panels.hpp): shape combo,
   mass / size sliders, "Spawn at camera target" button. Internally pushes
   the command through the transport.
4. Selection-aware actions: when a body is selected, the inspector grows
   "Delete", "Convert to static / dynamic", "Attach joint to clicked
   body" buttons.
5. Tie in with [scene-format.md](../scene-format/scene-format.md): add Save / Load
   buttons that round-trip through the URDF loader. Now the user can
   build interactively, save, and rerun headlessly.

## Risks / Open Questions
- Body / collider creation must be done on the **physics thread** to
  avoid mutating SoA arrays while the render thread is iterating them.
  All commands already cross the transport so this falls out for free
  — but new contributors may try to call `world.create_body` directly
  from a panel; document the rule.
- Stable-ID handling on save / load: URDF has no notion of `uint32_t`
  IDs; round-trip loses external references. Fine for the demo, worth
  flagging in [scene-format.md](../scene-format/scene-format.md).
- Spawn rate limiting: holding the spawn key shouldn't flood the queue
  faster than physics can drain it. Add a per-frame cap.
