@page feature_ui_overhaul UI Overhaul

# UI Overhaul

## Current State
The visualizer's panels live in [ui/Panels.hpp](../../../include/visr/ui/Panels.hpp)
(~1000 LOC, all inline). Each panel calls `ImGui::SetNextWindowPos` /
`SetNextWindowSize` with hand-tuned coordinates so windows tile poorly
on large monitors and overlap on small ones. The selection panel,
sim-control panel, and contact panel can all sit on top of each other.

The "ImGui input bleeding to raylib" bug: `SelectionSystem::update`
in [systems/SelectionSystem.hpp](../../../include/visr/systems/SelectionSystem.hpp)
gates picking on `!ImGui::GetIO().WantCaptureMouse`, but
`CameraSystem::update` (RMB drag, WASD) does NOT — so dragging an ImGui
slider with RMB also rotates the camera, and WASD inside an ImGui text
field still moves the camera.

## Motivation
- The current layout is "messy" (user feedback) and gets worse as more
  panels are added in [test-app-interactivity.md](../test-app-interactivity/test-app-interactivity.md).
- The input bleed makes parameter editing feel broken — every text
  edit triggers camera motion.
- Without a docking layout, screenshots / videos require manual window
  arrangement every session.

## Proposed Approach
1. Enable ImGui docking. Set
   `ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable` in
   `RenderSystem::init`. Add a default `ImGui::DockSpaceOverViewport`
   call at the top of `ui::draw_all`.
2. Persist layouts via `imgui.ini` checked into `docs/visr/` for the
   demo. Users get a sensible default; pros configure their own.
3. Unify input gating in `CameraSystem::update`: refuse all input when
   `ImGui::GetIO().WantCaptureMouse || WantCaptureKeyboard`. Apply the
   same gate to the pause shortcut in `VisrApp::run`.
4. Group related panels with `ImGui::Begin` flags `NoBringToFrontOnFocus`
   for read-only inspectors (so clicking them doesn't raise them over
   the live sim window).
5. Theme pass: replace the `ImGui::SeparatorText` clusters with a
   `ImGui::CollapsingHeader` each so panels start small and expand on
   demand.

## Risks / Open Questions
- ImGui docking branch is in master since 1.92 (we use 1.92.6 — fine).
- Persisted `imgui.ini` is per-machine; check if relative paths work or
  if we need to set `ImGui::GetIO().IniFilename` to a known location.
- Camera-input gating change is observable behaviour — flag it in the
  changelog so users used to "WASD always moves" know it now requires
  3-D viewport focus.
