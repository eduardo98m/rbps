@page feature_visualizer_refactor Visualizer Refactor

# Visualizer Refactor

## Current State
The `visr` module ([include/visr/](../../../include/visr/)) is entirely
header-only with 13 headers, several of which are large (`ui/Panels.hpp`
is ~1000 lines, `ui/RaylibDraw.hpp` ~500). Logic is mixed with rendering:
panel `draw_*` functions issue commands, mutate selection, and call
ImPlot. Tests cover only the data-plumbing layer (transport, snapshot
builder, debug channel, command dispatch); no system or panel is
exercised in CI.

## Motivation
- Compile times balloon because every translation unit that includes
  `VisrApp.hpp` re-parses ~3000 lines of inline UI code.
- Selection / picking / panel state is scattered across inline globals
  in `ui/Panels.hpp`, which makes UI changes risky.
- Adding new panels requires editing `Panels.hpp` directly — no plug-in
  mechanism, no per-panel testing.

## Proposed Approach
1. Convert `visr` from header-only to a real library with a `src/visr/`
   tree mirroring `include/visr/`. Move `ui::draw_*` panel
   implementations and `draw::draw_scene` body into `.cpp` files.
2. Replace the inline globals in [ui/Panels.hpp](../../../include/visr/ui/Panels.hpp)
   (`g_plot_export_request`, `g_export_status`, etc.) with a `UiState`
   struct owned by `RenderSystem`. Pass it explicitly to each panel.
3. Introduce a `Panel` interface (`virtual void draw(UiState&, ...)`) and
   an ordered `std::vector<std::unique_ptr<Panel>>` on `RenderSystem`.
   Built-in panels register themselves; user code adds extras the same way.
4. Add CTest coverage for at least one panel using a synthetic
   `FrameSnapshot`. ImGui can run headless with `ImGui::CreateContext()`
   without a renderer backend — enough to assert the command queue
   contents after simulated input.
5. Update `tests/visr/CMakeLists.txt` to compile the panels in headless
   mode (`visr_headless` already exists for this).

## Risks / Open Questions
- ImGui's per-frame state (`ImGui::Begin / End` ids, persistence ini
  files) is global. Splitting panels across TUs may surface subtle
  ordering bugs — keep `draw_all` as the single ordered driver.
- The `Panel` virtual call adds one indirect call per frame per panel
  (~10 panels × 60 Hz = trivial); confirm with a quick benchmark before
  shipping.
- Backward compatibility: callers using `extra_guis` (function vector
  in `VisrApp`) keep working; new code uses `Panel`.
