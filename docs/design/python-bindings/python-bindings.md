@page feature_python_bindings Python Bindings (pybind11)

# Python Bindings (pybind11)

## Current State
The engine is pure C++17 with no Python interop. The only entry point
is `visr_demo` in [src/visr_demo/main.cpp](../../../src/visr_demo/main.cpp),
a hand-coded C++ scene runner. The CMake project produces a static
library `rbps_lib` and the demo executable; no shared library, no
ABI considerations, no Python module.

## Motivation
- **Every RL framework drives the simulator from Python.** Stable-
  Baselines3, CleanRL, Tianshou, RLlib, JAX-based stacks. Without
  bindings, no RL paper is publishable.
- **Every competing engine ships Python first.** MuJoCo, PyBullet,
  Brax, Genesis, Newton, IsaacLab. A C++-only release looks
  unfinished to reviewers regardless of correctness.
- **Notebooks accelerate iteration.** Loading a URDF, stepping the
  engine, and plotting state in a Jupyter cell turns a 30-minute
  recompile cycle into a 30-second one.
- **Numpy zero-copy is cheap given SoA.** The engine's storage
  layout is already what numpy wants — contiguous arrays per field.

## Proposed Approach
**pybind11 + scikit-build-core, packaged as `rbps`. Gymnasium wrapper
ships in the same package.**

1. Add `pybind11` via `FetchContent` (header-only, permissive). New
   CMake target `rbps_py` building a Python extension module
   (`rbps_py.cpython-3X-*.so`). Gated behind `option(RBPS_BUILD_PYTHON
   "Build Python bindings" OFF)`.
2. Bindings module structure under `python/rbps/`:
   - `_native.cpp` — pybind11 module exposing `World`,
     `BodyParams`, `JointParams`, `ColliderParams`, snapshot, and
     plain-C++ enums.
   - `__init__.py` — Python-facing types, dataclasses for params,
     numpy adapters.
   - `gym.py` — `RbpsEnv` subclassing `gymnasium.Env` with
     `reset` / `step` / `render`.
3. **Numpy zero-copy where possible.** `World.body_positions()`
   returns a numpy view with shape `(n_bodies, 3)`, dtype `float64`,
   strided over the `position` SoA field. Lifetime tied to the
   `World` via pybind11's `py::keep_alive`.
4. **GIL release.** `World.step` releases the GIL via
   `py::gil_scoped_release` so callers can prepare observations or
   batch other envs in parallel.
5. **URDF entry point.** `rbps.load_urdf(path) -> World` once
   [scene-format](../scene-format/scene-format.md) and
   [articulated-bodies-xpbd](../articulated-bodies-xpbd/articulated-bodies-xpbd.md)
   land. Until then, scenes are constructed via `World.create_body` /
   `create_joint` / etc. mirroring the C++ API.
6. **Packaging via scikit-build-core.** `pyproject.toml` declares the
   build backend; `pip install .` builds the C++ extension and
   installs the package. CI matrix: cpython 3.10–3.13 × {linux, macos}.
7. **Tests:** `tests/python/` runs `pytest` — basic import,
   step/reset round-trip, snapshot/restore, gym `check_env`
   compatibility.

## Risks / Open Questions
- **ABI lifetime.** `World` cannot move once Python holds numpy views
  into its SoA fields. Mark `World` non-movable in C++ when bindings
  are enabled, or document that mutation invalidates views.
- **Float type.** The engine uses `double`; numpy's default is
  `float64` so this is fine, but document for users coming from
  single-precision sims (MuJoCo MJX, Brax) that they'll need to
  cast at the boundary.
- **`set_state` API surface.** RL frameworks set state in many ways
  (per-body pose, joint angles, full snapshot). Start with
  `World.snapshot() / restore()` plus per-body setters; add
  joint-angle convenience helpers when articulations land.
- **Build system complexity.** scikit-build-core + Doxygen + raylib +
  Catch2 + pybind11 is a lot of moving pieces. Keep `RBPS_BUILD_PYTHON`
  default-OFF so the C++-only build remains the simplest path.
