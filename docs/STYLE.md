# Documentation Style Guide

All public symbols in `include/` are documented with **JavaDoc-style Doxygen
comments**. This guide is short by design — keep docstrings practical, not
ceremonial.

## Comment form

Use `/** ... */` blocks placed on the **declaration in the header**, never on
the definition in the `.cpp`. Doxygen merges the two and `.cpp` files stay
focused on code.

```cpp
/**
 * @brief One-line summary of what this function does.
 *
 * Optional longer description. Wraps at ~100 cols. Explain WHY the function
 * exists, not WHAT each line does.
 *
 * @param x  Description of x.
 * @param[out] out  Filled with the result.
 * @return What the return value means; omit for void.
 *
 * @pre  Caller-side invariant.
 * @post Function-side guarantee.
 *
 * @ingroup rbps
 */
void do_something(int x, vec3& out);
```

## Required tags

| Tag       | When                                           |
|-----------|------------------------------------------------|
| `@brief`  | Always. Single line.                           |
| `@param`  | Every parameter, in order.                     |
| `@return` | Every non-void function.                       |
| `@tparam` | Every template parameter.                      |
| `@ingroup`| Every public symbol — see groups below.        |

## Optional tags

`@pre`, `@post`, `@note`, `@warning`, `@see`, `@code` / `@endcode`.

Use `@code` examples on entry-point types (e.g. `World`, `BodyCollection`,
`vec3`) — not on every helper.

## Groups

Each module declares a top-level group in one header (typically the most
prominent one, e.g. `Body.hpp` for `rbps`):

```cpp
/**
 * @defgroup rbps rbps — Physics core
 * @brief Rigid-body dynamics, constraints, joints, contacts.
 */
```

Then every public symbol in that module adds `@ingroup rbps` to its block.

**Group names in use:**

- `math3d` — 3D math primitives.
- `storage` — SoA containers.
- `ivc` — Index Vector Core (IVC) — stable ID management for Struct-of-Arrays collections.
- `rbc` — Collision shapes, broad/narrow phase.
- `rbps` — Physics core, public API.
- `visr` — Visualizer (optional).
- `internals` — Algorithm implementations (GJK, EPA, solver). Documented but
  surfaced in a separate sidebar group so casual users aren't overwhelmed.

A symbol can belong to two groups: e.g. GJK functions are `@ingroup rbc` and
`@ingroup internals` so they appear under both.

## File-level headers

Every `.hpp` starts with:

```cpp
/**
 * @file vec3.hpp
 * @brief 3-component vector type and inline operators.
 * @ingroup math3d
 */
```

## What NOT to document

- Trivial accessors (`int size() const;`) — let the name speak.
- Operator overloads beyond a one-line `@brief`.
- Implementation details that change frequently. If a comment would rot,
  don't write it.

## Macros that synthesise types

For macros like `DEFINE_DYN_SOA` and `BODY_FIELDS`, document the **macro
itself** with a `@code` example showing what it expands to. Don't try to
document each generated symbol individually.

## Algorithm internals

Functions inside `gjk/`, `EPA`, the constraint solver, and broad-phase
internals follow the same style but:

- Add `@ingroup internals` (in addition to the module group).
- Focus comments on **invariants, numerical caveats, and references** to
  papers / textbooks rather than re-deriving the math.

## Verifying compliance

After completing a phase, the following must hold:

```bash
# No public header in the phase's module is missing @brief.
grep -L "@brief" include/<module>/**/*.hpp

# Doxygen builds with zero new warnings.
cmake --build build --target docs 2>&1 | grep -i warning
```
