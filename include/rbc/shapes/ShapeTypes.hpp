#pragma once
#include <variant>
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/Ellipsoid.hpp"
#include "rbc/shapes/Capsule.hpp"
#include "rbc/shapes/Cylinder.hpp"
#include "rbc/shapes/Cone.hpp"
#include "rbc/shapes/Plane.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Mesh.hpp"
#include "rbc/shapes/ConvexHull.hpp"

/**
 * @defgroup rbc rbc — Rigid-Body Collision
 * @brief Collision shapes, broad-phase, narrow-phase, GJK/EPA, and contact manifolds.
 *
 * The rbc library answers a single question: given two shapes with poses,
 * are they overlapping, and if so, where? It is split into three layers:
 *
 * - **Shapes** (`include/rbc/shapes/`) — geometric primitives plus a
 *   uniform interface (`support`, `compute_aabb`, `face_corners`,
 *   `is_gjk_convex`, `representative_radius`). All shape kinds are
 *   discriminated through a `std::variant` in `Shape`.
 * - **Broad / narrow phase** (`AABB.hpp`, `BroadPhase.hpp`,
 *   `narrow_phase.hpp`, `Dispatcher.hpp`) — bounding-box pruning, then
 *   per-pair dispatch to the right algorithm.
 * - **Algorithms** (`gjk/`, `analytic/`) — GJK + EPA for generic convex
 *   pairs, hand-written analytic routines for common pairs where they
 *   beat GJK on speed and robustness.
 *
 * All public collision queries return a `ContactManifold` (defined in
 * `gjk/ContactManifoldGenerator.hpp`) with up to 4 contact points.
 */

/**
 * @file ShapeTypes.hpp
 * @brief `Shape` variant + variant-level dispatchers (`compute_aabb`,
 *        `shape_support`, `shape_face_corners`, `shape_representative_radius`).
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Type-erased collision shape: a `std::variant` over every concrete shape.
     *
     * The variant alternative list IS the master list of shape kinds. Adding
     * a new shape is one line in the `std::variant<...>` plus a header that
     * provides `support`, `compute_aabb`, `is_gjk_convex`, `representative_radius`,
     * and `face_corners`. No macros, no parallel enum, no manual dispatch table.
     *
     * Consumers query the kind with `is<T>()`, read a typed alternative with
     * `get<T>()` (throws on mismatch), or get a nullable pointer with `as<T>()`.
     *
     * @code
     * Shape s = Box{m3d::vec3{1, 0.5, 0.5}};
     * if (s.is<Box>()) {
     *     const Box& b = s.get<Box>();
     *     // ...
     * }
     * AABB world_box = compute_aabb(s, body_tf);
     * @endcode
     *
     * @ingroup rbc
     */
    struct Shape
    {
        std::variant<Sphere, Box, Ellipsoid, Capsule, Cylinder, Cone, Plane, Heightmap, Mesh, ConvexHull> v;

        /** @brief Default-construct holding a default `Sphere`. */
        Shape() = default;
        /** @brief Construct from a concrete shape. */
        Shape(const Sphere     &s) : v(s) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Box        &b) : v(b) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Ellipsoid  &e) : v(e) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Capsule    &c) : v(c) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Cylinder &c) : v(c) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Cone       &c) : v(c) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Plane      &p) : v(p) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Heightmap  &h) : v(h) {}
        /** @brief Construct from a concrete shape. */
        Shape(const Mesh       &m) : v(m) {}
        /** @brief Construct from a concrete shape. */
        Shape(const ConvexHull &h) : v(h) {}
        

        /** @brief Read the active alternative; throws `std::bad_variant_access` on mismatch. */
        template <class T> const T &get() const { return std::get<T>(v); }
        /** @brief Read the active alternative; returns `nullptr` on mismatch. */
        template <class T> const T *as()  const { return std::get_if<T>(&v); }
        /** @brief Test whether the variant currently holds a `T`. */
        template <class T> bool     is()  const { return std::holds_alternative<T>(v); }
    };

    // ── Variant-level dispatchers ────────────────────────────────────────────
    // Each routes through std::visit to the per-shape free function. The
    // ADL-found overload is selected at compile time per alternative; runtime
    // cost is one indirect call (one variant index lookup + jump).

    /**
     * @brief World-space AABB of `shape` placed at `tf`.
     *
     * Variant-level dispatcher: routes through `std::visit` to the per-shape
     * `compute_aabb(const T&, const tf&)` overload.
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Shape &shape, const m3d::tf &tf)
    {
        return std::visit([&](const auto &x) { return compute_aabb(x, tf); }, shape.v);
    }

    /**
     * @brief Local-space support point of `shape` along `dir`.
     *
     * Variant-level dispatcher used by GJK. Only meaningful for convex
     * shapes (`is_gjk_convex(const T*) == true`); non-convex shapes return
     * a placeholder and should never be called this way at runtime.
     *
     * @ingroup rbc
     */
    inline m3d::vec3 shape_support(const Shape &shape, const m3d::vec3 &dir)
    {
        return std::visit([&](const auto &x) { return support(x, dir); }, shape.v);
    }

    /**
     * @brief World-space corners of the face of `shape` that is most aligned with `dir`.
     *
     * Used by the contact-manifold generator as the reference / incident
     * polygon. Returns the number of corners written (up to 4); shapes
     * without flat faces fall back to a disc approximation in
     * `get_generic_face_corners`.
     *
     * @ingroup rbc
     */
    inline int shape_face_corners(const Shape &shape, const m3d::tf &tf,
                                  const m3d::vec3 &dir, m3d::vec3 out[4])
    {
        return std::visit([&](const auto &x) { return face_corners(x, tf, dir, out); }, shape.v);
    }

    /**
     * @brief A representative size used for contact-tolerance scaling.
     *
     * Roughly the largest principal radius / half-extent of the shape.
     * Returns 0 for unbounded shapes (`Plane`, `Heightmap`, `Mesh`).
     *
     * @ingroup rbc
     */
    inline m3d::scalar shape_representative_radius(const Shape &shape)
    {
        return std::visit([](const auto &x) { return representative_radius(x); }, shape.v);
    }
} // namespace rbc
