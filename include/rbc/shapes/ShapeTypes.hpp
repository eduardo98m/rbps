#pragma once
#include <variant>
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"
#include "rbc/shapes/Ellipsoid.hpp"
#include "rbc/shapes/Capsule.hpp"
#include "rbc/shapes/Cone.hpp"
#include "rbc/shapes/Plane.hpp"
#include "rbc/shapes/Heightmap.hpp"
#include "rbc/shapes/Mesh.hpp"

namespace rbc
{
    // Shape — thin wrapper around a std::variant of every concrete shape type.
    //
    // The variant alternative list IS the master list of shape kinds — adding
    // a new shape is one line here plus a header (with support, compute_aabb,
    // is_gjk_convex, representative_radius, face_corners). No macros, no
    // parallel enum, no manual dispatch-table updates.
    //
    // Consumers query the kind with shape.is<T>(), read a typed alternative
    // with shape.get<T>() (throws on mismatch), or get a nullable pointer
    // with shape.as<T>().
    struct Shape
    {
        std::variant<Sphere, Box, Ellipsoid, Capsule, Cone, Plane, Heightmap, Mesh> v;

        Shape() = default;
        Shape(const Sphere    &s) : v(s) {}
        Shape(const Box       &b) : v(b) {}
        Shape(const Ellipsoid &e) : v(e) {}
        Shape(const Capsule   &c) : v(c) {}
        Shape(const Cone      &c) : v(c) {}
        Shape(const Plane     &p) : v(p) {}
        Shape(const Heightmap &h) : v(h) {}
        Shape(const Mesh      &m) : v(m) {}

        template <class T> const T &get() const { return std::get<T>(v); }
        template <class T> const T *as()  const { return std::get_if<T>(&v); }
        template <class T> bool     is()  const { return std::holds_alternative<T>(v); }
    };

    // ── Variant-level dispatchers ────────────────────────────────────────────
    // Each routes through std::visit to the per-shape free function. The
    // ADL-found overload is selected at compile time per alternative; runtime
    // cost is one indirect call (one variant index lookup + jump).

    inline AABB compute_aabb(const Shape &shape, const m3d::tf &tf)
    {
        return std::visit([&](const auto &x) { return compute_aabb(x, tf); }, shape.v);
    }

    inline m3d::vec3 shape_support(const Shape &shape, const m3d::vec3 &dir)
    {
        return std::visit([&](const auto &x) { return support(x, dir); }, shape.v);
    }

    inline int shape_face_corners(const Shape &shape, const m3d::tf &tf,
                                  const m3d::vec3 &dir, m3d::vec3 out[4])
    {
        return std::visit([&](const auto &x) { return face_corners(x, tf, dir, out); }, shape.v);
    }

    inline m3d::scalar shape_representative_radius(const Shape &shape)
    {
        return std::visit([](const auto &x) { return representative_radius(x); }, shape.v);
    }
} // namespace rbc
