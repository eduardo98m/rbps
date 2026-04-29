#pragma once
// ============================================================================
//  Dispatcher.hpp
//
//  Routes each shape-pair to the correct CollisionAlgorithm specialisation.
//  The primary template falls back to GJK + EPA + ContactManifoldGenerator
//  which gives a proper 1-4 point manifold even for non-analytic pairs.
//
//  Analytic specialisations (SphereSphere, SphereBox, BoxBox, …) should
//  fill the ContactManifold directly and bypass GJK/EPA entirely.
// ============================================================================

#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/Contact.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/gjk/ContactManifoldGenerator.hpp"   // gjk_epa_manifold + generate_manifold
#include <array>
#include <cstddef>
#include <utility>
#include <variant>

namespace rbc
{

    // Compile-time convexity query. Forwards to the per-shape free function
    // is_gjk_convex(const T*) defined next to each shape struct. Using a
    // pointer tag (rather than a value) avoids requiring constexpr default
    // constructors on every shape.
    template <class T>
    inline constexpr bool gjk_convex_v =
        is_gjk_convex(static_cast<const T *>(nullptr));

    // ── Primary template: GJK/EPA + manifold generation ──────────────────────
    // Only convex pairs reach the GJK fallback. Non-convex pairs (Plane,
    // Heightmap, Mesh on either side) without an analytic specialisation
    // return false silently — there is no useful default for them.
    template <typename A, typename B>
    struct CollisionAlgorithm
    {
        static bool test(const A       &a, const m3d::tf &tf_a,
                         const B       &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            if constexpr (gjk_convex_v<A> && gjk_convex_v<B>)
            {
                Shape sa = a, sb = b;
                return gjk_epa_manifold(sa, tf_a, sb, tf_b, manifold);
            }
            else
            {
                (void)a; (void)b; (void)tf_a; (void)tf_b; (void)manifold;
                return false;
            }
        }
    };

    // ── Symmetric helper: (B,A) reuses (A,B) and flips the normal ────────────
    template <typename A, typename B>
    struct CollisionAlgorithmSym
    {
        static bool test(const A       &a, const m3d::tf &tf_a,
                         const B       &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            bool hit = CollisionAlgorithm<B, A>::test(b, tf_b, a, tf_a, manifold);
            if (hit)
                manifold.normal = -manifold.normal;
            return hit;
        }
    };

} // namespace rbc

// ── Analytic specialisations ─────────────────────────────────────────────────
#include "rbc/analytic/SphereSphere.hpp"
#include "rbc/analytic/SphereBox.hpp"
#include "rbc/analytic/BoxBox.hpp"
#include "rbc/analytic/SphereCapsule.hpp"
#include "rbc/analytic/CapsuleCapsule.hpp"
#include "rbc/analytic/CapsuleBox.hpp"
#include "rbc/analytic/PlaneCollision.hpp"
#include "rbc/analytic/MeshCollision.hpp"
#include "rbc/analytic/HeightmapCollision.hpp"

namespace rbc
{
    namespace detail
    {
        // One concrete dispatch function per (A, B) pair. The template indices
        // come from std::variant's alternative list so the runtime
        // shape.v.index() and the type identity match by construction.
        template <std::size_t I, std::size_t J>
        bool dispatch_pair(const Shape     &a, const m3d::tf &tf_a,
                           const Shape     &b, const m3d::tf &tf_b,
                           ContactManifold &out)
        {
            using A = std::variant_alternative_t<I, decltype(Shape::v)>;
            using B = std::variant_alternative_t<J, decltype(Shape::v)>;
            return CollisionAlgorithm<A, B>::test(
                std::get<A>(a.v), tf_a, std::get<B>(b.v), tf_b, out);
        }

        using DispatchFn = bool (*)(const Shape &, const m3d::tf &,
                                    const Shape &, const m3d::tf &,
                                    ContactManifold &);

        // Build one row of the table for fixed row index I: instantiate
        // dispatch_pair<I, J> for every J in the column index pack.
        template <std::size_t I, std::size_t... Js>
        constexpr auto make_row(std::index_sequence<Js...>)
        {
            return std::array<DispatchFn, sizeof...(Js)>{ &dispatch_pair<I, Js>... };
        }

        // Expand rows over the row-index pack to produce the full N×N table.
        template <std::size_t... Is>
        constexpr auto make_table(std::index_sequence<Is...>)
        {
            return std::array<std::array<DispatchFn, sizeof...(Is)>, sizeof...(Is)>{
                make_row<Is>(std::index_sequence<Is...>{})...
            };
        }

        // N comes from the variant — single source of truth. Add a shape to
        // the variant in ShapeTypes.hpp and the table grows automatically.
        constexpr std::size_t kShapeCount = std::variant_size_v<decltype(Shape::v)>;

        inline constexpr auto DispatchTable =
            make_table(std::make_index_sequence<kShapeCount>{});
    } // namespace detail

    inline bool dispatch(const Shape     &a, const m3d::tf &tf_a,
                         const Shape     &b, const m3d::tf &tf_b,
                         ContactManifold &out)
    {
        return detail::DispatchTable[a.v.index()][b.v.index()](a, tf_a, b, tf_b, out);
    }

} // namespace rbc