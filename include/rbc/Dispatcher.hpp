#pragma once

/**
 * @defgroup internals internals — Algorithm implementations
 * @brief Lower-level building blocks documented for maintainers.
 *
 * Symbols here are documented but not part of the public API: collision
 * algorithm specialisations, GJK / EPA, the constraint solver, broad-phase
 * sweep helpers, etc. They live in a separate sidebar group so casual
 * users browsing the API aren't overwhelmed.
 */

/**
 * @file Dispatcher.hpp
 * @brief Compile-time dispatch from `(Shape, Shape)` to the right collision algorithm.
 * @ingroup rbc
 *
 * The primary `CollisionAlgorithm<A, B>` template falls back to GJK + EPA +
 * `ContactManifoldGenerator` for any convex pair without an analytic
 * specialisation. Analytic algorithms (sphere–sphere, sphere–box, box–box,
 * plane vs anything, …) override the primary by partial specialisation in
 * `analytic/*.hpp` and fill the `ContactManifold` directly without going
 * through GJK.
 *
 * The runtime entry point is `dispatch(Shape, tf, Shape, tf, ContactManifold&)`,
 * which indexes a compile-time `N×N` table built from the variant's
 * alternative list.
 */

#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/Contact.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/gjk/ContactManifoldGenerator.hpp"
#include <array>
#include <cstddef>
#include <utility>
#include <variant>

namespace rbc
{

    /**
     * @brief Compile-time predicate: `true` iff `T` is a convex bounded shape.
     *
     * Forwards to the per-shape free function `is_gjk_convex(const T*)`.
     * The pointer tag avoids requiring a default constructor on every
     * shape; the value is never read.
     *
     * @ingroup rbc
     */
    template <class T>
    inline constexpr bool gjk_convex_v =
        is_gjk_convex(static_cast<const T *>(nullptr));

    /**
     * @brief Primary template: GJK + EPA + manifold generation for convex pairs.
     *
     * Only convex–convex pairs reach the GJK fallback. A non-convex pair
     * (Plane, Heightmap, or Mesh on either side) without an analytic
     * specialisation returns `false` silently — there is no useful default.
     *
     * Analytic specialisations live in `analytic/*.hpp` and override this
     * template by partial specialisation.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    template <typename A, typename B>
    struct CollisionAlgorithm
    {
        /**
         * @brief Run the algorithm; fill `manifold` and return `true` on overlap.
         */
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

    /**
     * @brief Symmetric helper: `(B, A)` reuses `(A, B)` and flips the contact normal.
     *
     * Most analytic specialisations only handle `(A, B)`; the `(B, A)`
     * direction inherits from this helper so the dispatcher gets a
     * working entry without code duplication.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    template <typename A, typename B>
    struct CollisionAlgorithmSym
    {
        /** @brief Forward to `CollisionAlgorithm<B, A>::test` and flip the normal. */
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
        /**
         * @brief One concrete dispatch function per `(A, B)` shape pair.
         *
         * Template indices come from `std::variant`'s alternative list, so
         * the runtime `shape.v.index()` and the type identity match by
         * construction.
         *
         * @ingroup internals
         */
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

        /// Function-pointer signature stored in the dispatch table.
        using DispatchFn = bool (*)(const Shape &, const m3d::tf &,
                                    const Shape &, const m3d::tf &,
                                    ContactManifold &);

        /**
         * @brief Build one row `I` of the dispatch table.
         *
         * Instantiates `dispatch_pair<I, J>` for every `J` in the column
         * index pack.
         *
         * @ingroup internals
         */
        template <std::size_t I, std::size_t... Js>
        constexpr auto make_row(std::index_sequence<Js...>)
        {
            return std::array<DispatchFn, sizeof...(Js)>{ &dispatch_pair<I, Js>... };
        }

        /**
         * @brief Expand rows over the row-index pack to produce the full N×N table.
         * @ingroup internals
         */
        template <std::size_t... Is>
        constexpr auto make_table(std::index_sequence<Is...>)
        {
            return std::array<std::array<DispatchFn, sizeof...(Is)>, sizeof...(Is)>{
                make_row<Is>(std::index_sequence<Is...>{})...
            };
        }

        /**
         * @brief Number of distinct shape kinds — single source of truth from the variant.
         *
         * Add a shape to `Shape::v` in `ShapeTypes.hpp` and the table grows
         * automatically.
         *
         * @ingroup internals
         */
        constexpr std::size_t kShapeCount = std::variant_size_v<decltype(Shape::v)>;

        /// The compile-time `N×N` dispatch table. @ingroup internals
        inline constexpr auto DispatchTable =
            make_table(std::make_index_sequence<kShapeCount>{});
    } // namespace detail

    /**
     * @brief Run the collision algorithm for `(a, b)`.
     *
     * Looks up the right `CollisionAlgorithm<A, B>::test` via the
     * compile-time dispatch table and forwards the call. This is the
     * runtime entry point used by `test_narrow_phase`.
     *
     * @example test_dispatcher.cpp
     *
     * @ingroup rbc
     */
    inline bool dispatch(const Shape     &a, const m3d::tf &tf_a,
                         const Shape     &b, const m3d::tf &tf_b,
                         ContactManifold &out)
    {
        return detail::DispatchTable[a.v.index()][b.v.index()](a, tf_a, b, tf_b, out);
    }

} // namespace rbc
