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
#include <variant>

namespace rbc
{

    // ── Primary template: GJK/EPA + manifold generation ──────────────────────
    template <typename A, typename B>
    struct CollisionAlgorithm
    {
        static bool test(const A       &a, const m3d::tf &tf_a,
                         const B       &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            Shape sa = a, sb = b;
            return gjk_epa_manifold(sa, tf_a, sb, tf_b, manifold);
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
//#include "rbc/analytic/SphereCapsule.hpp"
//#include "rbc/analytic/CapsuleCapsule.hpp"
//#include "rbc/analytic/PlaneCollision.hpp"
// #include "rbc/analytic/MeshCollision.hpp"
//#include "rbc/analytic/HeightmapCollision.hpp"

namespace rbc
{
    // ── Dispatch wrapper ──────────────────────────────────────────────────────
    template <typename A, typename B>
    bool dispatch_wrapper(const Shape     &a, const m3d::tf &tf_a,
                          const Shape     &b, const m3d::tf &tf_b,
                          ContactManifold &out)
    {
        return CollisionAlgorithm<A, B>::test(a.get<A>(), tf_a, b.get<B>(), tf_b, out);
    }

    using CollisionFunc = bool (*)(const Shape &, const m3d::tf &,
                                   const Shape &, const m3d::tf &,
                                   ContactManifold &);

#define GENERATE_ROW(InnerType, InnerName, FixedOuterType) \
    dispatch_wrapper<FixedOuterType, InnerType>,

#define GENERATE_MATRIX_LINE(OuterType, OuterName) \
    {RBC_SHAPE_LIST_INNER(GENERATE_ROW, OuterType)},

    static constexpr CollisionFunc DispatchTable
        [static_cast<int>(ShapeType::Count)]
        [static_cast<int>(ShapeType::Count)] =
    {
        RBC_SHAPE_LIST(GENERATE_MATRIX_LINE)
    };

    inline bool dispatch(const Shape     &a, const m3d::tf &tf_a,
                         const Shape     &b, const m3d::tf &tf_b,
                         ContactManifold &out)
    {
        return DispatchTable
            [static_cast<int>(a.type)]
            [static_cast<int>(b.type)](a, tf_a, b, tf_b, out);
    }

} // namespace rbc