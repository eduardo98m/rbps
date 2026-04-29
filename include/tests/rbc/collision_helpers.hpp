#pragma once
#include "rbc/Dispatcher.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "tests/transform_helpers.hpp"   // test::tf_at (shared across rbc + rbps)

// ── rbc collision test helpers ──────────────────────────────────────────────
// Replaces the gjk_reference() copy-paste duplicated across 5 analytic tests
// and the m3d::tf-construction / .get<T>() / .points[0].x boilerplate.
//
// All free functions, all inline (header-only); ADL still finds them with a
// `test::` prefix at call sites for clarity.
namespace test
{
    // GJK + EPA cross-validation. Runs the generic Minkowski-diff GJK fallback
    // on any pair of shapes and fills `out` as a single-point ContactManifold.
    // Returns false if shapes don't penetrate.
    inline bool gjk_reference(const rbc::Shape &sa, const m3d::tf &tfa,
                              const rbc::Shape &sb, const m3d::tf &tfb,
                              rbc::ContactManifold &out)
    {
        rbc::MinkowskiDiff md(&sa, &sb, tfa, tfb);
        m3d::vec3 guess = tfb.pos - tfa.pos;
        if (m3d::length_sq(guess) < m3d::EPSILON)
            guess = m3d::vec3(1.0, 0.0, 0.0);

        rbc::GJK gjk;
        if (gjk.evaluate(md, guess) != rbc::GJK::Inside)
            return false;

        rbc::EPA epa;
        if (epa.evaluate(gjk, md) != rbc::EPA::Valid)
            return false;

        out.normal                       = epa.normal;
        out.num_points                   = 1;
        out.points[0].penetration_depth  = epa.depth;
        out.points[0].position           = epa.contact_point;
        return true;
    }

    // Type-deduced wrapper around CollisionAlgorithm<A, B>::test that takes
    // Shape-wrapped operands. Avoids the .get<A>() / .get<B>() boilerplate.
    template <class A, class B>
    inline bool collide(const rbc::Shape &sa, const m3d::tf &tfa,
                        const rbc::Shape &sb, const m3d::tf &tfb,
                        rbc::ContactManifold &out)
    {
        return rbc::CollisionAlgorithm<A, B>::test(
            sa.get<A>(), tfa, sb.get<B>(), tfb, out);
    }

    // Single-point manifold accessors (the common case in analytic tests).
    inline m3d::scalar depth(const rbc::ContactManifold &m)
    {
        return m.points[0].penetration_depth;
    }
    inline m3d::vec3 contact_position(const rbc::ContactManifold &m)
    {
        return m.points[0].position;
    }
} // namespace test
