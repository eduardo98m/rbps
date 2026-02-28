#pragma once
#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/Contact.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include <variant>

namespace rbc
{

    //  Primary template: GJK/EPA fallback for any unspecialised pair
    template <typename A, typename B>
    struct CollisionAlgorithm
    {
        static bool test(const A &a, const m3d::tf &tf_a,
                         const B &b, const m3d::tf &tf_b,
                         Contact &out)
        {
            Shape sa = a, sb = b;
            MinkowskiDiff md(&sa, &sb, tf_a, tf_b);

            // Guard against zero-length initial guess (when centers coincide)
            m3d::vec3 guess = tf_b.pos - tf_a.pos;
            if (m3d::length_sq(guess) < m3d::EPSILON)
                guess = m3d::vec3(1.0, 0.0, 0.0);

            GJK gjk;
            if (gjk.evaluate(md, guess) != GJK::Inside)
                return false;

            EPA epa;
            if (epa.evaluate(gjk, md) != EPA::Valid)
                return false;

            out.normal = epa.normal;
            out.penetration_depth = epa.depth;
            out.pos = epa.contact_point;
            return true;
        }
    };

    // Symmetric helper: (B,A) reuses (A,B) and flips normal
    template <typename A, typename B>
    struct CollisionAlgorithmSym
    {
        static bool test(const A &a, const m3d::tf &tf_a,
                         const B &b, const m3d::tf &tf_b,
                         Contact &out)
        {
            bool hit = CollisionAlgorithm<B, A>::test(b, tf_b, a, tf_a, out);
            if (hit)
                out.normal = -out.normal;
            return hit;
        }
    };
}

#include "rbc/analytic/SphereSphere.hpp"
#include "rbc/analytic/SphereBox.hpp"
#include "rbc/analytic/BoxBox.hpp"

namespace rbc
{
    // Generic Wrapper: Extracts the correct types using get `get<T>()`
    // and calls the correct CollisionAlgorithm specialization. This is what the dispatch table will call.
    template <typename A, typename B>
    bool dispatch_wrapper(const Shape &a, const m3d::tf &tf_a,
                          const Shape &b, const m3d::tf &tf_b,
                          Contact &out)
    {
        return CollisionAlgorithm<A, B>::test(a.get<A>(), tf_a, b.get<B>(), tf_b, out);
    }

    using CollisionFunc = bool (*)(const Shape &, const m3d::tf &, const Shape &, const m3d::tf &, Contact &);

    /**
     * 2D DISPATCH MATRIX GENERATION (X-Macros)
     * ---------------------------------------
     * We use a nested expansion technique to automatically generate a N x N
     * lookup table of function pointers.
     * 2. GENERATE_ROW: Creates a single entry: dispatch_wrapper<RowType, ColType>.
     * 3. GENERATE_MATRIX_LINE: Wraps a full horizontal expansion in braces { ... }.
     */
#define GENERATE_ROW(InnerType, InnerName, FixedOuterType) \
    dispatch_wrapper<FixedOuterType, InnerType>,

#define GENERATE_MATRIX_LINE(OuterType, OuterName) \
    {RBC_SHAPE_LIST_INNER(GENERATE_ROW, OuterType)},

    static constexpr CollisionFunc DispatchTable[static_cast<int>(ShapeType::Count)][static_cast<int>(ShapeType::Count)] = {
        RBC_SHAPE_LIST(GENERATE_MATRIX_LINE)};

    inline bool dispatch(const Shape &a, const m3d::tf &tf_a,
                         const Shape &b, const m3d::tf &tf_b,
                         Contact &out)
    {
        return DispatchTable[static_cast<int>(a.type)][static_cast<int>(b.type)](a, tf_a, b, tf_b, out);
    }

} // namespace rbc