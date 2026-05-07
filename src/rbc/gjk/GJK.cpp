// GJK — overlap test for convex shapes via Minkowski-difference origin search.
//
// Reference algorithm (Ericson, Real-Time Collision Detection §9.5):
//   1. Initialize the simplex set Q to one or more points (up to d + 1
//      points, where d is the dimension) from the Minkowski difference
//      of A and B.
//   2. Compute the point P of minimum norm in CH(Q).
//   3. If P is the origin itself, the origin is clearly contained in
//      the Minkowski difference of A and B. Stop and return A and B
//      as intersecting.
//   4. Reduce Q to the smallest subset Q' of Q such that P ∈ CH(Q').
//      That is, remove any points from Q not determining the
//      subsimplex of Q in which P lies.
//   5. Let V = sA−B(−P) = sA(−P) − sB(P) be a supporting point in
//      direction −P.
//   6. If V is no more extremal in direction −P than P itself, stop
//      and return A and B as not intersecting. The length of the
//      vector from the origin to P is the separation distance of
//      A and B.
//   7. Add V to Q and go to 2.
//
// Steps 2–4 are implemented by `do_simplex` (per-rank handlers in
// `simplex/Line.cpp`, `Triangle.cpp`, `Tetrahedron.cpp`); step 5 by the
// MinkowskiDiff support call; step 6 by the early-out below.

#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/simplex/DoSimplex.hpp"

namespace rbc
{
    GJK::GJK()
        : status(Failed),
          current_shape(nullptr),
          ray(0.0, 0.0, 0.0),
          distance(0.0),
          active_simplex(nullptr)
    {
        simplex.reset();
    }

    GJK::Status GJK::evaluate(const MinkowskiDiff &shape, const m3d::vec3 &initial_guess)
    {
        current_shape  = &shape;
        active_simplex = &simplex;
        simplex.reset();

        m3d::vec3 dir = (m3d::length_sq(initial_guess) > m3d::EPSILON)
                          ? initial_guess
                          : m3d::vec3(1.0, 0.0, 0.0);

        // Step 1 — seed the simplex with one support sample.
        SimplexVertex v0;
        v0.w0 = shape.support_a(dir);
        v0.w1 = shape.support_b(-dir);
        v0.w  = v0.w0 - v0.w1;
        add_to_simplex(simplex, v0);
        dir      = -v0.w;
        ray      = dir;
        distance = m3d::length(ray);

        for (unsigned int it = 0; it < max_iterations; ++it)
        {
            // Step 5 — sample the support along the current direction.
            SimplexVertex next;
            next.w0 = shape.support_a(dir);
            next.w1 = shape.support_b(-dir);
            next.w  = next.w0 - next.w1;

            // Step 6 — early out when the new support cannot reach the origin.
            if (m3d::dot(next.w, dir) < 0.0)
            {
                ray      = dir;
                distance = m3d::length(dir);
                status   = Valid;
                return status;
            }

            add_to_simplex(simplex, next);

            // Steps 2–4 — reduce the simplex and update the search direction.
            if (do_simplex(simplex, dir))
            {
                ray      = m3d::vec3(0.0, 0.0, 0.0);
                distance = 0.0;
                status   = Inside;
                return status;
            }

            ray      = dir;
            distance = m3d::length(dir);

            if (m3d::length_sq(dir) < tolerance * tolerance)
            {
                status = Inside;
                return status;
            }
        }

        status = Failed;
        return status;
    }

} // namespace rbc
