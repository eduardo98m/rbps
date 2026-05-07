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
#include <math3d/math3d.hpp>

namespace rbc
{
    namespace
    {
        // Grow the simplex toward rank 4 by sampling supports along
        // directions perpendicular to the current simplex feature.
        // Used when the GJK main loop's search direction collapses to
        // zero (origin lies on the current simplex line/triangle) so
        // EPA receives a rank-4 seed even when the contact is purely
        // touching. A coplanar resulting tetrahedron is fine — EPA's
        // coplanar-seed branch handles it.
        void grow_simplex_to_rank4(Simplex &simplex, const MinkowskiDiff &shape,
                                    m3d::scalar tolerance)
        {
            while (simplex.rank < 4)
            {
                m3d::vec3 perp(0, 0, 0);

                if (simplex.rank == 1)
                {
                    perp = m3d::vec3(1, 0, 0);
                }
                else if (simplex.rank == 2)
                {
                    const m3d::vec3 ab = simplex.vertex[1]->w - simplex.vertex[0]->w;
                    const m3d::scalar ax = m3d::abs(ab.x);
                    const m3d::scalar ay = m3d::abs(ab.y);
                    const m3d::scalar az = m3d::abs(ab.z);
                    const m3d::vec3 axis = (ax <= ay && ax <= az)
                                             ? m3d::vec3(1, 0, 0)
                                             : (ay <= az ? m3d::vec3(0, 1, 0)
                                                         : m3d::vec3(0, 0, 1));
                    perp = m3d::cross(ab, axis);
                    if (m3d::length_sq(perp) < m3d::EPSILON * m3d::EPSILON)
                        perp = m3d::cross(ab, m3d::vec3(0, 0, 1));
                }
                else // rank == 3
                {
                    const m3d::vec3 ab = simplex.vertex[1]->w - simplex.vertex[0]->w;
                    const m3d::vec3 ac = simplex.vertex[2]->w - simplex.vertex[0]->w;
                    perp = m3d::cross(ab, ac);
                }

                if (m3d::length_sq(perp) < m3d::EPSILON * m3d::EPSILON)
                    break;

                SimplexVertex p1, p2;
                p1.w0 = shape.support_a( perp); p1.w1 = shape.support_b(-perp); p1.w = p1.w0 - p1.w1;
                p2.w0 = shape.support_a(-perp); p2.w1 = shape.support_b( perp); p2.w = p2.w0 - p2.w1;

                const m3d::vec3 anchor = simplex.vertex[0]->w;
                const m3d::scalar d1 = m3d::dot(p1.w - anchor,  perp);
                const m3d::scalar d2 = m3d::dot(p2.w - anchor, -perp);

                // Pick the side with the greater unsigned extent.
                const SimplexVertex &chosen = (d1 >= d2) ? p1 : p2;

                // Skip if the chosen point coincides with an existing
                // simplex vertex (would not grow rank).
                bool dup = false;
                for (int i = 0; i < simplex.rank; ++i)
                {
                    if (m3d::length_sq(simplex.vertex[i]->w - chosen.w) < tolerance * tolerance)
                    {
                        dup = true;
                        break;
                    }
                }
                if (dup) break;

                add_to_simplex(simplex, chosen);
            }
        }
    } // namespace

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
                // Origin lies on the current simplex feature — the
                // shapes are touching exactly (or nearly so). Grow the
                // simplex with perpendicular supports so EPA receives
                // a rank-4 seed and can take its coplanar-seed branch.
                grow_simplex_to_rank4(simplex, shape, tolerance);
                status = Inside;
                return status;
            }
        }

        status = Failed;
        return status;
    }

} // namespace rbc
