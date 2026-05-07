#pragma once

/**
 * @file ContactManifoldGenerator.hpp
 * @brief Build a 1–4 point contact manifold from a single EPA result.
 * @ingroup rbc
 *
 * `manifold_detail::clip_polygon_by_plane` and `manifold_detail::reduce_to_4`
 * remain `inline` here because the visual debugger
 * (src/collision_debugger/PipelineRun.hpp) calls them directly from a TU
 * that doesn't link the manifold .cpp.
 *
 * `generate_manifold` and `gjk_epa_manifold` are declared here and
 * defined in ContactManifoldGenerator.cpp using a literal port of the
 * reference clipping algorithm (face/edge feature selection via
 * ConvexHull adjacency tables, neighbor-face boundary planes).
 */

#include "rbc/Contact.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include <algorithm>

namespace rbc
{
    namespace manifold_detail
    {
        /**
         * @brief Sutherland–Hodgman: clip polygon `poly` by half-space `plane`.
         *
         * Keeps the part of `poly` on the positive side of the plane (where
         * `dot(point - plane_p, plane_n) >= 0`). Output buffer must hold at
         * least `2 * n` entries.
         */
        inline int clip_polygon_by_plane(const m3d::vec3 *poly, int n,
                                         const m3d::vec3 &plane_n,
                                         const m3d::vec3 &plane_p,
                                         m3d::vec3 *out)
        {
            int out_n = 0;
            for (int i = 0; i < n; ++i)
            {
                const m3d::vec3 &A = poly[i];
                const m3d::vec3 &B = poly[(i + 1) % n];

                const m3d::scalar dA = m3d::dot(A - plane_p, plane_n);
                const m3d::scalar dB = m3d::dot(B - plane_p, plane_n);

                if (dA >= 0.0)
                    out[out_n++] = A;

                if ((dA > 0.0) != (dB > 0.0))
                {
                    const m3d::scalar t = dA / (dA - dB);
                    out[out_n++] = A + (B - A) * t;
                }
            }
            return out_n;
        }

        /**
         * @brief Reduce `n` contact candidates to at most 4 by maximising support area.
         *
         * Greedy: deepest, then farthest from it, then farthest from that
         * segment, then farthest from the resulting triangle.
         */
        inline int reduce_to_4(const m3d::vec3 *pts,
                               const m3d::scalar *depths,
                               int n,
                               m3d::vec3 *out_pts,
                               m3d::scalar *out_depths)
        {
            if (n <= 4)
            {
                for (int i = 0; i < n; ++i)
                {
                    out_pts[i] = pts[i];
                    out_depths[i] = depths[i];
                }
                return n;
            }

            int i0 = 0;
            for (int i = 1; i < n; ++i)
                if (depths[i] > depths[i0])
                    i0 = i;

            int i1 = (i0 == 0) ? 1 : 0;
            m3d::scalar best = m3d::length_sq(pts[i1] - pts[i0]);
            for (int i = 0; i < n; ++i)
            {
                if (i == i0) continue;
                m3d::scalar d = m3d::length_sq(pts[i] - pts[i0]);
                if (d > best) { best = d; i1 = i; }
            }

            int i2 = -1;
            best = 0.0;
            const m3d::vec3 seg = pts[i1] - pts[i0];
            for (int i = 0; i < n; ++i)
            {
                if (i == i0 || i == i1) continue;
                m3d::scalar d = m3d::length_sq(m3d::cross(pts[i] - pts[i0], seg));
                if (d > best) { best = d; i2 = i; }
            }
            if (i2 < 0)
            {
                for (int i = 0; i < n; ++i)
                    if (i != i0 && i != i1) { i2 = i; break; }
            }

            int i3 = -1;
            best = 0.0;
            if (i2 >= 0)
            {
                const m3d::vec3 tri_n = m3d::cross(pts[i1] - pts[i0], pts[i2] - pts[i0]);
                for (int i = 0; i < n; ++i)
                {
                    if (i == i0 || i == i1 || i == i2) continue;
                    m3d::scalar d = m3d::abs(m3d::dot(pts[i] - pts[i0], tri_n));
                    if (d > best) { best = d; i3 = i; }
                }
                if (i3 < 0)
                    for (int i = 0; i < n; ++i)
                        if (i != i0 && i != i1 && i != i2) { i3 = i; break; }
            }

            out_pts[0] = pts[i0]; out_depths[0] = depths[i0];
            out_pts[1] = pts[i1]; out_depths[1] = depths[i1];
            if (i2 >= 0) { out_pts[2] = pts[i2]; out_depths[2] = depths[i2]; }
            if (i3 >= 0) { out_pts[3] = pts[i3]; out_depths[3] = depths[i3]; return 4; }
            return (i2 >= 0) ? 3 : 2;
        }

    } // namespace manifold_detail

    /**
     * @brief Build a manifold from an EPA result.
     *
     * Defined in ContactManifoldGenerator.cpp.
     */
    void generate_manifold(const m3d::vec3 &epa_normal,
                           m3d::scalar epa_depth,
                           const m3d::vec3 &epa_contact,
                           const Shape &shape_a,
                           const m3d::tf &tf_a,
                           const Shape &shape_b,
                           const m3d::tf &tf_b,
                           ContactManifold &manifold);

    /**
     * @brief Run GJK → EPA → `generate_manifold` in one call.
     *
     * Returns false if GJK reports separation or EPA fails to converge.
     */
    bool gjk_epa_manifold(const Shape &sa,
                          const m3d::tf &tf_a,
                          const Shape &sb,
                          const m3d::tf &tf_b,
                          ContactManifold &manifold);

} // namespace rbc
