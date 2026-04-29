#pragma once

/**
 * @file ContactManifoldGenerator.hpp
 * @brief Build a 1–4 point contact manifold from a single EPA result.
 * @ingroup rbc
 * @ingroup internals
 *
 * Given an EPA result (one normal, one depth, one contact point) the
 * generator produces a proper 1–4 point manifold by:
 * 1. Selecting the **reference** face on shape A (the face most aligned
 *    with the EPA normal) via `face_corners`.
 * 2. Selecting the **incident** face on shape B (most anti-aligned).
 * 3. Clipping the incident polygon against the reference face's side
 *    planes (Sutherland–Hodgman) and depth-testing against the
 *    reference face plane.
 * 4. Reducing the surviving points to at most 4 by maximising support
 *    area (deepest, then farthest, then farthest from segment, then
 *    farthest from triangle).
 *
 * Pairs with an analytic fast path (`SphereSphere`, `SphereBox`, …)
 * fill the manifold directly and skip this generator. The path through
 * here is the GJK/EPA fallback used by the primary `CollisionAlgorithm<A, B>`
 * template.
 */

#include "rbc/Contact.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include <cstring>
#include <algorithm>

namespace rbc
{

    // ---------------------------------------------------------------------------
    //  Sutherland-Hodgman clip (shared with BoxBox.hpp — defined here as inline
    //  so there is no ODR violation; BoxBox.hpp has its own copy in detail::)
    // ---------------------------------------------------------------------------

    namespace manifold_detail
    {

        /**
         * @brief Sutherland–Hodgman: clip polygon `poly` by half-space `plane`.
         *
         * Keeps the part of `poly` on the positive side of the plane (where
         * `dot(point - plane_p, plane_n) >= 0`).
         *
         * @param poly   Input polygon vertices in order.
         * @param n      Vertex count of `poly`.
         * @param plane_n Plane normal (need not be unit length).
         * @param plane_p Any point on the plane.
         * @param[out] out Output vertices; must have room for at least `2 * n` entries.
         * @return Number of vertices written to `out`.
         *
         * @ingroup internals
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
         * Greedy selection: pick the deepest first, then the farthest from
         * it, then the farthest from that segment, then the farthest from
         * the resulting triangle. The 4-point output is the most stable
         * support set for an XPBD or impulse solver.
         *
         * @ingroup internals
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

            // 1. Deepest
            int i0 = 0;
            for (int i = 1; i < n; ++i)
                if (depths[i] > depths[i0])
                    i0 = i;

            // 2. Farthest from i0
            int i1 = (i0 == 0) ? 1 : 0;
            m3d::scalar best = m3d::length_sq(pts[i1] - pts[i0]);
            for (int i = 0; i < n; ++i)
            {
                if (i == i0)
                    continue;
                m3d::scalar d = m3d::length_sq(pts[i] - pts[i0]);
                if (d > best)
                {
                    best = d;
                    i1 = i;
                }
            }

            // 3. Farthest from segment
            int i2 = -1;
            best = 0.0;
            const m3d::vec3 seg = pts[i1] - pts[i0];
            for (int i = 0; i < n; ++i)
            {
                if (i == i0 || i == i1)
                    continue;
                m3d::scalar d = m3d::length_sq(m3d::cross(pts[i] - pts[i0], seg));
                if (d > best)
                {
                    best = d;
                    i2 = i;
                }
            }
            if (i2 < 0)
            {
                for (int i = 0; i < n; ++i)
                    if (i != i0 && i != i1)
                    {
                        i2 = i;
                        break;
                    }
            }

            // 4. Farthest from triangle
            int i3 = -1;
            best = 0.0;
            if (i2 >= 0)
            {
                const m3d::vec3 tri_n = m3d::cross(pts[i1] - pts[i0], pts[i2] - pts[i0]);
                for (int i = 0; i < n; ++i)
                {
                    if (i == i0 || i == i1 || i == i2)
                        continue;
                    m3d::scalar d = m3d::abs(m3d::dot(pts[i] - pts[i0], tri_n));
                    if (d > best)
                    {
                        best = d;
                        i3 = i;
                    }
                }
                if (i3 < 0)
                    for (int i = 0; i < n; ++i)
                        if (i != i0 && i != i1 && i != i2)
                        {
                            i3 = i;
                            break;
                        }
            }

            out_pts[0] = pts[i0];
            out_depths[0] = depths[i0];
            out_pts[1] = pts[i1];
            out_depths[1] = depths[i1];
            if (i2 >= 0)
            {
                out_pts[2] = pts[i2];
                out_depths[2] = depths[i2];
            }
            if (i3 >= 0)
            {
                out_pts[3] = pts[i3];
                out_depths[3] = depths[i3];
                return 4;
            }
            return (i2 >= 0) ? 3 : 2;
        }

    } // namespace manifold_detail

    /**
     * @brief Expand a single-point EPA result into a 1–4 point manifold.
     *
     * Picks the reference face on `shape_a` (most aligned with `epa_normal`)
     * and the incident face on `shape_b` (most anti-aligned), clips the
     * incident polygon against the reference face's side planes, then
     * reduces to ≤ 4 contacts. Falls back to the EPA single-point result
     * if clipping eliminates everything.
     *
     * @param epa_normal  Penetration normal from EPA (A→B convention).
     * @param epa_depth   Penetration depth from EPA (positive on overlap).
     * @param epa_contact EPA contact point in world space.
     * @param shape_a     First shape.
     * @param tf_a        Transform of `shape_a`.
     * @param shape_b     Second shape.
     * @param tf_b        Transform of `shape_b`.
     * @param[out] manifold Filled with up to 4 contact points.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    inline void generate_manifold(const m3d::vec3 &epa_normal,
                                  m3d::scalar epa_depth,
                                  const m3d::vec3 &epa_contact,
                                  const Shape &shape_a,
                                  const m3d::tf &tf_a,
                                  const Shape &shape_b,
                                  const m3d::tf &tf_b,
                                  ContactManifold &manifold)
    {
        manifold.normal = epa_normal;
        manifold.num_points = 0;

        // ── Determine reference and incident "face polygons" ──────────────────
        // Each shape supplies its own face_corners() — Box gives exact
        // corners, others fall back to a disc approximation. Variant-level
        // dispatch lives in shape_face_corners (ShapeTypes.hpp).
        m3d::vec3 ref_corners[4], inc_corners[4];
        const int ref_n = shape_face_corners(shape_a, tf_a,  epa_normal, ref_corners);
        const int inc_n = shape_face_corners(shape_b, tf_b, -epa_normal, inc_corners);

        // ── Sutherland-Hodgman: clip incident polygon against reference face ──
        // Build the reference face normal (outward = epa_normal direction)
        // The reference face plane passes through ref_corners[0].
        const m3d::vec3 ref_face_n = epa_normal; // outward normal of reference face
        const m3d::scalar ref_d = m3d::dot(ref_corners[0], ref_face_n);

        // For the side planes we need to know the face "tangent frame".
        // We approximate with the two axes perpendicular to ref_face_n.
        // Use the winding of ref_corners to build edge normals.
        m3d::vec3 buf0[16], buf1[16];
        int cnt = inc_n;
        for (int i = 0; i < inc_n; ++i)
            buf0[i] = inc_corners[i];

        for (int i = 0; i < ref_n && cnt > 0; ++i)
        {
            const m3d::vec3 edge = ref_corners[(i + 1) % ref_n] - ref_corners[i];
            const m3d::vec3 side_n = m3d::normalize(m3d::cross(ref_face_n, edge));
            cnt = manifold_detail::clip_polygon_by_plane(
                buf0, cnt, side_n, ref_corners[i], buf1);
            for (int k = 0; k < cnt; ++k)
                buf0[k] = buf1[k];
        }

        // Keep only points that are on or behind the reference face plane
        m3d::vec3 keep_pts[16];
        m3d::scalar keep_dep[16];
        int keep_n = 0;

        for (int i = 0; i < cnt; ++i)
        {
            const m3d::scalar sd = m3d::dot(buf0[i], ref_face_n) - ref_d;
            if (sd <= m3d::EPSILON) // on or below reference face
            {
                keep_pts[keep_n] = buf0[i];
                // Penetration depth for this point along the normal.
                // A rough estimate: use the global EPA depth (conservative).
                // More precise: distance from the point to shape A's reference face.
                keep_dep[keep_n] = (sd < 0.0) ? (-sd) : epa_depth;
                ++keep_n;
            }
        }

        // Fall back to the single EPA contact point if clipping gave nothing
        if (keep_n == 0)
        {
            manifold.num_points = 1;
            manifold.points[0].position = epa_contact;
            manifold.points[0].penetration_depth = epa_depth;
            return;
        }

        // Reduce to ≤4
        m3d::vec3 red_pts[4];
        m3d::scalar red_dep[4];
        int red_n = manifold_detail::reduce_to_4(
            keep_pts, keep_dep, keep_n, red_pts, red_dep);

        manifold.num_points = red_n;
        for (int i = 0; i < red_n; ++i)
        {
            manifold.points[i].position = red_pts[i];
            manifold.points[i].penetration_depth = red_dep[i];
        }
    }

    /**
     * @brief Convenience: run GJK → EPA → `generate_manifold` in one call.
     *
     * The primary `CollisionAlgorithm<A, B>` template calls this for any
     * convex pair that doesn't have an analytic specialisation.
     *
     * @return `false` if GJK reports the shapes are separated or EPA
     *         fails to converge; `true` (and `manifold` is filled) on overlap.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    inline bool gjk_epa_manifold(const Shape &sa,
                                 const m3d::tf &tf_a,
                                 const Shape &sb,
                                 const m3d::tf &tf_b,
                                 ContactManifold &manifold)
    {
        MinkowskiDiff md(&sa, &sb, tf_a, tf_b);

        m3d::vec3 guess = tf_b.pos - tf_a.pos;
        if (m3d::length_sq(guess) < m3d::EPSILON)
            guess = m3d::vec3(1.0, 0.0, 0.0);

        GJK gjk;
        if (gjk.evaluate(md, guess) != GJK::Inside)
            return false;

        EPA epa;
        if (epa.evaluate(gjk, md) != EPA::Valid)
            return false;

        generate_manifold(epa.normal, epa.depth, epa.contact_point,
                          sa, tf_a, sb, tf_b, manifold);
        return true;
    }

} // namespace rbc