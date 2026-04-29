#pragma once
// ============================================================================
//  ContactManifoldGenerator.hpp
//
//  Given an EPA result (one point, one normal, one depth) this generates a
//  proper 1-4 point manifold by:
//
//   1. Finding the REFERENCE face (most aligned with normal) on shape A.
//   2. Finding the INCIDENT face (most anti-aligned) on shape B.
//   3. Clipping the incident face polygon against the reference face's side
//      planes (Sutherland-Hodgman), then depth-testing against the reference
//      face plane.
//   4. Reducing to ≤4 contact points.
//
//  For primitive pairs that have an analytic fast-path (SphereSphere,
//  SphereBox, etc.) the analytic algorithm should fill the manifold directly
//  and not call this generator.
//
//  This is used by the GJK/EPA *fallback* path in CollisionAlgorithm<A,B>.
// ============================================================================

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

        // Reduce n contact candidates to ≤4 by maximising the support area.
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

    // ---------------------------------------------------------------------------
    //  Main entry point
    // ---------------------------------------------------------------------------

    // Expand a single-point EPA result into a full manifold.
    // `epa_normal`   — penetration normal (A→B convention from EPA)
    // `epa_depth`    — penetration depth
    // `epa_contact`  — EPA contact point (barycentric on the Minkowski diff face)
    // `shape_a/b`    — the two shapes
    // `tf_a/tf_b`    — their world transforms
    //
    // The function fills `manifold` with 1-4 contact points.
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

    // ---------------------------------------------------------------------------
    //  Convenience: run GJK → EPA → generate_manifold in one call.
    //  This is what the primary CollisionAlgorithm<A,B> template should call
    //  instead of filling manifold manually.
    // ---------------------------------------------------------------------------
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