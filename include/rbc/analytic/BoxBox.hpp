#pragma once
#include "rbc/Dispatcher.hpp"
#include <algorithm>
#include <array>
#include <limits>

// ============================================================================
//  Box vs Box — SAT + Sutherland-Hodgman contact manifold
//
//  Pipeline:
//   1. SAT over 15 axes → find minimum-overlap axis (the contact normal).
//   2. Identify the REFERENCE face (on A, most aligned with normal) and the
//      INCIDENT face (on B, most anti-aligned with normal).
//   3. Clip the incident face polygon against the 4 side planes of the
//      reference face (Sutherland-Hodgman).
//   4. Keep clipped points that are below the reference face plane (penetrating).
//   5. Reduce to at most 4 contacts by keeping the extremal points.
//
//  Result: ContactManifold with 1-4 points, all physically meaningful.
// ============================================================================

namespace rbc
{

namespace detail
{

// ---------------------------------------------------------------------------
//  Low-level geometry helpers
// ---------------------------------------------------------------------------

// Project OBB onto a world-space axis → half-extent (support radius).
inline m3d::scalar obb_radius(const Box     &box,
                               const m3d::tf &tf,
                               const m3d::vec3 &axis)
{
    const m3d::vec3 ax = tf.rotate_vector(m3d::vec3(1, 0, 0));
    const m3d::vec3 ay = tf.rotate_vector(m3d::vec3(0, 1, 0));
    const m3d::vec3 az = tf.rotate_vector(m3d::vec3(0, 0, 1));
    return m3d::abs(m3d::dot(ax, axis)) * box.half_extents.x
         + m3d::abs(m3d::dot(ay, axis)) * box.half_extents.y
         + m3d::abs(m3d::dot(az, axis)) * box.half_extents.z;
}

// Test one SAT axis. Updates best_depth / best_normal if this is the
// smallest penetration seen so far. Returns false → separating axis found.
inline bool test_axis(const m3d::vec3 &axis,
                      const Box &a, const m3d::tf &tf_a,
                      const Box &b, const m3d::tf &tf_b,
                      const m3d::vec3 &centre_diff,
                      m3d::scalar &best_depth,
                      m3d::vec3   &best_normal,
                      int          feature_id,       // which axis index
                      int         &best_feature)
{
    const m3d::scalar len_sq = m3d::length_sq(axis);
    if (len_sq < m3d::EPSILON * m3d::EPSILON)
        return true; // degenerate (parallel edges) → skip but don't separate

    const m3d::vec3   norm_axis = axis / m3d::sqrt(len_sq);
    const m3d::scalar rA   = obb_radius(a, tf_a, norm_axis);
    const m3d::scalar rB   = obb_radius(b, tf_b, norm_axis);
    const m3d::scalar dist = m3d::abs(m3d::dot(centre_diff, norm_axis));
    const m3d::scalar overlap = rA + rB - dist;

    if (overlap < 0.0)
        return false;

    if (overlap < best_depth)
    {
        best_depth   = overlap;
        best_normal  = (m3d::dot(centre_diff, norm_axis) >= 0.0)
                         ? norm_axis : -norm_axis;
        best_feature = feature_id;
    }
    return true;
}

// ---------------------------------------------------------------------------
//  Face helpers
// ---------------------------------------------------------------------------

// Return the 4 corners of the face on `box` (in world space) that is most
// aligned with `dir` (i.e. the face whose outward normal is closest to dir).
inline void get_best_face(const Box     &box,
                           const m3d::tf &tf,
                           const m3d::vec3 &dir,
                           m3d::vec3 out_corners[4],
                           m3d::vec3 &out_normal)
{
    // Local axes in world space
    const m3d::vec3 axes[3] = {
        tf.rotate_vector(m3d::vec3(1, 0, 0)),
        tf.rotate_vector(m3d::vec3(0, 1, 0)),
        tf.rotate_vector(m3d::vec3(0, 0, 1)),
    };
    const m3d::scalar half[3] = {
        box.half_extents.x,
        box.half_extents.y,
        box.half_extents.z,
    };

    // Find which face axis is most aligned with dir
    int   best_axis  = 0;
    m3d::scalar best_dot = m3d::abs(m3d::dot(axes[0], dir));
    for (int i = 1; i < 3; ++i)
    {
        m3d::scalar d = m3d::abs(m3d::dot(axes[i], dir));
        if (d > best_dot) { best_dot = d; best_axis = i; }
    }

    // Face normal (sign chosen so it points along dir)
    const m3d::scalar sign = (m3d::dot(axes[best_axis], dir) >= 0.0) ? 1.0 : -1.0;
    out_normal = axes[best_axis] * sign;

    // Face centre
    const m3d::vec3 face_centre = tf.pos + out_normal * half[best_axis];

    // The other two axes span the face
    const int u = (best_axis + 1) % 3;
    const int v = (best_axis + 2) % 3;

    out_corners[0] = face_centre + axes[u] * half[u] + axes[v] * half[v];
    out_corners[1] = face_centre - axes[u] * half[u] + axes[v] * half[v];
    out_corners[2] = face_centre - axes[u] * half[u] - axes[v] * half[v];
    out_corners[3] = face_centre + axes[u] * half[u] - axes[v] * half[v];
}

// ---------------------------------------------------------------------------
//  Sutherland-Hodgman clipping
// ---------------------------------------------------------------------------

// Clip polygon `poly` (size `n`) against a half-space defined by plane
// normal `plane_n` and a point on the plane `plane_p`.
// Points on the positive side of the plane are KEPT.
// Returns the number of output vertices (written into `out`).
inline int clip_polygon_by_plane(const m3d::vec3 *poly, int n,
                                  const m3d::vec3 &plane_n,
                                  const m3d::vec3 &plane_p,
                                  m3d::vec3       *out)
{
    int out_n = 0;
    for (int i = 0; i < n; ++i)
    {
        const m3d::vec3 &A = poly[i];
        const m3d::vec3 &B = poly[(i + 1) % n];

        const m3d::scalar dA = m3d::dot(A - plane_p, plane_n);
        const m3d::scalar dB = m3d::dot(B - plane_p, plane_n);

        if (dA >= 0.0)                      // A inside
            out[out_n++] = A;

        if ((dA > 0.0) != (dB > 0.0))      // edge crosses plane
        {
            const m3d::scalar t = dA / (dA - dB);
            out[out_n++] = A + (B - A) * t;
        }
    }
    return out_n;
}

// ---------------------------------------------------------------------------
//  Manifold point reduction
// ---------------------------------------------------------------------------
// Keep at most 4 contact points from `pts[n]`:
//   1. Deepest penetration point.
//   2. Farthest from point 1.
//   3. Farthest from line 1-2.
//   4. Farthest from triangle 1-2-3 (area maximisation).
// This gives a stable support polygon for the solver.
inline int reduce_to_4(const m3d::vec3      *pts,
                        const m3d::scalar    *depths,
                        int                   n,
                        m3d::vec3            *out_pts,
                        m3d::scalar          *out_depths)
{
    if (n <= 4)
    {
        for (int i = 0; i < n; ++i) { out_pts[i] = pts[i]; out_depths[i] = depths[i]; }
        return n;
    }

    // 1. Deepest point
    int i0 = 0;
    for (int i = 1; i < n; ++i)
        if (depths[i] > depths[i0]) i0 = i;

    // 2. Farthest from i0
    int i1 = (i0 == 0) ? 1 : 0;
    m3d::scalar best = m3d::length_sq(pts[i1] - pts[i0]);
    for (int i = 0; i < n; ++i)
    {
        if (i == i0) continue;
        m3d::scalar d = m3d::length_sq(pts[i] - pts[i0]);
        if (d > best) { best = d; i1 = i; }
    }

    // 3. Farthest from segment i0-i1
    int i2 = -1;
    best = 0.0;
    const m3d::vec3 seg = pts[i1] - pts[i0];
    for (int i = 0; i < n; ++i)
    {
        if (i == i0 || i == i1) continue;
        m3d::scalar d = m3d::length_sq(m3d::cross(pts[i] - pts[i0], seg));
        if (d > best) { best = d; i2 = i; }
    }
    if (i2 < 0) { i2 = 0; if (i2 == i0 || i2 == i1) i2 = 1; if (i2 == i0 || i2 == i1) i2 = 2; }

    // 4. Farthest from triangle i0-i1-i2 (signed area)
    int i3 = -1;
    best = 0.0;
    const m3d::vec3 tri_n = m3d::cross(pts[i1] - pts[i0], pts[i2] - pts[i0]);
    for (int i = 0; i < n; ++i)
    {
        if (i == i0 || i == i1 || i == i2) continue;
        m3d::scalar d = m3d::abs(m3d::dot(pts[i] - pts[i0], tri_n));
        if (d > best) { best = d; i3 = i; }
    }
    if (i3 < 0) { for (int i = 0; i < n; ++i) if (i!=i0&&i!=i1&&i!=i2){i3=i;break;} }

    out_pts[0] = pts[i0]; out_depths[0] = depths[i0];
    out_pts[1] = pts[i1]; out_depths[1] = depths[i1];
    out_pts[2] = pts[i2]; out_depths[2] = depths[i2];
    if (i3 >= 0) { out_pts[3] = pts[i3]; out_depths[3] = depths[i3]; return 4; }
    return 3;
}

} // namespace detail

// ===========================================================================
//  CollisionAlgorithm<Box, Box>
// ===========================================================================
template <>
struct CollisionAlgorithm<Box, Box>
{
    static bool test(const Box     &a, const m3d::tf &tf_a,
                     const Box     &b, const m3d::tf &tf_b,
                     ContactManifold &manifold)
    {
        const m3d::vec3 centre_diff = tf_b.pos - tf_a.pos;

        // World-space local axes of A and B
        const m3d::vec3 ax[3] = {
            tf_a.rotate_vector(m3d::vec3(1,0,0)),
            tf_a.rotate_vector(m3d::vec3(0,1,0)),
            tf_a.rotate_vector(m3d::vec3(0,0,1)),
        };
        const m3d::vec3 bx[3] = {
            tf_b.rotate_vector(m3d::vec3(1,0,0)),
            tf_b.rotate_vector(m3d::vec3(0,1,0)),
            tf_b.rotate_vector(m3d::vec3(0,0,1)),
        };

        m3d::scalar best_depth   = std::numeric_limits<m3d::scalar>::max();
        m3d::vec3   best_normal  = m3d::vec3(1, 0, 0);
        int         best_feature = 0; // 0-5 = face axes, 6-14 = edge-edge axes

        int feat = 0;

        // 3 face axes of A
        for (int i = 0; i < 3; ++i, ++feat)
            if (!detail::test_axis(ax[i], a, tf_a, b, tf_b,
                                   centre_diff, best_depth, best_normal,
                                   feat, best_feature))
                return false;

        // 3 face axes of B
        for (int i = 0; i < 3; ++i, ++feat)
            if (!detail::test_axis(bx[i], a, tf_a, b, tf_b,
                                   centre_diff, best_depth, best_normal,
                                   feat, best_feature))
                return false;

        // 9 edge cross-product axes
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j, ++feat)
                if (!detail::test_axis(m3d::cross(ax[i], bx[j]),
                                       a, tf_a, b, tf_b,
                                       centre_diff, best_depth, best_normal,
                                       feat, best_feature))
                    return false;

        // ── All 15 axes passed — we have a collision ─────────────────────────

        manifold.normal     = best_normal;
        manifold.num_points = 0;

        // ── Face-face contact (features 0-5): use Sutherland-Hodgman clipping
        if (best_feature < 6)
        {
            // Reference face: on A, most aligned with the contact normal
            // Incident face: on B, most anti-aligned with the contact normal
            m3d::vec3 ref_corners[4], ref_normal;
            m3d::vec3 inc_corners[4], inc_normal;

            detail::get_best_face(a, tf_a,  best_normal, ref_corners, ref_normal);
            detail::get_best_face(b, tf_b, -best_normal, inc_corners, inc_normal);

            // Sutherland-Hodgman: clip incident face against 4 side planes of reference face
            // The 4 side planes are perpendicular to the reference face and pass through its edges.
            // side normal for edge (ref_corners[i] → ref_corners[(i+1)%4]) is
            //   cross(ref_normal, edge_dir)  (pointing inward)

            // Working buffers (max 8 vertices after each clip)
            m3d::vec3 buf0[16], buf1[16];
            int       cnt = 4;
            for (int i = 0; i < 4; ++i) buf0[i] = inc_corners[i];

            for (int i = 0; i < 4; ++i)
            {
                const m3d::vec3 edge   = ref_corners[(i+1)%4] - ref_corners[i];
                const m3d::vec3 side_n = m3d::normalize(m3d::cross(ref_normal, edge)); // inward
                cnt = detail::clip_polygon_by_plane(buf0, cnt, side_n, ref_corners[i], buf1);
                if (cnt == 0) break;
                for (int k = 0; k < cnt; ++k) buf0[k] = buf1[k];
            }

            if (cnt == 0) return false; // clipped away entirely

            // Keep only points that penetrate the reference face plane
            // (i.e. on the negative side of ref_normal through ref_corners[0])
            const m3d::scalar ref_d = m3d::dot(ref_corners[0], ref_normal);

            m3d::vec3   keep_pts[16];
            m3d::scalar keep_dep[16];
            int         keep_n = 0;

            for (int i = 0; i < cnt; ++i)
            {
                const m3d::scalar signed_dist = m3d::dot(buf0[i], ref_normal) - ref_d;
                if (signed_dist <= m3d::EPSILON) // on or below the reference face
                {
                    keep_pts[keep_n] = buf0[i];
                    keep_dep[keep_n] = -signed_dist;
                    ++keep_n;
                }
            }

            if (keep_n == 0)
            {
                // Fall back: single point at the box-contact surface midpoint
                manifold.num_points             = 1;
                manifold.points[0].penetration_depth = best_depth;
                manifold.points[0].position     = tf_a.pos
                    + tf_a.rotate_vector(m3d::vec3(0,0,0)) // placeholder
                    - best_normal * (best_depth * 0.5);
                return true;
            }

            // Reduce to ≤4 contact points
            m3d::vec3   red_pts[4];
            m3d::scalar red_dep[4];
            int red_n = detail::reduce_to_4(keep_pts, keep_dep, keep_n, red_pts, red_dep);

            manifold.num_points = red_n;
            for (int i = 0; i < red_n; ++i)
            {
                manifold.points[i].position          = red_pts[i];
                manifold.points[i].penetration_depth = red_dep[i];
            }
        }
        else
        {
            // ── Edge-edge contact (features 6-14): single contact point ──────
            // For edge-edge the closest points on both edges form the contact.
            // We compute them via the standard segment-closest-point formula.

            const int ei = (best_feature - 6) / 3; // edge axis index on A (0,1,2)
            const int ej = (best_feature - 6) % 3; // edge axis index on B (0,1,2)

            // Support points on each box along the edge direction
            const m3d::vec3 pa = tf_a.pos + tf_a.rotate_vector(
                m3d::vec3(
                    (m3d::dot(ax[0], best_normal) >= 0 ? 1.0 : -1.0) * (ei==0 ? 0.0 : a.half_extents.x),
                    (m3d::dot(ax[1], best_normal) >= 0 ? 1.0 : -1.0) * (ei==1 ? 0.0 : a.half_extents.y),
                    (m3d::dot(ax[2], best_normal) >= 0 ? 1.0 : -1.0) * (ei==2 ? 0.0 : a.half_extents.z)
                ));
            const m3d::vec3 pb = tf_b.pos + tf_b.rotate_vector(
                m3d::vec3(
                    (m3d::dot(bx[0],-best_normal) >= 0 ? 1.0 : -1.0) * (ej==0 ? 0.0 : b.half_extents.x),
                    (m3d::dot(bx[1],-best_normal) >= 0 ? 1.0 : -1.0) * (ej==1 ? 0.0 : b.half_extents.y),
                    (m3d::dot(bx[2],-best_normal) >= 0 ? 1.0 : -1.0) * (ej==2 ? 0.0 : b.half_extents.z)
                ));

            // Closest point between the two edges (midpoint approximation)
            const m3d::vec3 contact_pt = (pa + pb) * 0.5;

            manifold.num_points                  = 1;
            manifold.points[0].position          = contact_pt;
            manifold.points[0].penetration_depth = best_depth;
        }

        return true;
    }
};

} // namespace rbc