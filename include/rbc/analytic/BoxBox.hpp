#pragma once
#include "rbc/Dispatcher.hpp"
#include <algorithm>
#include <array>
#include <limits>

/**
 * @file BoxBox.hpp
 * @brief Analytic Box–Box collision algorithm (SAT + face/face manifold).
 * @ingroup rbc
 * @ingroup internals
 *
 * Standard 15-axis Separating Axis Test on two oriented boxes:
 * 3 face normals from each box plus 9 edge–edge cross products. When the
 * separating axis is a face normal the algorithm builds a face/face
 * contact manifold (up to 4 contact points); for edge–edge separation it
 * falls back to a single contact point.
 *
 * Box–Box is currently the only pair that produces a true face/face
 * manifold; other shape pairs use 1-point contacts. See
 * `docs/design/contact-generation.md` for the planned generalisation.
 */

namespace rbc
{
    namespace detail
    {
        // Project OBB onto a world-space axis
        inline m3d::scalar obb_radius(const Box &box, const m3d::tf &tf, const m3d::vec3 &axis)
        {
            const m3d::vec3 ax = tf.rotate_vector(m3d::vec3(1, 0, 0));
            const m3d::vec3 ay = tf.rotate_vector(m3d::vec3(0, 1, 0));
            const m3d::vec3 az = tf.rotate_vector(m3d::vec3(0, 0, 1));
            return m3d::abs(m3d::dot(ax, axis)) * box.half_extents.x +
                   m3d::abs(m3d::dot(ay, axis)) * box.half_extents.y +
                   m3d::abs(m3d::dot(az, axis)) * box.half_extents.z;
        }

        inline bool test_axis(const m3d::vec3 &axis,
                              const Box &a, const m3d::tf &tf_a,
                              const Box &b, const m3d::tf &tf_b,
                              const m3d::vec3 &centre_diff,
                              m3d::scalar &best_depth,
                              m3d::vec3 &best_normal,
                              int feature_id,
                              int &best_feature,
                              m3d::scalar bias = 0.0)
        {
            const m3d::scalar len_sq = m3d::length_sq(axis);
            if (len_sq < m3d::EPSILON * m3d::EPSILON)
                return true;

            const m3d::vec3 norm_axis = axis / m3d::sqrt(len_sq);
            const m3d::scalar rA = obb_radius(a, tf_a, norm_axis);
            const m3d::scalar rB = obb_radius(b, tf_b, norm_axis);
            const m3d::scalar dist = m3d::abs(m3d::dot(centre_diff, norm_axis));
            const m3d::scalar overlap = rA + rB - dist;

            if (overlap < 0.0)
                return false;

            if ((overlap + bias) < best_depth)
            {
                best_depth = overlap + bias;
                best_normal = (m3d::dot(centre_diff, norm_axis) >= 0.0) ? norm_axis : -norm_axis;
                best_feature = feature_id;
            }
            return true;
        }

        inline void get_best_face(const Box &box, const m3d::tf &tf, const m3d::vec3 &dir,
                                  m3d::vec3 out_corners[4], m3d::vec3 &out_normal)
        {
            const m3d::vec3 axes[3] = {
                tf.rotate_vector(m3d::vec3(1, 0, 0)),
                tf.rotate_vector(m3d::vec3(0, 1, 0)),
                tf.rotate_vector(m3d::vec3(0, 0, 1))};
            const m3d::scalar half[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};

            int best_axis = 0;
            m3d::scalar best_dot = m3d::abs(m3d::dot(axes[0], dir));
            for (int i = 1; i < 3; ++i)
            {
                m3d::scalar d = m3d::abs(m3d::dot(axes[i], dir));
                if (d > best_dot)
                {
                    best_dot = d;
                    best_axis = i;
                }
            }

            const m3d::scalar sign = (m3d::dot(axes[best_axis], dir) >= 0.0) ? 1.0 : -1.0;
            out_normal = axes[best_axis] * sign;
            const m3d::vec3 face_centre = tf.pos + out_normal * half[best_axis];

            const int u = (best_axis + 1) % 3;
            const int v = (best_axis + 2) % 3;

            out_corners[0] = face_centre + axes[u] * half[u] + axes[v] * half[v];
            out_corners[1] = face_centre - axes[u] * half[u] + axes[v] * half[v];
            out_corners[2] = face_centre - axes[u] * half[u] - axes[v] * half[v];
            out_corners[3] = face_centre + axes[u] * half[u] - axes[v] * half[v];
        }

        inline int clip_polygon_by_plane(const m3d::vec3 *poly, int n,
                                         const m3d::vec3 &plane_n, const m3d::vec3 &plane_p,
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

        // --- FIX: Coplanar Area Maximization ---
        inline int reduce_to_4(const m3d::vec3 *pts, const m3d::scalar *depths, int n,
                               m3d::vec3 *out_pts, m3d::scalar *out_depths)
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
            m3d::scalar best = -1.0;
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

            int i2 = -1;
            best = -1.0;
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
                i2 = 0;
                while (i2 == i0 || i2 == i1)
                    ++i2;
            }

            // To maximize area, find the point furthest on the OPPOSITE side of the diagonal as i2
            int i3 = -1;
            m3d::scalar min_dot = 1.0;
            const m3d::vec3 tri_n = m3d::cross(pts[i2] - pts[i0], seg);
            for (int i = 0; i < n; ++i)
            {
                if (i == i0 || i == i1 || i == i2)
                    continue;
                m3d::scalar d = m3d::dot(m3d::cross(pts[i] - pts[i0], seg), tri_n);
                if (d < min_dot)
                {
                    min_dot = d;
                    i3 = i;
                }
            }

            // If all points are on the same side, fallback to finding the point furthest from i2
            if (i3 < 0 || min_dot >= 0.0)
            {
                best = -1.0;
                for (int i = 0; i < n; ++i)
                {
                    if (i == i0 || i == i1 || i == i2)
                        continue;
                    m3d::scalar d = m3d::length_sq(pts[i] - pts[i2]);
                    if (d > best)
                    {
                        best = d;
                        i3 = i;
                    }
                }
            }
            if (i3 < 0)
            {
                i3 = 0;
                while (i3 == i0 || i3 == i1 || i3 == i2)
                    ++i3;
            }

            out_pts[0] = pts[i0];
            out_depths[0] = depths[i0];
            out_pts[1] = pts[i1];
            out_depths[1] = depths[i1];
            out_pts[2] = pts[i2];
            out_depths[2] = depths[i2];
            out_pts[3] = pts[i3];
            out_depths[3] = depths[i3];
            return 4;
        }
    }

    /**
     * @brief Box vs Box via 15-axis SAT; produces a face/face manifold (1–4 contacts).
     * @ingroup rbc
     * @ingroup internals
     */
    template <>
    struct CollisionAlgorithm<Box, Box>
    {
        /** @brief Run box–box. */
        static bool test(const Box &a, const m3d::tf &tf_a,
                         const Box &b, const m3d::tf &tf_b,
                         ContactManifold &manifold)
        {
            const m3d::vec3 centre_diff = tf_b.pos - tf_a.pos;
            const m3d::vec3 ax[3] = {tf_a.rotate_vector(m3d::vec3(1, 0, 0)), tf_a.rotate_vector(m3d::vec3(0, 1, 0)), tf_a.rotate_vector(m3d::vec3(0, 0, 1))};
            const m3d::vec3 bx[3] = {tf_b.rotate_vector(m3d::vec3(1, 0, 0)), tf_b.rotate_vector(m3d::vec3(0, 1, 0)), tf_b.rotate_vector(m3d::vec3(0, 0, 1))};

            m3d::scalar best_depth = std::numeric_limits<m3d::scalar>::max();
            m3d::vec3 best_normal = m3d::vec3(1, 0, 0);
            int best_feature = 0, feat = 0;

            for (int i = 0; i < 3; ++i, ++feat)
                if (!detail::test_axis(ax[i], a, tf_a, b, tf_b, centre_diff, best_depth, best_normal, feat, best_feature))
                    return false;

            for (int i = 0; i < 3; ++i, ++feat)
                if (!detail::test_axis(bx[i], a, tf_a, b, tf_b, centre_diff, best_depth, best_normal, feat, best_feature))
                    return false;

            const m3d::scalar edge_bias = 0.001f;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j, ++feat)
                    if (!detail::test_axis(m3d::cross(ax[i], bx[j]), a, tf_a, b, tf_b, centre_diff, best_depth, best_normal, feat, best_feature, edge_bias))
                        return false;

            if (best_feature >= 6)
                best_depth -= edge_bias;

            manifold.normal = best_normal;
            manifold.num_points = 0;

            // Face-face contact
            if (best_feature < 6)
            {
                bool ref_is_A = (best_feature < 3);
                const Box &ref_box = ref_is_A ? a : b;
                const m3d::tf &ref_tf = ref_is_A ? tf_a : tf_b;
                const Box &inc_box = ref_is_A ? b : a;
                const m3d::tf &inc_tf = ref_is_A ? tf_b : tf_a;

                m3d::vec3 ref_normal_target = ref_is_A ? best_normal : -best_normal;
                m3d::vec3 ref_corners[4], ref_normal, inc_corners[4], inc_normal;

                detail::get_best_face(ref_box, ref_tf, ref_normal_target, ref_corners, ref_normal);
                detail::get_best_face(inc_box, inc_tf, -ref_normal_target, inc_corners, inc_normal);

                m3d::vec3 buf0[16], buf1[16];
                int cnt = 4;
                for (int i = 0; i < 4; ++i)
                    buf0[i] = inc_corners[i];

                m3d::vec3 ref_center = (ref_corners[0] + ref_corners[1] + ref_corners[2] + ref_corners[3]) * 0.25f;

                for (int i = 0; i < 4; ++i)
                {
                    const m3d::vec3 edge = ref_corners[(i + 1) % 4] - ref_corners[i];
                    m3d::vec3 side_n = m3d::normalize(m3d::cross(ref_normal, edge));

                    if (m3d::dot(side_n, ref_center - ref_corners[i]) < 0.0f)
                        side_n = -side_n;

                    cnt = detail::clip_polygon_by_plane(buf0, cnt, side_n, ref_corners[i], buf1);
                    if (cnt == 0)
                        break;
                    for (int k = 0; k < cnt; ++k)
                        buf0[k] = buf1[k];
                }

                if (cnt == 0)
                    return false;

                const m3d::scalar ref_d = m3d::dot(ref_corners[0], ref_normal);
                m3d::vec3 keep_pts[16];
                m3d::scalar keep_dep[16];
                int keep_n = 0;

                for (int i = 0; i < cnt; ++i)
                {
                    const m3d::scalar signed_dist = m3d::dot(buf0[i], ref_normal) - ref_d;
                    if (signed_dist <= m3d::EPSILON)
                    {
                        keep_pts[keep_n] = buf0[i] - ref_normal * (signed_dist * 0.5f);
                        keep_dep[keep_n] = -signed_dist;
                        ++keep_n;
                    }
                }

                // --- FIX: Eliminate explosive fallback ---
                // If clipping missed entirely (e.g., grazing corner float errors), gracefully skip the frame
                // instead of placing an explosive fallback contact at the center of mass.
                if (keep_n == 0)
                    return false;

                m3d::vec3 red_pts[4];
                m3d::scalar red_dep[4];
                int red_n = detail::reduce_to_4(keep_pts, keep_dep, keep_n, red_pts, red_dep);

                manifold.num_points = red_n;
                for (int i = 0; i < red_n; ++i)
                {
                    manifold.points[i].position = red_pts[i];
                    manifold.points[i].penetration_depth = red_dep[i];
                }
            }
            else
            {
                // Edge-edge contact
                const int ei = (best_feature - 6) / 3;
                const int ej = (best_feature - 6) % 3;

                const m3d::scalar extA[3] = {a.half_extents.x, a.half_extents.y, a.half_extents.z};
                const m3d::scalar extB[3] = {b.half_extents.x, b.half_extents.y, b.half_extents.z};

                const m3d::vec3 pa = tf_a.pos + tf_a.rotate_vector(m3d::vec3(
                                                    (m3d::dot(ax[0], best_normal) >= 0 ? 1.0 : -1.0) * (ei == 0 ? 0.0 : extA[0]),
                                                    (m3d::dot(ax[1], best_normal) >= 0 ? 1.0 : -1.0) * (ei == 1 ? 0.0 : extA[1]),
                                                    (m3d::dot(ax[2], best_normal) >= 0 ? 1.0 : -1.0) * (ei == 2 ? 0.0 : extA[2])));

                const m3d::vec3 pb = tf_b.pos + tf_b.rotate_vector(m3d::vec3(
                                                    (m3d::dot(bx[0], -best_normal) >= 0 ? 1.0 : -1.0) * (ej == 0 ? 0.0 : extB[0]),
                                                    (m3d::dot(bx[1], -best_normal) >= 0 ? 1.0 : -1.0) * (ej == 1 ? 0.0 : extB[1]),
                                                    (m3d::dot(bx[2], -best_normal) >= 0 ? 1.0 : -1.0) * (ej == 2 ? 0.0 : extB[2])));

                m3d::vec3 d1 = ax[ei], d2 = bx[ej];
                m3d::vec3 r = pa - pb;

                m3d::scalar b_dot = m3d::dot(d1, d2);
                m3d::scalar c_dot = m3d::dot(d1, r);
                m3d::scalar f_dot = m3d::dot(d2, r);
                m3d::scalar denom = 1.0f - b_dot * b_dot;

                m3d::scalar s = 0.0f, t = 0.0f;
                if (denom > m3d::EPSILON)
                    s = (b_dot * f_dot - c_dot) / denom;

                // --- FIX: Interdependent Edge Clamping ---
                m3d::scalar extA_val = extA[ei];
                m3d::scalar extB_val = extB[ej];

                // If s is clamped, we must recalculate t, otherwise we get fake torque
                if (s < -extA_val || s > extA_val)
                    s = std::clamp(s, -extA_val, extA_val);

                t = s * b_dot + f_dot;
                if (t < -extB_val || t > extB_val)
                {
                    t = std::clamp(t, -extB_val, extB_val);
                    // If t is clamped, recalculate s again to lock in the true closest point
                    s = std::clamp(t * b_dot - c_dot, -extA_val, extA_val);
                }

                manifold.num_points = 1;
                manifold.points[0].position = (pa + d1 * s + pb + d2 * t) * 0.5f;
                manifold.points[0].penetration_depth = best_depth;
            }

            return true;
        }
    };
}