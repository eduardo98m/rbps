#pragma once
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    struct Box
    {
        m3d::vec3 half_extents;
        // Constructors
        Box() : half_extents(0.5, 0.5, 0.5) {}
        Box(const m3d::vec3 &half_extents) : half_extents(half_extents) {}

        inline bool operator==(const Box &other) const
        {
            return half_extents == other.half_extents;
        }

        inline bool operator!=(const Box &other) const
        {
            return !(*this == other);
        }
    };

    inline m3d::vec3 support(const Box &box, const m3d::vec3 &dir)
    {
        // The furthest point of a box maps to its corners based on the sign of the direction
        return m3d::vec3(
            (dir.x > 0) ? box.half_extents.x : -box.half_extents.x,
            (dir.y > 0) ? box.half_extents.y : -box.half_extents.y,
            (dir.z > 0) ? box.half_extents.z : -box.half_extents.z);
    }

    inline m3d::scalar compute_volume(const Box &box)
    {
        return 8.0 * box.half_extents.x * box.half_extents.y * box.half_extents.z; // Volume of the box
    }

    inline m3d::smat3 compute_inertia_tensor(const Box &box)
    {
        m3d::scalar x2 = 4.0 * box.half_extents.x * box.half_extents.x;
        m3d::scalar y2 = 4.0 * box.half_extents.y * box.half_extents.y;
        m3d::scalar z2 = 4.0 * box.half_extents.z * box.half_extents.z;
        m3d::scalar mass = compute_volume(box); // Assuming unit density for simplicity (How we should ahndle this? Should we add a density parameter to the box struct?)
        return m3d::smat3(
            (1.0 / 12.0) * mass * (y2 + z2), // Ixx
            (1.0 / 12.0) * mass * (x2 + z2), // Iyy
            (1.0 / 12.0) * mass * (x2 + y2), // Izz
            0,                               // Ixy
            0,                               // Ixz
            0                                // Iyz
        );
    }

    // Project the OBB onto world axes by summing |R_ij| * half_extents_j
    // for each world axis i. This is the standard tight AABB for a rotated box.
    //
    // NOTE: Assumes m3d::mat3_cast(quat) returns a column-major mat3 where
    //       mat[col][row] gives the element, and tf.rot is an m3d::quat.
    //       Adjust the indexing if your math3d uses a different convention.
    inline AABB compute_aabb(const Box &b, const m3d::tf &tf)
    {
        const m3d::mat3 R = m3d::mat3_cast(tf.rot);
        const m3d::vec3 &h = b.half_extents;

        // World-space half-extents of the AABB that wraps the rotated box.
        // extent_i = |R_i0|*hx + |R_i1|*hy + |R_i2|*hz  (row i of R dotted with |h|)
        const m3d::vec3 extent(
            m3d::abs(R[0][0]) * h.x + m3d::abs(R[1][0]) * h.y + m3d::abs(R[2][0]) * h.z,
            m3d::abs(R[0][1]) * h.x + m3d::abs(R[1][1]) * h.y + m3d::abs(R[2][1]) * h.z,
            m3d::abs(R[0][2]) * h.x + m3d::abs(R[1][2]) * h.y + m3d::abs(R[2][2]) * h.z);

        return {tf.pos - extent, tf.pos + extent};
    }

    // Marker for the dispatcher: Box is a convex bounded shape.
    constexpr bool is_gjk_convex(const Box *) { return true; }

    inline m3d::scalar representative_radius(const Box &b)
    {
        return m3d::max(m3d::max(b.half_extents.x, b.half_extents.y), b.half_extents.z);
    }

    // Returns the 4 corners of the box face whose outward normal is most
    // aligned with `dir`. Used as the reference / incident face polygon
    // by the GJK/EPA manifold generator.
    inline int face_corners(const Box &b, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 corners[4])
    {
        const m3d::vec3 axes[3] = {
            tf.rotate_vector(m3d::vec3(1, 0, 0)),
            tf.rotate_vector(m3d::vec3(0, 1, 0)),
            tf.rotate_vector(m3d::vec3(0, 0, 1)),
        };
        const m3d::scalar half[3] = {b.half_extents.x, b.half_extents.y, b.half_extents.z};

        int best = 0;
        m3d::scalar best_d = m3d::abs(m3d::dot(axes[0], dir));
        for (int i = 1; i < 3; ++i)
        {
            const m3d::scalar d = m3d::abs(m3d::dot(axes[i], dir));
            if (d > best_d)
            {
                best_d = d;
                best = i;
            }
        }

        const m3d::scalar sign = (m3d::dot(axes[best], dir) >= 0.0) ? 1.0 : -1.0;
        const m3d::vec3 centre = tf.pos + axes[best] * (sign * half[best]);
        const int u = (best + 1) % 3, v = (best + 2) % 3;

        corners[0] = centre + axes[u] * half[u] + axes[v] * half[v];
        corners[1] = centre - axes[u] * half[u] + axes[v] * half[v];
        corners[2] = centre - axes[u] * half[u] - axes[v] * half[v];
        corners[3] = centre + axes[u] * half[u] - axes[v] * half[v];
        return 4;
    }
}