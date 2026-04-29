#pragma once
#include "vec3.hpp"
#include "quat.hpp"

/**
 * @file tf.hpp
 * @brief Rigid transform: a translation paired with a quaternion rotation.
 * @ingroup math3d
 */

namespace m3d
{
    /**
     * @brief Rigid (SE(3)) transform = translation + rotation.
     *
     * Represents a frame change with no scaling or shear. Used pervasively
     * for body poses, joint frames, sensor mounting, etc. Composition is
     * intentionally not provided as an operator — chain the helpers
     * explicitly to make ordering obvious.
     *
     * @code
     * tf body_pose{ {0, 5, 0}, quat::from_axis_angle({0,1,0}, PI/4) };
     * vec3 corner_local{0.5, 0.5, 0.5};
     * vec3 corner_world = body_pose.transform_point(corner_local);
     * @endcode
     *
     * @ingroup math3d
     */
    struct tf
    {
        vec3 pos;  ///< Translation, applied after rotation.
        quat rot;  ///< Rotation, applied first.

        /** @brief Identity transform (origin, no rotation). */
        tf() : pos(0, 0, 0), rot(1, 0, 0, 0) {}
        /** @brief Build from a translation and a (preferably unit) quaternion. */
        tf(const vec3 &p, const quat &q) : pos(p), rot(q) {}

        /**
         * @brief Map a point from this frame's local space to world space.
         *
         * Computes `pos + rot * p`.
         *
         * @param p Point in local coordinates.
         * @return Point in world coordinates.
         */
        inline vec3 transform_point(const vec3 &p) const
        {
            return pos + rotate(rot, p);
        }

        /**
         * @brief Rotate a free vector (direction) — translation is ignored.
         *
         * Use this for vectors that are not anchored to the origin, such as
         * normals, axes, velocities, and force directions.
         *
         * @param d Vector in local coordinates.
         * @return Vector in world coordinates.
         */
        inline vec3 rotate_vector(const vec3 &d) const
        {
            return rotate(rot, d);
        }

        /**
         * @brief Rotate a world-space direction back into this frame's local space.
         *
         * Equivalent to `rotate(conjugate(rot), d)` for unit `rot`.
         *
         * @param d Vector in world coordinates.
         * @return Vector in this frame's local coordinates.
         */
        inline vec3 inverse_rotate_vector(const vec3 &d) const
        {
            return rotate(conjugate(rot), d);
        }
    };
} // namespace m3d
