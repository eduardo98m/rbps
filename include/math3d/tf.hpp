#pragma once
#include "vec3.hpp"
#include "quat.hpp"

namespace m3d
{
    struct tf
    {
        vec3 pos;
        quat rot;

        tf() : pos(0, 0, 0), rot(1, 0, 0, 0) {}
        tf(const vec3 &p, const quat &q) : pos(p), rot(q) {}

        // Converts a point in the local frame of this transform to world space
        inline vec3 transform_point(const vec3 &p) const
        {
            return pos + rotate(rot, p);
        }

        // Rotates a vectors (Ignore translation)
        inline vec3 rotate_vector(const vec3 &d) const
        {
            return rotate(rot, d);
        }

        // Rotates a vector by the inverse of this transform's rotation (useful for transforming directions into local space)
        inline vec3 inverse_rotate_vector(const vec3 &d) const
        {
            return rotate(conjugate(rot), d);
        }
    };
}