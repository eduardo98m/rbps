#pragma once
#include "rbc/shapes/CollisionShape.hpp"
#include <math3d/math3d.hpp>

namespace rbc
{
    struct MinkowskiDiff
    {
        const CollisionShape *shape_a;
        const CollisionShape *shape_b;
        m3d::tf tf_a;
        m3d::tf tf_b;

        MinkowskiDiff(const CollisionShape *a, const CollisionShape *b,
                      const m3d::tf &ta, const m3d::tf &tb)
            : shape_a(a), shape_b(b), tf_a(ta), tf_b(tb) {}

        // Calcula: Support(A) - Support(B) en coordenadas globales
        m3d::vec3 support(const m3d::vec3 &dir) const
        {
            // 1. Transformar dirección al espacio local de A, buscar support, devolver a global
            m3d::vec3 local_dir_a = tf_a.inverse_rotate_vector(dir);
            m3d::vec3 p_a = tf_a.transform_point(shape_a->support(local_dir_a));

            // 2. Transformar dirección opuesta al espacio local de B, buscar support, devolver a global
            m3d::vec3 local_dir_b = tf_b.inverse_rotate_vector(-dir);
            m3d::vec3 p_b = tf_b.transform_point(shape_b->support(local_dir_b));

            return p_a - p_b;
        }
    };
}