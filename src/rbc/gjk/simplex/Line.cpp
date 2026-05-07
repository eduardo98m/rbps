#include "rbc/gjk/simplex/DoSimplex.hpp"
#include "SimplexHelpers.hpp"

namespace rbc
{
    bool do_simplex_2(Simplex &s, m3d::vec3 &direction)
    {
        const m3d::vec3 a = s.vertex[0]->w;
        const m3d::vec3 b = s.vertex[1]->w;

        const m3d::vec3 ao = -a;
        const m3d::vec3 ab = b - a;

        if (m3d::dot(ab, ao) >= 0.0)
        {
            s.rank = 2;
            direction = simplex_detail::triple_cross(ab, ao, ab);

            // Origin lies on the AB line: triple_cross collapses to zero.
            // The literal reference would stall here, so EPA receives a
            // rank-2 simplex and reports NonLinear (this is what shows
            // up in tet-edge-vs-tet-edge contacts). Pick any axis not
            // parallel to ab and cross with it to get a perpendicular
            // search direction; the next support sample breaks the
            // collinearity and the simplex grows to rank 3, then 4.
            if (m3d::length_sq(direction) < 1e-6)
            {
                const m3d::scalar ax = m3d::abs(ab.x);
                const m3d::scalar ay = m3d::abs(ab.y);
                const m3d::scalar az = m3d::abs(ab.z);
                const m3d::vec3 axis = (ax <= ay && ax <= az)
                                         ? m3d::vec3(1.0, 0.0, 0.0)
                                         : (ay <= az
                                              ? m3d::vec3(0.0, 1.0, 0.0)
                                              : m3d::vec3(0.0, 0.0, 1.0));
                direction = m3d::cross(ab, axis);
                if (m3d::length_sq(direction) < 1e-6)
                    direction = m3d::cross(ab, m3d::vec3(0.0, 0.0, 1.0));
            }
        }
        else
        {
            s.rank = 1;
            direction = ao;
        }
        return false;
    }
} // namespace rbc
