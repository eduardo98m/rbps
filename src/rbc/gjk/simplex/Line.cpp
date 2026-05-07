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
        }
        else
        {
            s.rank = 1;
            direction = ao;
        }
        return false;
    }
} // namespace rbc
