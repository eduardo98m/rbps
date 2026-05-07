#include "rbc/gjk/simplex/DoSimplex.hpp"
#include "SimplexHelpers.hpp"

namespace rbc
{
    bool do_simplex_3(Simplex &s, m3d::vec3 &direction)
    {
        const m3d::vec3 a = s.vertex[0]->w;
        const m3d::vec3 b = s.vertex[1]->w;
        const m3d::vec3 c = s.vertex[2]->w;

        const m3d::vec3 ao  = -a;
        const m3d::vec3 ab  = b - a;
        const m3d::vec3 ac  = c - a;
        const m3d::vec3 abc = m3d::cross(ab, ac);

        using simplex_detail::triple_cross;

        if (m3d::dot(m3d::cross(abc, ac), ao) >= 0.0)
        {
            if (m3d::dot(ac, ao) >= 0.0)
            {
                // AC region: drop B, keep A and C.
                s.vertex[1] = s.vertex[2];
                s.rank = 2;
                direction = triple_cross(ac, ao, ac);
            }
            else if (m3d::dot(ab, ao) >= 0.0)
            {
                // AB region: drop C, keep A and B.
                s.rank = 2;
                direction = triple_cross(ab, ao, ab);
            }
            else
            {
                // A region.
                s.rank = 1;
                direction = ao;
            }
        }
        else
        {
            if (m3d::dot(m3d::cross(ab, abc), ao) >= 0.0)
            {
                if (m3d::dot(ab, ao) >= 0.0)
                {
                    // AB region.
                    s.rank = 2;
                    direction = triple_cross(ab, ao, ab);
                }
                else
                {
                    // A region.
                    s.rank = 1;
                    direction = ao;
                }
            }
            else
            {
                if (m3d::dot(abc, ao) >= 0.0)
                {
                    // ABC region, origin above the triangle plane.
                    s.rank = 2 + 1;
                    direction = abc;
                }
                else
                {
                    // ABC region, origin below — swap B and C so the
                    // simplex winding matches the new search direction.
                    SimplexVertex *tmp = s.vertex[1];
                    s.vertex[1] = s.vertex[2];
                    s.vertex[2] = tmp;
                    s.rank = 3;
                    direction = -abc;
                }
            }
        }
        return false;
    }
} // namespace rbc
