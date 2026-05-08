#include "rbc/gjk/simplex/DoSimplex.hpp"
#include "SimplexHelpers.hpp"

namespace rbc
{
    bool do_simplex_4(Simplex &s, m3d::vec3 &direction)
    {
        SimplexVertex *const va = s.vertex[0];
        SimplexVertex *const vb = s.vertex[1];
        SimplexVertex *const vc = s.vertex[2];
        SimplexVertex *const vd = s.vertex[3];

        const m3d::vec3 a = va->w;
        const m3d::vec3 b = vb->w;
        const m3d::vec3 c = vc->w;
        const m3d::vec3 d = vd->w;

        const m3d::vec3 ao  = -a;
        const m3d::vec3 ab  = b - a;
        const m3d::vec3 ac  = c - a;
        const m3d::vec3 ad  = d - a;
        const m3d::vec3 abc = m3d::cross(ab, ac);
        const m3d::vec3 acd = m3d::cross(ac, ad);
        const m3d::vec3 adb = m3d::cross(ad, ab);

        // Use a small positive threshold instead of `>= 0.0` so origins
        // that lie *exactly* on a face plane (dot ≈ 0, common when the
        // shapes are perfectly aligned and an MD face passes through
        // origin) fall into case 0x0 (enclosed) rather than reducing
        // to a sub-feature. Without this, GJK never terminates on
        // aligned-stack configurations and EPA never gets a seed.
        constexpr m3d::scalar kInsideEps = 1e-10;
        unsigned char plane_info = 0x0;
        if (m3d::dot(abc, ao) > kInsideEps) plane_info |= 0x1;
        if (m3d::dot(acd, ao) > kInsideEps) plane_info |= 0x2;
        if (m3d::dot(adb, ao) > kInsideEps) plane_info |= 0x4;

        using simplex_detail::triple_cross;

        switch (plane_info)
        {
            case 0x0:
                // Origin lies on the inside of all three triangles → enclosed.
                return true;

            case 0x1:
                // Triangle ABC.
                if (m3d::dot(m3d::cross(abc, ac), ao) >= 0.0)
                {
                    if (m3d::dot(ac, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vc; s.rank = 2;
                        direction = triple_cross(ac, ao, ac);
                    }
                    else if (m3d::dot(ab, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vb; s.rank = 2;
                        direction = triple_cross(ab, ao, ab);
                    }
                    else
                    {
                        s.vertex[0] = va; s.rank = 1;
                        direction = ao;
                    }
                }
                else if (m3d::dot(m3d::cross(ab, abc), ao) >= 0.0)
                {
                    if (m3d::dot(ab, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vb; s.rank = 2;
                        direction = triple_cross(ab, ao, ab);
                    }
                    else
                    {
                        s.vertex[0] = va; s.rank = 1;
                        direction = ao;
                    }
                }
                else
                {
                    s.vertex[0] = va; s.vertex[1] = vb; s.vertex[2] = vc; s.rank = 3;
                    direction = abc;
                }
                break;

            case 0x2:
                // Triangle ACD.
                if (m3d::dot(m3d::cross(acd, ad), ao) >= 0.0)
                {
                    if (m3d::dot(ad, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vd; s.rank = 2;
                        direction = triple_cross(ad, ao, ad);
                    }
                    else if (m3d::dot(ac, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vc; s.rank = 2;
                        direction = triple_cross(ac, ao, ac);
                    }
                    else
                    {
                        s.vertex[0] = va; s.rank = 1;
                        direction = ao;
                    }
                }
                else if (m3d::dot(m3d::cross(ac, acd), ao) >= 0.0)
                {
                    if (m3d::dot(ac, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vc; s.rank = 2;
                        direction = triple_cross(ac, ao, ac);
                    }
                    else
                    {
                        s.vertex[0] = va; s.rank = 1;
                        direction = ao;
                    }
                }
                else
                {
                    s.vertex[0] = va; s.vertex[1] = vc; s.vertex[2] = vd; s.rank = 3;
                    direction = acd;
                }
                break;

            case 0x3:
                // Line AC.
                if (m3d::dot(ac, ao) >= 0.0)
                {
                    s.vertex[0] = va; s.vertex[1] = vc; s.rank = 2;
                    direction = triple_cross(ac, ao, ac);
                }
                else
                {
                    s.vertex[0] = va; s.rank = 1;
                    direction = ao;
                }
                break;

            case 0x4:
                // Triangle ADB.
                if (m3d::dot(m3d::cross(adb, ab), ao) >= 0.0)
                {
                    if (m3d::dot(ab, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vb; s.rank = 2;
                        direction = triple_cross(ab, ao, ab);
                    }
                    else if (m3d::dot(ad, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vd; s.rank = 2;
                        direction = triple_cross(ad, ao, ad);
                    }
                    else
                    {
                        s.vertex[0] = va; s.rank = 1;
                        direction = ao;
                    }
                }
                else if (m3d::dot(m3d::cross(ad, adb), ao) >= 0.0)
                {
                    if (m3d::dot(ad, ao) >= 0.0)
                    {
                        s.vertex[0] = va; s.vertex[1] = vd; s.rank = 2;
                        direction = triple_cross(ad, ao, ad);
                    }
                    else
                    {
                        s.vertex[0] = va; s.rank = 1;
                        direction = ao;
                    }
                }
                else
                {
                    s.vertex[0] = va; s.vertex[1] = vd; s.vertex[2] = vb; s.rank = 3;
                    direction = adb;
                }
                break;

            case 0x5:
                // Line AB.
                if (m3d::dot(ab, ao) >= 0.0)
                {
                    s.vertex[0] = va; s.vertex[1] = vb; s.rank = 2;
                    direction = triple_cross(ab, ao, ab);
                }
                else
                {
                    s.vertex[0] = va; s.rank = 1;
                    direction = ao;
                }
                break;

            case 0x6:
                // Line AD.
                if (m3d::dot(ad, ao) >= 0.0)
                {
                    s.vertex[0] = va; s.vertex[1] = vd; s.rank = 2;
                    direction = triple_cross(ad, ao, ad);
                }
                else
                {
                    s.vertex[0] = va; s.rank = 1;
                    direction = ao;
                }
                break;

            case 0x7:
            default:
                // Point A.
                s.vertex[0] = va; s.rank = 1;
                direction = ao;
                break;
        }
        return false;
    }
} // namespace rbc
