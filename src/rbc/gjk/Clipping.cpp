#include "rbc/gjk/Clipping.hpp"
#include <vector>

namespace rbc { namespace clipping {

    namespace
    {
        bool is_point_in_plane(const Plane &plane, const m3d::vec3 &p)
        {
            const m3d::scalar d = -m3d::dot(plane.normal, plane.point);
            return m3d::dot(p, plane.normal) + d >= 0.0;
        }

        bool plane_edge_intersection(const Plane &plane,
                                     const m3d::vec3 &start,
                                     const m3d::vec3 &end,
                                     m3d::vec3 &out)
        {
            constexpr m3d::scalar EPS = 1e-6;

            const m3d::vec3 ab = end - start;
            const m3d::scalar ab_p = m3d::dot(plane.normal, ab);
            if (m3d::abs(ab_p) <= EPS)
                return false;

            const m3d::scalar dist = -m3d::dot(plane.normal, plane.point);
            const m3d::vec3 p_co = plane.normal * (-dist);

            m3d::scalar fac = -m3d::dot(plane.normal, start - p_co) / ab_p;
            if (fac < 0.0) fac = 0.0;
            if (fac > 1.0) fac = 1.0;

            out = start + ab * fac;
            return true;
        }
    } // namespace

    int sutherland_hodgman(const m3d::vec3 *in, int n_in,
                           const Plane *planes, int n_planes,
                           m3d::vec3 *out, int out_capacity,
                           bool remove_only)
    {
        if (n_in <= 0 || n_planes <= 0 || out_capacity <= 0)
            return 0;

        std::vector<m3d::vec3> a;
        std::vector<m3d::vec3> b;
        a.reserve(static_cast<size_t>(n_in) * 2);
        b.reserve(static_cast<size_t>(n_in) * 2);
        a.assign(in, in + n_in);

        for (int p = 0; p < n_planes; ++p)
        {
            if (a.empty()) break;

            const Plane &plane = planes[p];
            b.clear();

            m3d::vec3 start_point = a.back();
            for (size_t j = 0; j < a.size(); ++j)
            {
                const m3d::vec3 end_point = a[j];
                const bool start_in = is_point_in_plane(plane, start_point);
                const bool end_in   = is_point_in_plane(plane, end_point);

                if (remove_only)
                {
                    if (end_in)
                        b.push_back(end_point);
                }
                else
                {
                    if (start_in && end_in)
                    {
                        b.push_back(end_point);
                    }
                    else if (start_in && !end_in)
                    {
                        m3d::vec3 cut;
                        if (plane_edge_intersection(plane, start_point, end_point, cut))
                            b.push_back(cut);
                    }
                    else if (!start_in && end_in)
                    {
                        m3d::vec3 cut;
                        if (plane_edge_intersection(plane, start_point, end_point, cut))
                            b.push_back(cut);
                        b.push_back(end_point);
                    }
                }
                start_point = end_point;
            }
            a.swap(b);
        }

        const int n = (static_cast<int>(a.size()) < out_capacity)
                        ? static_cast<int>(a.size())
                        : out_capacity;
        for (int i = 0; i < n; ++i)
            out[i] = a[i];
        return n;
    }

    bool skew_line_closest_points(const m3d::vec3 &p1, const m3d::vec3 &d1,
                                  const m3d::vec3 &p2, const m3d::vec3 &d2,
                                  m3d::vec3 &l1, m3d::vec3 &l2)
    {
        const m3d::scalar n1 = d1.x * d2.x + d1.y * d2.y + d1.z * d2.z;
        const m3d::scalar n2 = d2.x * d2.x + d2.y * d2.y + d2.z * d2.z;
        const m3d::scalar m1 = -d1.x * d1.x - d1.y * d1.y - d1.z * d1.z;
        const m3d::scalar m2 = -d2.x * d1.x - d2.y * d1.y - d2.z * d1.z;
        const m3d::scalar r1 = -d1.x * p2.x + d1.x * p1.x
                              - d1.y * p2.y + d1.y * p1.y
                              - d1.z * p2.z + d1.z * p1.z;
        const m3d::scalar r2 = -d2.x * p2.x + d2.x * p1.x
                              - d2.y * p2.y + d2.y * p1.y
                              - d2.z * p2.z + d2.z * p1.z;

        const m3d::scalar denom = (n1 * m2) - (n2 * m1);
        if (m3d::abs(denom) < m3d::EPSILON) return false;

        const m3d::scalar n = ((r1 * m2) - (r2 * m1)) / denom;
        const m3d::scalar m = ((n1 * r2) - (n2 * r1)) / denom;

        l1 = p1 + d1 * m;
        l2 = p2 + d2 * n;
        return true;
    }

}} // namespace rbc::clipping
