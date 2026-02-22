#include "rbc/gjk/GJK.hpp"
#include <algorithm>
#include <cmath>

namespace rbc
{

    // Helper for triple product
    inline m3d::scalar triple_product(const m3d::vec3 &a, const m3d::vec3 &b, const m3d::vec3 &c)
    {
        return m3d::dot(a, m3d::cross(b, c));
    }

    GJK::GJK()
    {
        initialize();
    }

    void GJK::initialize()
    {
        nfree = 0;
        status = Failed;
        active_simplex = nullptr;
        distance = 0.0;
        ray = m3d::vec3(0, 0, 0);
    }

    void GJK::remove_vertex(Simplex &s)
    {
        free_v[nfree++] = s.vertex[--s.rank];
    }

    void GJK::append_vertex(Simplex &s, const m3d::vec3 &v)
    {
        s.vertex[s.rank] = free_v[--nfree];

        // Compute support points in local space, then transform to world space
        m3d::vec3 local_dir_a = current_shape->tf_a.inverse_rotate_vector(v);
        s.vertex[s.rank]->w0 = current_shape->tf_a.transform_point(current_shape->shape_a->support(local_dir_a));

        m3d::vec3 local_dir_b = current_shape->tf_b.inverse_rotate_vector(-v);
        s.vertex[s.rank]->w1 = current_shape->tf_b.transform_point(current_shape->shape_b->support(local_dir_b));

        // Minkowski difference vertex
        s.vertex[s.rank]->w = s.vertex[s.rank]->w0 - s.vertex[s.rank]->w1;
        s.rank++;
    }

    bool GJK::enclose_origin()
    {
        m3d::vec3 axis(0, 0, 0);
        switch (active_simplex->rank)
        {
        case 1:
            for (int i = 0; i < 3; ++i)
            {
                axis[i] = 1;
                append_vertex(*active_simplex, axis);
                if (enclose_origin())
                    return true;
                remove_vertex(*active_simplex);
                axis[i] = -1;
                append_vertex(*active_simplex, -axis);
                if (enclose_origin())
                    return true;
                remove_vertex(*active_simplex);
                axis[i] = 0;
            }
            break;
        case 2:
        {
            m3d::vec3 d = active_simplex->vertex[1]->w - active_simplex->vertex[0]->w;
            for (int i = 0; i < 3; ++i)
            {
                axis[i] = 1;
                m3d::vec3 p = m3d::cross(d, axis);
                if (m3d::length_sq(p) > 0)
                {
                    append_vertex(*active_simplex, p);
                    if (enclose_origin())
                        return true;
                    remove_vertex(*active_simplex);
                    append_vertex(*active_simplex, -p);
                    if (enclose_origin())
                        return true;
                    remove_vertex(*active_simplex);
                }
                axis[i] = 0;
            }
        }
        break;
        case 3:
            axis = m3d::cross(active_simplex->vertex[1]->w - active_simplex->vertex[0]->w,
                              active_simplex->vertex[2]->w - active_simplex->vertex[0]->w);
            if (m3d::length_sq(axis) > 0)
            {
                append_vertex(*active_simplex, axis);
                if (enclose_origin())
                    return true;
                remove_vertex(*active_simplex);
                append_vertex(*active_simplex, -axis);
                if (enclose_origin())
                    return true;
                remove_vertex(*active_simplex);
            }
            break;
        case 4:
            if (std::abs(triple_product(
                    active_simplex->vertex[0]->w - active_simplex->vertex[3]->w,
                    active_simplex->vertex[1]->w - active_simplex->vertex[3]->w,
                    active_simplex->vertex[2]->w - active_simplex->vertex[3]->w)) > 0)
            {
                return true;
            }
            break;
        }
        return false;
    }

    GJK::Status GJK::evaluate(const MinkowskiDiff &shape, const m3d::vec3 &initial_guess)
    {
        current_shape = &shape;
        free_v[0] = &store_v[0];
        free_v[1] = &store_v[1];
        free_v[2] = &store_v[2];
        free_v[3] = &store_v[3];
        nfree = 4;
        current = 0;
        status = Valid;
        distance = 0.0;
        simplices[0].rank = 0;

        m3d::scalar rl = m3d::length(initial_guess);
        ray = (rl < tolerance) ? m3d::vec3(-1, 0, 0) : initial_guess;
        rl = (rl < tolerance) ? 1.0 : rl;

        size_t iterations = 0;
        m3d::scalar alpha = 0;

        do
        {
            int next = 1 - current;
            Simplex &curr_simplex = simplices[current];
            Simplex &next_simplex = simplices[next];

            if (rl < tolerance)
            {
                status = Inside;
                break;
            }

            append_vertex(curr_simplex, -ray);
            const m3d::vec3 &w = curr_simplex.vertex[curr_simplex.rank - 1]->w;

            m3d::scalar omega = m3d::dot(ray, w) / rl;
            alpha = std::max(alpha, omega);

            if (iterations > 0 && (rl - alpha) - tolerance * rl <= 0)
            {
                remove_vertex(curr_simplex);
                distance = rl;
                if (distance < tolerance)
                    status = Inside;
                break;
            }

            bool inside = false;
            switch (curr_simplex.rank)
            {
            case 1:
                ray = w;
                inside = false;
                next_simplex.rank = 1;
                next_simplex.vertex[0] = curr_simplex.vertex[0];
                break;
            case 2:
                inside = project_line_origin(curr_simplex, next_simplex);
                break;
            case 3:
                inside = project_triangle_origin(curr_simplex, next_simplex);
                break;
            case 4:
                inside = project_tetrahedron_origin(curr_simplex, next_simplex);
                break;
            }

            current = next;
            if (!inside)
                rl = m3d::length(ray);
            if (inside || rl == 0)
            {
                status = Inside;
                break;
            }

            status = (++iterations < max_iterations) ? status : Failed;
        } while (status == Valid);

        active_simplex = &simplices[current];
        if (status == Inside && active_simplex->rank < 4)
        {
            // Attempt to build a tetrahedron that still contains the origin
            if (!enclose_origin())
            {
                // If it fails, we keep the current simplex (origin is inside a triangle/edge)
                // but EPA will have to handle it (we'll also modify EPA to accept triangles).
            }
        }

        return status;
    }

    bool GJK::project_line_origin(const Simplex &curr, Simplex &next)
    {
        const m3d::vec3 &A = curr.vertex[1]->w;
        const m3d::vec3 &B = curr.vertex[0]->w;
        m3d::vec3 AB = B - A;
        m3d::scalar d = m3d::dot(AB, -A);

        if (d == 0)
        {
            ray = A;
            next.vertex[0] = curr.vertex[1];
            next.rank = 1;
            free_v[nfree++] = curr.vertex[0];
            return m3d::length_sq(A) == 0;
        }
        else if (d < 0)
        {
            ray = A;
            next.vertex[0] = curr.vertex[1];
            next.rank = 1;
            free_v[nfree++] = curr.vertex[0];
        }
        else
        {
            ray = m3d::dot(AB, B) * A + d * B;
            next.vertex[0] = curr.vertex[0];
            next.vertex[1] = curr.vertex[1];
            next.rank = 2;
            ray = ray / m3d::length_sq(AB);
        }
        return false;
    }

    bool GJK::project_triangle_origin(const Simplex &curr, Simplex &next)
    {
        const m3d::vec3 &A = curr.vertex[2]->w;
        const m3d::vec3 &B = curr.vertex[1]->w;
        const m3d::vec3 &C = curr.vertex[0]->w;

        m3d::vec3 AB = B - A;
        m3d::vec3 AC = C - A;
        m3d::vec3 ABC = m3d::cross(AB, AC);

        if (m3d::dot(m3d::cross(ABC, AC), -A) >= 0)
        {
            m3d::scalar towardsC = m3d::dot(AC, -A);
            if (towardsC >= 0)
            {
                ray = m3d::dot(AC, C) * A + towardsC * C;
                next.vertex[0] = curr.vertex[0];
                next.vertex[1] = curr.vertex[2];
                next.rank = 2;
                ray = ray / m3d::length_sq(AC);
                free_v[nfree++] = curr.vertex[1];
            }
            else
            {
                m3d::scalar towardsB = m3d::dot(AB, -A);
                if (towardsB < 0)
                {
                    ray = A;
                    next.vertex[0] = curr.vertex[2];
                    next.rank = 1;
                    free_v[nfree++] = curr.vertex[1];
                }
                else
                {
                    ray = m3d::dot(AB, B) * A + towardsB * B;
                    next.vertex[0] = curr.vertex[1];
                    next.vertex[1] = curr.vertex[2];
                    next.rank = 2;
                    ray = ray / m3d::length_sq(AB);
                }
                free_v[nfree++] = curr.vertex[0];
            }
        }
        else
        {
            if (m3d::dot(m3d::cross(AB, ABC), -A) >= 0)
            {
                m3d::scalar towardsB = m3d::dot(AB, -A);
                if (towardsB < 0)
                {
                    ray = A;
                    next.vertex[0] = curr.vertex[2];
                    next.rank = 1;
                    free_v[nfree++] = curr.vertex[1];
                }
                else
                {
                    ray = m3d::dot(AB, B) * A + towardsB * B;
                    next.vertex[0] = curr.vertex[1];
                    next.vertex[1] = curr.vertex[2];
                    next.rank = 2;
                    ray = ray / m3d::length_sq(AB);
                }
                free_v[nfree++] = curr.vertex[0];
            }
            else
            {
                next.rank = 3;
                next.vertex[2] = curr.vertex[2];
                m3d::scalar ABCdotAO = m3d::dot(ABC, -A);

                if (ABCdotAO == 0)
                {
                    next.vertex[0] = curr.vertex[0];
                    next.vertex[1] = curr.vertex[1];
                    ray = m3d::vec3(0, 0, 0);
                    return true;
                }
                if (ABCdotAO > 0)
                {
                    next.vertex[0] = curr.vertex[0];
                    next.vertex[1] = curr.vertex[1];
                }
                else
                {
                    next.vertex[0] = curr.vertex[1];
                    next.vertex[1] = curr.vertex[0];
                }
                ray = -ABCdotAO / m3d::length_sq(ABC) * ABC;
                return false;
            }
        }
        return false;
    }

    bool GJK::project_tetrahedron_origin(const Simplex &curr, Simplex &next)
    {
        const m3d::vec3 &A = curr.vertex[3]->w;
        const m3d::vec3 &B = curr.vertex[2]->w;
        const m3d::vec3 &C = curr.vertex[1]->w;
        const m3d::vec3 &D = curr.vertex[0]->w;

        m3d::vec3 ABC = m3d::cross(B - A, C - A);
        m3d::vec3 ACD = m3d::cross(C - A, D - A);
        m3d::vec3 ADB = m3d::cross(D - A, B - A);

        if (m3d::dot(ABC, -A) > 0)
        {
            free_v[nfree++] = curr.vertex[0];
            return project_triangle_origin(curr, next);
        }
        if (m3d::dot(ACD, -A) > 0)
        {
            free_v[nfree++] = curr.vertex[2];
            return project_triangle_origin(curr, next);
        }
        if (m3d::dot(ADB, -A) > 0)
        {
            free_v[nfree++] = curr.vertex[1];
            return project_triangle_origin(curr, next);
        }

        ray = m3d::vec3(0, 0, 0);
        next.vertex[0] = curr.vertex[0];
        next.vertex[1] = curr.vertex[1];
        next.vertex[2] = curr.vertex[2];
        next.vertex[3] = curr.vertex[3];
        next.rank = 4;
        return true;
    }

} // namespace rbc