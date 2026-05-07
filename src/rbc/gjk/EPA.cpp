// EPA — penetration depth and normal recovery via polytope expansion.
//
// Seed: the rank-4 GJK simplex (4 vertices, 4 triangle faces). Each
// iteration:
//   1. Find the alive face whose plane is closest to the origin.
//   2. Sample the Minkowski-difference support along that face's outward
//      normal.
//   3. If the support is no further out than the face plane (within
//      tolerance), terminate — the closest face is the answer.
//   4. Otherwise, mark every face visible from the new support
//      (centroid + normal-dot test) as obsolete, collect the silhouette
//      edges (cancellation rule: an edge that appears in both directions
//      is interior and removed), and stitch new faces from each
//      silhouette edge to the new vertex.
//
// Faces and vertices are heap-allocated into pre-allocated pointer pools
// (`faces[]`, `vertices[]`) so the visual debugger can scan them after
// `evaluate` returns. Removed faces stay in the pool with `obsolete =
// true`; the debugger filters.

#include "rbc/gjk/EPA.hpp"
#include <cfloat>
#include <cmath>
#include <vector>

namespace rbc
{
    namespace
    {
        // Tolerance for "is the origin coplanar with this face?" — used
        // when seeding the polytope from a degenerate GJK simplex.
        constexpr m3d::scalar kCoplanarTolerance = 1e-12;

        // Compute outward unit normal and unsigned distance for a face.
        // The returned normal is flipped if necessary so the rest of the
        // polytope vertices lie on its negative side (i.e. inside).
        bool compute_face_normal(EPAFace &f, SimplexVertex **all_verts,
                                 unsigned int n_all)
        {
            const m3d::vec3 a = f.v[0]->w;
            const m3d::vec3 b = f.v[1]->w;
            const m3d::vec3 c = f.v[2]->w;

            m3d::vec3 n = m3d::cross(b - a, c - a);
            const m3d::scalar n_len = m3d::length(n);
            if (n_len < m3d::EPSILON)
                return false; // degenerate triangle

            n = n / n_len;
            m3d::scalar d = m3d::dot(n, a);

            if (d < -kCoplanarTolerance)
            {
                n = -n;
                d = -d;
            }
            else if (d <= kCoplanarTolerance)
            {
                // Origin lies (numerically) on this face's plane. Pick
                // the orientation under which every other polytope
                // vertex sits behind the face.
                bool fixed = false;
                for (unsigned int i = 0; i < n_all; ++i)
                {
                    const m3d::scalar t = m3d::dot(n, all_verts[i]->w);
                    if (t > kCoplanarTolerance)
                    {
                        n = -n; d = 0.0;
                        fixed = true;
                        break;
                    }
                    if (t < -kCoplanarTolerance)
                    {
                        d = 0.0;
                        fixed = true;
                        break;
                    }
                }
                if (!fixed)
                    return false; // all vertices coplanar — degenerate polytope
            }

            f.n = n;
            f.d = d;
            f.obsolete = false;
            return true;
        }

        struct Edge { SimplexVertex *a; SimplexVertex *b; };

        // Add an edge with reverse-cancellation: if (b, a) is already
        // present, remove it and skip; otherwise append (a, b). The
        // remaining edges form the silhouette of the visible region.
        void add_edge(std::vector<Edge> &edges, SimplexVertex *a, SimplexVertex *b)
        {
            for (size_t i = 0; i < edges.size(); ++i)
            {
                if (edges[i].a == b && edges[i].b == a)
                {
                    edges.erase(edges.begin() + static_cast<long>(i));
                    return;
                }
            }
            edges.push_back({a, b});
        }
    } // namespace

    EPA::EPA()
        : status(Failed),
          normal(0.0, 0.0, 0.0),
          depth(0.0),
          contact_point(0.0, 0.0, 0.0),
          faces(nullptr),
          vertices(nullptr),
          num_faces(0),
          num_vertices(0)
    {
        faces    = new EPAFace*[max_faces]();
        vertices = new SimplexVertex*[max_vertices]();
    }

    EPA::~EPA()
    {
        for (unsigned int i = 0; i < num_faces; ++i)
            delete faces[i];
        for (unsigned int i = 0; i < num_vertices; ++i)
            delete vertices[i];
        delete[] faces;
        delete[] vertices;
    }

    EPA::Status EPA::evaluate(const GJK &gjk_solver, const MinkowskiDiff &shape)
    {
        // Reset state for a fresh evaluation.
        for (unsigned int i = 0; i < num_faces; ++i)    delete faces[i];
        for (unsigned int i = 0; i < num_vertices; ++i) delete vertices[i];
        num_faces    = 0;
        num_vertices = 0;
        normal        = m3d::vec3(0.0, 0.0, 0.0);
        depth         = 0.0;
        contact_point = m3d::vec3(0.0, 0.0, 0.0);

        const Simplex &seed = gjk_solver.simplex;
        if (seed.rank != 4)
        {
            status = NonLinear;
            return status;
        }

        // Copy the four simplex vertices into our owned vertex pool.
        for (int i = 0; i < 4; ++i)
        {
            if (num_vertices >= max_vertices) { status = OutOfVertices; return status; }
            vertices[num_vertices++] = new SimplexVertex(*seed.vertex[i]);
        }

        // Detect a coplanar seed (face-on-face touching contacts).
        // The Minkowski difference's surface passes through the origin
        // and the four GJK simplex vertices end up on it, so the seed
        // tetrahedron is flat and `compute_face_normal` rejects every
        // face. Sample the support perpendicular to the plane on each
        // side; if at least one side is flat (within tolerance), this
        // is a touching contact: return `Valid` with depth = 0 and the
        // plane's outward normal.
        {
            const m3d::vec3 va = vertices[0]->w;
            const m3d::vec3 vb = vertices[1]->w;
            const m3d::vec3 vc = vertices[2]->w;
            const m3d::vec3 vd = vertices[3]->w;

            m3d::vec3 plane_n = m3d::cross(vb - va, vc - va);
            if (m3d::length_sq(plane_n) < m3d::EPSILON * m3d::EPSILON)
                plane_n = m3d::cross(vb - va, vd - va);
            if (m3d::length_sq(plane_n) < m3d::EPSILON * m3d::EPSILON)
                plane_n = m3d::cross(vc - va, vd - va);

            if (m3d::length_sq(plane_n) > m3d::EPSILON * m3d::EPSILON)
            {
                plane_n = m3d::normalize(plane_n);
                const m3d::scalar dist_d = m3d::abs(m3d::dot(vd - va, plane_n));
                if (dist_d < 1e-5)
                {
                    // Coplanar seed. Sample perpendicular extent on each side.
                    const m3d::vec3 sp = shape.support_a( plane_n) - shape.support_b(-plane_n);
                    const m3d::vec3 sn = shape.support_a(-plane_n) - shape.support_b( plane_n);
                    const m3d::scalar dist_pos = m3d::dot(sp - va,  plane_n);
                    const m3d::scalar dist_neg = m3d::dot(sn - va, -plane_n);

                    constexpr m3d::scalar PERP = 1e-4;
                    if (dist_pos < PERP || dist_neg < PERP)
                    {
                        // Polytope is flat on at least one side — touching contact.
                        // Outward normal points into the side with less extent.
                        normal = (dist_pos <= dist_neg) ? plane_n : -plane_n;
                        depth  = 0.0;
                        contact_point = (vertices[0]->w0 + vertices[1]->w0
                                       + vertices[2]->w0 + vertices[3]->w0) * 0.25;
                        status = Valid;
                        return status;
                    }
                }
            }
        }

        SimplexVertex *vA = vertices[0];
        SimplexVertex *vB = vertices[1];
        SimplexVertex *vC = vertices[2];
        SimplexVertex *vD = vertices[3];

        auto add_face = [&](SimplexVertex *p, SimplexVertex *q, SimplexVertex *r) -> bool {
            if (num_faces >= max_faces) { status = OutOfFaces; return false; }
            EPAFace *f = new EPAFace;
            f->v[0] = p; f->v[1] = q; f->v[2] = r;
            f->obsolete = false;
            if (!compute_face_normal(*f, vertices, num_vertices))
            {
                delete f;
                return false;
            }
            faces[num_faces++] = f;
            return true;
        };

        if (!add_face(vA, vB, vC) ||
            !add_face(vA, vC, vD) ||
            !add_face(vA, vD, vB) ||
            !add_face(vB, vC, vD))
        {
            // status set by add_face on OutOfFaces; otherwise polytope is degenerate.
            if (status != OutOfFaces) status = NonLinear;
            return status;
        }

        std::vector<Edge> edges;
        edges.reserve(64);

        for (unsigned int it = 0; it < max_iterations; ++it)
        {
            // 1. Find the alive face nearest the origin.
            EPAFace *closest = nullptr;
            m3d::scalar min_d = DBL_MAX;
            unsigned int closest_idx = 0;
            for (unsigned int i = 0; i < num_faces; ++i)
            {
                EPAFace *f = faces[i];
                if (!f || f->obsolete) continue;
                if (f->d < min_d)
                {
                    min_d = f->d;
                    closest = f;
                    closest_idx = i;
                }
            }

            if (!closest)
            {
                status = Failed;
                return status;
            }

            // 2. Sample the support along the closest-face normal.
            const m3d::vec3 dir = closest->n;
            SimplexVertex *w = new SimplexVertex;
            w->w0 = shape.support_a(dir);
            w->w1 = shape.support_b(-dir);
            w->w  = w->w0 - w->w1;

            const m3d::scalar d_proj = m3d::dot(w->w, dir);

            // 3. Convergence: the support is on (or below) the face.
            if (d_proj - min_d < tolerance)
            {
                delete w;

                normal = closest->n;
                depth  = min_d;

                // Project origin onto the closest face's plane in
                // Minkowski-difference space, take that point's
                // barycentric weights over the face, and apply them to
                // v[k]->w0 to recover the world contact on shape A.
                const m3d::vec3 q = closest->n * min_d;
                const m3d::vec3 p0 = closest->v[0]->w;
                const m3d::vec3 p1 = closest->v[1]->w;
                const m3d::vec3 p2 = closest->v[2]->w;
                const m3d::vec3 e0 = p1 - p0;
                const m3d::vec3 e1 = p2 - p0;
                const m3d::vec3 r  = q - p0;
                const m3d::scalar d00 = m3d::dot(e0, e0);
                const m3d::scalar d01 = m3d::dot(e0, e1);
                const m3d::scalar d11 = m3d::dot(e1, e1);
                const m3d::scalar d20 = m3d::dot(r,  e0);
                const m3d::scalar d21 = m3d::dot(r,  e1);
                const m3d::scalar denom = d00 * d11 - d01 * d01;
                if (m3d::abs(denom) < m3d::EPSILON)
                {
                    contact_point = closest->v[0]->w0;
                }
                else
                {
                    const m3d::scalar v_w = (d11 * d20 - d01 * d21) / denom;
                    const m3d::scalar w_w = (d00 * d21 - d01 * d20) / denom;
                    const m3d::scalar u_w = 1.0 - v_w - w_w;
                    contact_point = closest->v[0]->w0 * u_w
                                  + closest->v[1]->w0 * v_w
                                  + closest->v[2]->w0 * w_w;
                }

                status = Valid;
                return status;
            }

            // Avoid duplicate vertices — if the support coincides with
            // an existing polytope vertex, we'd recurse forever.
            for (unsigned int i = 0; i < num_vertices; ++i)
            {
                if (m3d::length_sq(vertices[i]->w - w->w) < m3d::EPSILON * m3d::EPSILON)
                {
                    delete w;
                    normal = closest->n;
                    depth  = min_d;
                    contact_point = closest->v[0]->w0;
                    status = Valid;
                    return status;
                }
            }
            (void)closest_idx;

            // 4. Add the new vertex; mark every visible face obsolete
            //    and collect their silhouette edges.
            if (num_vertices >= max_vertices) { delete w; status = OutOfVertices; return status; }
            vertices[num_vertices++] = w;

            edges.clear();
            for (unsigned int i = 0; i < num_faces; ++i)
            {
                EPAFace *f = faces[i];
                if (!f || f->obsolete) continue;

                const m3d::vec3 centroid =
                    (f->v[0]->w + f->v[1]->w + f->v[2]->w) * (1.0 / 3.0);
                if (m3d::dot(f->n, w->w - centroid) > 0.0)
                {
                    f->obsolete = true;
                    add_edge(edges, f->v[0], f->v[1]);
                    add_edge(edges, f->v[1], f->v[2]);
                    add_edge(edges, f->v[2], f->v[0]);
                }
            }

            // Stitch new faces from each silhouette edge to the new vertex.
            for (const Edge &e : edges)
            {
                if (num_faces >= max_faces) { status = OutOfFaces; return status; }
                EPAFace *nf = new EPAFace;
                nf->v[0] = e.a; nf->v[1] = e.b; nf->v[2] = w;
                nf->obsolete = false;
                if (!compute_face_normal(*nf, vertices, num_vertices))
                {
                    delete nf;
                    continue;
                }
                faces[num_faces++] = nf;
            }
        }

        status = Failed;
        return status;
    }

} // namespace rbc
