#include "rbc/gjk/EPA.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rbc
{

    // ---------------------------------------------------------------------------
    // Construction / destruction
    // ---------------------------------------------------------------------------

    EPA::EPA()
    {
        faces = new EPAFace *[max_faces];
        vertices = new SimplexVertex *[max_vertices];
        for (unsigned int i = 0; i < max_faces; ++i)
            faces[i] = new EPAFace();
        for (unsigned int i = 0; i < max_vertices; ++i)
            vertices[i] = new SimplexVertex();
        initialize();
    }

    EPA::~EPA()
    {
        for (unsigned int i = 0; i < max_faces; ++i)
            delete faces[i];
        for (unsigned int i = 0; i < max_vertices; ++i)
            delete vertices[i];
        delete[] faces;
        delete[] vertices;
    }

    void EPA::initialize()
    {
        num_faces = 0;
        num_vertices = 0;
        status = Failed;
        normal = m3d::vec3(0, 0, 0);
        depth = 0;
        contact_point = m3d::vec3(0, 0, 0);
    }

    // ---------------------------------------------------------------------------
    // bind_faces  –  connects edge e0 of f0 to edge e1 of f1
    // ---------------------------------------------------------------------------

    void EPA::bind_faces(EPAFace *f0, int e0, EPAFace *f1, int e1)
    {
        f0->edge_adj[e0] = e1;
        f0->adjacent[e0] = f1;
        f1->edge_adj[e1] = e0;
        f1->adjacent[e1] = f0;
    }

    // ---------------------------------------------------------------------------
    // new_face  –  allocate one face from the pool and compute its normal / d.
    //              Returns nullptr for a degenerate or pool-exhausted case.
    // ---------------------------------------------------------------------------

    EPAFace *EPA::new_face(SimplexVertex *a, SimplexVertex *b, SimplexVertex *c)
    {
        if (num_faces >= max_faces)
        {
            status = OutOfFaces;
            return nullptr;
        }

        EPAFace *f = faces[num_faces++];
        f->v[0] = a;
        f->v[1] = b;
        f->v[2] = c;
        f->obsolete = false;
        f->pass = 0;

        // Normal = normalised (b-a) × (c-a)
        f->n = m3d::cross(b->w - a->w, c->w - a->w);
        m3d::scalar l = m3d::length(f->n);

        if (l > m3d::EPSILON)
        {
            f->n = f->n / l;
            f->d = m3d::dot(a->w, f->n); // distance from origin to face plane
            if (f->d < -m3d::EPSILON)
            {
                f->n = -f->n;
                f->d = -f->d;
            }
        }
        else
        {
            // Degenerate face – remove it from the pool and bail.
            --num_faces;
            return nullptr;
        }

        return f;
    }

    // ---------------------------------------------------------------------------
    // find_closest_face  –  returns the non-obsolete face whose plane is closest
    //                       to the origin (minimum d).
    // ---------------------------------------------------------------------------

    EPAFace *EPA::find_closest_face()
    {
        EPAFace *best = nullptr;
        m3d::scalar best_d = std::numeric_limits<m3d::scalar>::max();

        for (unsigned int i = 0; i < num_faces; ++i)
        {
            if (faces[i]->obsolete)
                continue;

            m3d::scalar d = faces[i]->d;

            // Skip clearly invalid (back-facing) faces caused by FP noise.
            // 10*EPSILON is conservative but safe for double/float.
            if (d < -m3d::EPSILON * 10.0)
                continue;

            m3d::scalar sqd = d * d; // squared distance to origin
            if (sqd < best_d)
            {
                best_d = sqd;
                best = faces[i];
            }
        }
        return best;
    }

    // ---------------------------------------------------------------------------
    // expand  –  recursive horizon-finding step.
    //
    //  pass    : current iteration counter (prevents revisiting)
    //  w       : the new support vertex being added
    //  f       : the face we arrived at via edge e of its neighbour
    //  e       : the edge index on f that was shared with the calling face
    //  horizon : the growing ring of new faces
    //
    //  Two cases:
    //  A) w is BEHIND f (dot(f->n, w-f->v[e]) < 0):
    //     → edge e is a horizon edge; create a new face (f->v[(e+1)%3], f->v[e], w)
    //       and stitch it into the horizon ring.
    //  B) w is IN FRONT OF f (face is visible):
    //     → f must be removed; recurse to its two other neighbours.
    // ---------------------------------------------------------------------------

    bool EPA::expand(unsigned int pass, SimplexVertex *w,
                     EPAFace *f, int e, EPAHorizon &horizon)
    {
        // Defensive: a missing adjacency means the polytope was built in
        // an inconsistent state. Bail cleanly — the outer loop's
        // `if (!valid || horizon.nf < 3)` accepts the best face so far.
        if (!f)
            return false;

        //If we hit a face already visited in this pass, it's just an internal
        // edge of the visible region. We return TRUE to continue safely.
        if (f->pass == pass)
            return true;

        const int e1 = (e + 1) % 3;
        const int e2 = (e + 2) % 3;

        // Case A: w is behind f → this edge is part of the horizon.
        if (m3d::dot(f->n, w->w - f->v[e]->w) < -m3d::EPSILON)
        {
            // New face winds: (f->v[e1], f->v[e], w)
            //   edge 0 → connects back to f through edge e  (set by bind_faces below)
            //   edge 1 → will connect to the NEXT horizon face's edge 2
            //   edge 2 → will connect to the PREVIOUS horizon face's edge 1
            EPAFace *nf = new_face(f->v[e1], f->v[e], w);
            if (!nf)
                return false;

            // Stitch new face edge-0 to the existing face edge-e
            bind_faces(nf, 0, f, e);

            // Chain into the horizon ring
            if (horizon.cf)
                bind_faces(horizon.cf, 1, nf, 2);
            else
                horizon.ff = nf; // first face in the ring

            horizon.cf = nf;
            ++horizon.nf;
            return true;
        }

        // Case B: f is visible from w → mark it obsolete and recurse.
        f->pass = pass;
        if (expand(pass, w, f->adjacent[e1], f->edge_adj[e1], horizon) && expand(pass, w, f->adjacent[e2], f->edge_adj[e2], horizon))
        {
            f->obsolete = true;
            return true;
        }
        return false;
    }

    // ---------------------------------------------------------------------------
    // evaluate  –  main EPA entry point
    // ---------------------------------------------------------------------------

    EPA::Status EPA::evaluate(const GJK &gjk_solver, const MinkowskiDiff &shape)
    {
        const Simplex &simplex = *gjk_solver.active_simplex;

        if (simplex.rank < 3)
            return Failed;

        initialize();

        // --- 1. Copy GJK simplex into our vertex pool -------------------------
        unsigned int n = (simplex.rank == 3) ? 3 : 4;
        for (unsigned int i = 0; i < n; ++i)
        {
            *vertices[num_vertices] = *simplex.vertex[i];
            ++num_vertices;
        }

        // Deduplicate. GJK's project_tetrahedron_origin can deliver a rank-4
        // simplex with coincident vertices (it doesn't validate non-zero
        // tetrahedral volume). Building faces over duplicates produces
        // zero-area cross products in new_face(), which returns null and
        // makes the rank-4 wiring abort with Failed. Fold duplicates here
        // and let the appropriate path (rank-3 promotion or rank-4 wiring)
        // handle the cleaned-up vertex set.
        {
            const m3d::scalar dedup_tol_sq = m3d::EPSILON * m3d::EPSILON;
            unsigned int unique_n = 0;
            for (unsigned int i = 0; i < num_vertices; ++i)
            {
                bool dup = false;
                for (unsigned int j = 0; j < unique_n; ++j)
                {
                    if (m3d::length_sq(vertices[i]->w - vertices[j]->w) < dedup_tol_sq)
                    {
                        dup = true;
                        break;
                    }
                }
                if (!dup)
                {
                    if (unique_n != i)
                        std::swap(vertices[unique_n], vertices[i]);
                    ++unique_n;
                }
            }
            num_vertices = unique_n;
        }

        if (num_vertices < 3)
            return Failed; // cannot even form a triangle

        // Promote rank-3 (or rank-4-degenerated-to-3) to rank-4 by adding a
        // support point off the triangle plane. GJK reaches this state for
        // vertex-on-face tangent contacts (where enclose_origin couldn't
        // extrude) or when project_tetrahedron returned a degenerate simplex
        // with duplicates (caught above by dedup).
        if (num_vertices == 3)
        {
            const m3d::vec3 a = vertices[0]->w;
            const m3d::vec3 b = vertices[1]->w;
            const m3d::vec3 c = vertices[2]->w;
            m3d::vec3 plane_n = m3d::cross(b - a, c - a);
            const m3d::scalar plane_n_len = m3d::length(plane_n);
            if (plane_n_len < m3d::EPSILON)
                return NonLinear; // degenerate (collinear) triangle from GJK
            plane_n = plane_n / plane_n_len;

            if (num_vertices >= max_vertices)
                return OutOfVertices;
            SimplexVertex *w = vertices[num_vertices];

            // Probe Minkowski-difference supports on both sides of the plane.
            const m3d::vec3 dir_a_pos = shape.tf_a.inverse_rotate_vector( plane_n);
            const m3d::vec3 dir_b_pos = shape.tf_b.inverse_rotate_vector(-plane_n);
            const m3d::vec3 w0_pos = shape.tf_a.transform_point(shape_support(*shape.shape_a, dir_a_pos));
            const m3d::vec3 w1_pos = shape.tf_b.transform_point(shape_support(*shape.shape_b, dir_b_pos));
            const m3d::vec3 w_pos  = w0_pos - w1_pos;
            const m3d::scalar d_pos = m3d::dot(w_pos - a, plane_n);

            const m3d::vec3 dir_a_neg = shape.tf_a.inverse_rotate_vector(-plane_n);
            const m3d::vec3 dir_b_neg = shape.tf_b.inverse_rotate_vector( plane_n);
            const m3d::vec3 w0_neg = shape.tf_a.transform_point(shape_support(*shape.shape_a, dir_a_neg));
            const m3d::vec3 w1_neg = shape.tf_b.transform_point(shape_support(*shape.shape_b, dir_b_neg));
            const m3d::vec3 w_neg  = w0_neg - w1_neg;
            const m3d::scalar d_neg = m3d::dot(w_neg - a, plane_n);

            // Origin's signed distance from the triangle plane (in +n).
            // Pick the support on the SAME side as origin so the resulting
            // tetrahedron actually encloses origin. By convexity of the
            // Minkowski difference, support distance ≥ |d_origin| on the
            // matching side. Fall back to the larger-abs side when origin
            // is on the plane (d_origin ≈ 0).
            const m3d::scalar d_origin = -m3d::dot(a, plane_n);
            const bool prefer_pos = (d_origin > m3d::EPSILON) ||
                                    (d_origin >= -m3d::EPSILON &&
                                     std::abs(d_pos) >= std::abs(d_neg));

            if (prefer_pos)
            {
                if (std::abs(d_pos) < tolerance)
                    return Failed; // truly tangent — no penetration volume
                w->w0 = w0_pos; w->w1 = w1_pos; w->w = w_pos;
            }
            else
            {
                if (std::abs(d_neg) < tolerance)
                    return Failed;
                w->w0 = w0_neg; w->w1 = w1_neg; w->w = w_neg;
            }
            ++num_vertices;
            // Fall through to the rank-4 face-build block below.
        }

        // Wire up four faces with full adjacency. Reaches here for both
        // GJK rank == 4 and rank == 3 promoted via the block above.
        {
            // Ensure consistent CCW winding (origin inside, normals outward)
            if (m3d::dot(vertices[0]->w - vertices[3]->w,
                         m3d::cross(vertices[1]->w - vertices[3]->w,
                                    vertices[2]->w - vertices[3]->w)) < 0)
            {
                std::swap(vertices[0], vertices[1]);
            }

            EPAFace *f0 = new_face(vertices[0], vertices[1], vertices[2]);
            EPAFace *f1 = new_face(vertices[1], vertices[0], vertices[3]);
            EPAFace *f2 = new_face(vertices[2], vertices[1], vertices[3]);
            EPAFace *f3 = new_face(vertices[0], vertices[2], vertices[3]);

            if (!f0 || !f1 || !f2 || !f3)
                return Failed;

            bind_faces(f0, 0, f1, 0);
            bind_faces(f0, 1, f2, 0);
            bind_faces(f0, 2, f3, 0);
            bind_faces(f1, 1, f3, 2);
            bind_faces(f1, 2, f2, 1);
            bind_faces(f2, 2, f3, 1);
        }

        // --- 2. Expansion loop ------------------------------------------------
        EPAFace *best_face = nullptr;
        unsigned int pass = 0;

        for (unsigned int iteration = 0; iteration < max_iterations; ++iteration)
        {
            best_face = find_closest_face();
            if (!best_face)
            {
                status = Failed;
                break;
            }

            if (num_vertices >= max_vertices)
            {
                status = OutOfVertices;
                break;
            }

            SimplexVertex *w = vertices[num_vertices];

            m3d::vec3 local_dir_a = shape.tf_a.inverse_rotate_vector(best_face->n);
            m3d::vec3 local_dir_b = shape.tf_b.inverse_rotate_vector(-best_face->n);
            w->w0 = shape.tf_a.transform_point(shape_support(*shape.shape_a, local_dir_a));
            w->w1 = shape.tf_b.transform_point(shape_support(*shape.shape_b, local_dir_b));
            w->w = w->w0 - w->w1;

            m3d::scalar w_dist = m3d::dot(best_face->n, w->w) - best_face->d;
            if (w_dist <= tolerance)
            {
                status = Valid;
                break;
            }

            ++num_vertices;

            EPAHorizon horizon;
            best_face->pass = ++pass;

            bool valid = true;
            for (int j = 0; j < 3 && valid; ++j)
                valid = expand(pass, w, best_face->adjacent[j], best_face->edge_adj[j], horizon);

            if (!valid || horizon.nf < 3)
            {
                status = Valid; // accept best we have
                break;
            }

            bind_faces(horizon.ff, 2, horizon.cf, 1);
            best_face->obsolete = true;
            status = Valid;
        }

        // --- 3. Extract results -----------------------------------------------
        if (status != Valid || !best_face)
            return status;

        normal = best_face->n;
        depth = best_face->d;

        // (your existing barycentric contact_point code – unchanged)
        {
            const m3d::vec3 &a = best_face->v[0]->w;
            const m3d::vec3 &b = best_face->v[1]->w;
            const m3d::vec3 &c = best_face->v[2]->w;
            const m3d::vec3 &a0 = best_face->v[0]->w0;
            const m3d::vec3 &b0 = best_face->v[1]->w0;
            const m3d::vec3 &c0 = best_face->v[2]->w0;

            m3d::vec3 p = best_face->n * depth;
            m3d::vec3 ab = b - a;
            m3d::vec3 ac = c - a;
            m3d::vec3 ap = p - a;

            m3d::scalar d00 = m3d::dot(ab, ab);
            m3d::scalar d01 = m3d::dot(ab, ac);
            m3d::scalar d11 = m3d::dot(ac, ac);
            m3d::scalar d20 = m3d::dot(ap, ab);
            m3d::scalar d21 = m3d::dot(ap, ac);

            m3d::scalar denom = d00 * d11 - d01 * d01;

            if (std::abs(denom) > m3d::EPSILON)
            {
                m3d::scalar bv = (d11 * d20 - d01 * d21) / denom;
                m3d::scalar bw = (d00 * d21 - d01 * d20) / denom;
                m3d::scalar bu = 1.0f - bv - bw;
                contact_point = bu * a0 + bv * b0 + bw * c0;
            }
            else
            {
                contact_point = (a0 + b0 + c0) * (1.0f / 3.0f);
            }
        }

        return status;
    }

} // namespace rbc