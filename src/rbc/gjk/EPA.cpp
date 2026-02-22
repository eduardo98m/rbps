#include "rbc/gjk/EPA.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rbc {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

EPA::EPA()
{
    faces    = new EPAFace*[max_faces];
    vertices = new SimplexVertex*[max_vertices];
    for (unsigned int i = 0; i < max_faces;    ++i) faces[i]    = new EPAFace();
    for (unsigned int i = 0; i < max_vertices; ++i) vertices[i] = new SimplexVertex();
    initialize();
}

EPA::~EPA()
{
    for (unsigned int i = 0; i < max_faces;    ++i) delete faces[i];
    for (unsigned int i = 0; i < max_vertices; ++i) delete vertices[i];
    delete[] faces;
    delete[] vertices;
}

void EPA::initialize()
{
    num_faces    = 0;
    num_vertices = 0;
    status       = Failed;
    normal       = m3d::vec3(0, 0, 0);
    depth        = 0;
    contact_point = m3d::vec3(0, 0, 0);
}

// ---------------------------------------------------------------------------
// bind_faces  –  connects edge e0 of f0 to edge e1 of f1
// ---------------------------------------------------------------------------

void EPA::bind_faces(EPAFace* f0, int e0, EPAFace* f1, int e1)
{
    f0->edge_adj[e0] = e1;  f0->adjacent[e0] = f1;
    f1->edge_adj[e1] = e0;  f1->adjacent[e1] = f0;
}

// ---------------------------------------------------------------------------
// new_face  –  allocate one face from the pool and compute its normal / d.
//              Returns nullptr for a degenerate or pool-exhausted case.
// ---------------------------------------------------------------------------

EPAFace* EPA::new_face(SimplexVertex* a, SimplexVertex* b, SimplexVertex* c)
{
    if (num_faces >= max_faces) {
        status = OutOfFaces;
        return nullptr;
    }

    EPAFace* f = faces[num_faces++];
    f->v[0]    = a;
    f->v[1]    = b;
    f->v[2]    = c;
    f->obsolete = false;
    f->pass     = 0;

    // Normal = normalised (b-a) × (c-a)
    f->n = m3d::cross(b->w - a->w, c->w - a->w);
    m3d::scalar l = m3d::length(f->n);

    if (l > m3d::EPSILON) {
        f->n = f->n / l;
        f->d  = m3d::dot(a->w, f->n);   // distance from origin to face plane
    } else {
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

EPAFace* EPA::find_closest_face()
{
    EPAFace*    best = nullptr;
    m3d::scalar best_d = std::numeric_limits<m3d::scalar>::max();

    for (unsigned int i = 0; i < num_faces; ++i) {
        if (faces[i]->obsolete) continue;
        // Use squared distance so that faces with d ≈ 0 are still found correctly
        // and a face with d=-ε (numerical noise) does not win over a valid face.
        m3d::scalar d = faces[i]->d;
        if (d < best_d) {
            best_d = d;
            best   = faces[i];
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

bool EPA::expand(unsigned int pass, SimplexVertex* w,
                 EPAFace* f, int e, EPAHorizon& horizon)
{
    // If we reach a face we've already processed this pass the hull is broken.
    if (f->pass == pass) return false;

    const int e1 = (e + 1) % 3;
    const int e2 = (e + 2) % 3;

    // Case A: w is behind f → this edge is part of the horizon.
    if (m3d::dot(f->n, w->w - f->v[e]->w) < -m3d::EPSILON) {
        // New face winds: (f->v[e1], f->v[e], w)
        //   edge 0 → connects back to f through edge e  (set by bind_faces below)
        //   edge 1 → will connect to the NEXT horizon face's edge 2
        //   edge 2 → will connect to the PREVIOUS horizon face's edge 1
        EPAFace* nf = new_face(f->v[e1], f->v[e], w);
        if (!nf) return false;

        // Stitch new face edge-0 to the existing face edge-e
        bind_faces(nf, 0, f, e);

        // Chain into the horizon ring
        if (horizon.cf)
            bind_faces(horizon.cf, 1, nf, 2);
        else
            horizon.ff = nf;   // first face in the ring

        horizon.cf = nf;
        ++horizon.nf;
        return true;
    }

    // Case B: f is visible from w → mark it obsolete and recurse.
    f->pass = pass;
    if (   expand(pass, w, f->adjacent[e1], f->edge_adj[e1], horizon)
        && expand(pass, w, f->adjacent[e2], f->edge_adj[e2], horizon))
    {
        f->obsolete = true;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// evaluate  –  main EPA entry point
// ---------------------------------------------------------------------------

EPA::Status EPA::evaluate(const GJK& gjk_solver, const MinkowskiDiff& shape)
{
    const Simplex& simplex = *gjk_solver.active_simplex;

    if (simplex.rank < 4)
        return Failed;

    initialize();

    // --- 1. Copy the GJK tetrahedron vertices into EPA's vertex pool ----------

    for (int i = 0; i < 4; ++i) {
        *vertices[num_vertices] = *simplex.vertex[i];
        ++num_vertices;
    }

    // Ensure CCW winding (origin inside the tetrahedron on the positive side of every face).
    // The test: dot( v0-v3, (v1-v3) × (v2-v3) ) must be > 0.
    // If it is < 0 the winding is reversed; swap v0 and v1 to fix it.
    if (m3d::dot(vertices[0]->w - vertices[3]->w,
                 m3d::cross(vertices[1]->w - vertices[3]->w,
                            vertices[2]->w - vertices[3]->w)) < 0) {
        std::swap(vertices[0], vertices[1]);
    }

    // --- 2. Build the initial 4 faces ------------------------------------------
    // Face winding mirrors hpp-fcl so that every normal points outward.

    EPAFace* f0 = new_face(vertices[0], vertices[1], vertices[2]);
    EPAFace* f1 = new_face(vertices[1], vertices[0], vertices[3]);
    EPAFace* f2 = new_face(vertices[2], vertices[1], vertices[3]);
    EPAFace* f3 = new_face(vertices[0], vertices[2], vertices[3]);

    if (!f0 || !f1 || !f2 || !f3)
        return Failed;

    bind_faces(f0, 0, f1, 0);
    bind_faces(f0, 1, f2, 0);
    bind_faces(f0, 2, f3, 0);
    bind_faces(f1, 1, f3, 2);
    bind_faces(f1, 2, f2, 1);
    bind_faces(f2, 2, f3, 1);

    // --- 3. Expansion loop -----------------------------------------------------

    EPAFace*     best_face  = nullptr;
    unsigned int pass       = 0;

    for (unsigned int iteration = 0; iteration < max_iterations; ++iteration) {

        best_face = find_closest_face();
        if (!best_face) { status = Failed; break; }

        if (num_vertices >= max_vertices) { status = OutOfVertices; break; }

        // Query the support point along the closest face's outward normal.
        SimplexVertex* w = vertices[num_vertices];

        m3d::vec3 local_dir_a =  shape.tf_a.inverse_rotate_vector( best_face->n);
        m3d::vec3 local_dir_b =  shape.tf_b.inverse_rotate_vector(-best_face->n);
        w->w0 = shape.tf_a.transform_point(shape.shape_a->support(local_dir_a));
        w->w1 = shape.tf_b.transform_point(shape.shape_b->support(local_dir_b));
        w->w  = w->w0 - w->w1;

        // Convergence check: how much further beyond the current face is w?
        m3d::scalar w_dist = m3d::dot(best_face->n, w->w) - best_face->d;
        if (w_dist <= tolerance) {
            // The polytope face is as close to the CSO boundary as the tolerance allows.
            status = Valid;
            break;
        }

        // w extends the polytope → commit it and expand.
        ++num_vertices;

        EPAHorizon horizon;
        best_face->pass = ++pass;

        bool valid = true;
        for (int j = 0; j < 3 && valid; ++j)
            valid = expand(pass, w, best_face->adjacent[j], best_face->edge_adj[j], horizon);

        if (!valid || horizon.nf < 3) {
            // Expansion failed – hull became invalid. Accept best result so far.
            status = Valid;
            break;
        }

        // Close the horizon ring: connect the last face's edge-1 to the first face's edge-2.
        bind_faces(horizon.ff, 2, horizon.cf, 1);

        // Retire the face we just replaced.
        best_face->obsolete = true;

        status = Valid;
    }

    // --- 4. Extract results ----------------------------------------------------

    if (status != Valid || !best_face)
        return status;

    normal = best_face->n;
    depth  = best_face->d;

    // Compute the contact point as the barycentric projection of the origin
    // onto the closest face (in Minkowski-difference space), then map those
    // same barycentric weights onto the support points of shape A.
    //
    // The projection of the origin onto the face plane is: p = depth * normal
    // Solve for barycentric coords (u, v, w) of p in triangle (a, b, c):
    //   p = u*a + v*b + w*c,  u+v+w = 1
    {
        const m3d::vec3& a  = best_face->v[0]->w;
        const m3d::vec3& b  = best_face->v[1]->w;
        const m3d::vec3& c  = best_face->v[2]->w;
        const m3d::vec3& a0 = best_face->v[0]->w0;
        const m3d::vec3& b0 = best_face->v[1]->w0;
        const m3d::vec3& c0 = best_face->v[2]->w0;

        m3d::vec3 p  = best_face->n * depth;   // origin projected onto face plane
        m3d::vec3 ab = b - a;
        m3d::vec3 ac = c - a;
        m3d::vec3 ap = p - a;

        m3d::scalar d00 = m3d::dot(ab, ab);
        m3d::scalar d01 = m3d::dot(ab, ac);
        m3d::scalar d11 = m3d::dot(ac, ac);
        m3d::scalar d20 = m3d::dot(ap, ab);
        m3d::scalar d21 = m3d::dot(ap, ac);

        m3d::scalar denom = d00 * d11 - d01 * d01;

        if (std::abs(denom) > m3d::EPSILON) {
            m3d::scalar bv = (d11 * d20 - d01 * d21) / denom;
            m3d::scalar bw = (d00 * d21 - d01 * d20) / denom;
            m3d::scalar bu = 1.0f - bv - bw;
            contact_point  = bu * a0 + bv * b0 + bw * c0;
        } else {
            // Degenerate triangle – fall back to centroid
            contact_point = (a0 + b0 + c0) * (1.0f / 3.0f);
        }
    }

    return status;
}

} // namespace rbc