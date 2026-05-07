#pragma once

#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

/**
 * @file EPA.hpp
 * @brief Expanding Polytope Algorithm — penetration depth and normal recovery.
 * @ingroup rbc
 *
 * After GJK reports overlap, EPA grows the GJK simplex into a polytope
 * by repeatedly adding the support point in the direction of the
 * closest-to-origin face. Termination yields the penetration normal and
 * depth. A 1–4-point manifold is built afterwards by `gjk_epa_manifold`
 * in [ContactManifoldGenerator.hpp](ContactManifoldGenerator.hpp).
 */

namespace rbc
{
    // One face of the EPA polytope. Vertices are CCW outward when
    // viewed from outside the polytope; `n` is the unit outward normal,
    // `d` the signed distance from origin to the face plane (>= 0
    // because the origin is inside).
    struct EPAFace
    {
        m3d::vec3       n;
        m3d::scalar     d;
        SimplexVertex  *v[3];
        bool            obsolete; ///< True after the face is removed during expansion.
    };

    struct EPA
    {
        enum Status
        {
            Valid,
            Failed,
            OutOfFaces,
            OutOfVertices,
            NonLinear
        } status;

        unsigned int max_faces      = 256;
        unsigned int max_vertices   = 128;
        unsigned int max_iterations = 128;
        m3d::scalar  tolerance      = 1e-6;

        m3d::vec3   normal;
        m3d::scalar depth;
        m3d::vec3   contact_point;

        EPAFace       **faces;
        SimplexVertex **vertices;
        unsigned int    num_faces;
        unsigned int    num_vertices;

        EPA();
        ~EPA();

        Status evaluate(const GJK &gjk_solver, const MinkowskiDiff &shape);
    };

} // namespace rbc
