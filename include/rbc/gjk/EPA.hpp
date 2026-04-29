#pragma once

#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

/**
 * @file EPA.hpp
 * @brief Expanding Polytope Algorithm — penetration depth and normal recovery.
 * @ingroup rbc
 * @ingroup internals
 *
 * After GJK reports overlap (`Inside`), EPA grows the GJK simplex into a
 * polytope by repeatedly adding the support point in the direction of
 * the closest face. Termination yields the penetration normal (face
 * normal of the closest face) and depth (distance from origin to that
 * face). The result is a single contact point; a richer 1–4-point
 * manifold is built afterwards by `generate_manifold` in
 * [ContactManifoldGenerator.hpp](ContactManifoldGenerator.hpp).
 *
 * Reference: Ericson, *Real-Time Collision Detection*, §9.5.4 (EPA).
 */

namespace rbc
{

    /**
     * @brief One face of the EPA polytope.
     *
     * Faces are organised in a doubly-linked adjacency graph so the
     * "horizon" silhouette can be walked in O(face count) when expanding.
     *
     * @ingroup internals
     */
    struct EPAFace
    {
        m3d::vec3 n;          ///< Unit face normal (outward, away from origin).
        m3d::scalar d;        ///< Signed distance from origin to the face plane (≥ 0).
        SimplexVertex *v[3];  ///< CCW-ordered vertices (when viewed from outside).
        EPAFace *adjacent[3]; ///< Adjacent faces, one per edge.
        int edge_adj[3];      ///< Edge index on each adjacent face that links back to this one.
        bool obsolete;        ///< Marked `true` when the face is removed during expansion.
        unsigned int pass;    ///< Pass counter — used to detect already-visited faces during horizon traversal.
    };

    /**
     * @brief Closed ring of faces stitched around the new support point during expansion.
     *
     * The horizon is built CCW so the new faces share consistent winding
     * with their neighbours.
     *
     * @ingroup internals
     */
    struct EPAHorizon
    {
        EPAFace *cf;  ///< Most recently added horizon face (tail).
        EPAFace *ff;  ///< First horizon face added (head; used to close the ring).
        int nf;       ///< Number of faces added to the horizon so far.
        EPAHorizon() : cf(nullptr), ff(nullptr), nf(0) {}
    };

    /**
     * @brief EPA solver state and entry point.
     *
     * Owns its face and vertex pools (heap-allocated in the constructor,
     * freed in the destructor). Reusable: each call to `evaluate` resets
     * internal counters.
     *
     * @ingroup rbc
     * @ingroup internals
     */
    struct EPA
    {
        /**
         * @brief Outcome of `evaluate`.
         *
         * - `Valid`        — converged; `normal`, `depth`, `contact_point` are filled.
         * - `Failed`       — numerical breakdown.
         * - `OutOfFaces`   — face pool exhausted (raise `max_faces` if it recurs).
         * - `OutOfVertices`— vertex pool exhausted.
         * - `NonLinear`    — degenerate simplex feeding from GJK.
         */
        enum Status
        {
            Valid,
            Failed,
            OutOfFaces,
            OutOfVertices,
            NonLinear
        } status;

        unsigned int max_faces     = 256;  ///< Face-pool capacity.
        unsigned int max_vertices  = 128;  ///< Vertex-pool capacity.
        unsigned int max_iterations = 128; ///< Expansion-iteration cap.
        m3d::scalar  tolerance     = 1e-6; ///< Convergence threshold on face-distance change.

        m3d::vec3   normal;          ///< Penetration normal, A→B convention.
        m3d::scalar depth;           ///< Penetration depth (positive on overlap).
        m3d::vec3   contact_point;   ///< World-space contact point on the closest face.

        EPAFace       **faces;       ///< Pre-allocated face pool.
        SimplexVertex **vertices;    ///< Pre-allocated vertex pool.
        unsigned int    num_faces;
        unsigned int    num_vertices;

        /** @brief Allocate the face and vertex pools. */
        EPA();
        /** @brief Release the face and vertex pools. */
        ~EPA();

        /**
         * @brief Run EPA seeded by a converged GJK simplex.
         *
         * @param gjk_solver A solver whose `evaluate` returned `Inside`.
         * @param shape      The same Minkowski difference passed to GJK.
         * @return `Valid` on success; otherwise an error code.
         */
        Status evaluate(const GJK &gjk_solver, const MinkowskiDiff &shape);

        /** @brief Reset face/vertex counters before a new evaluation. @ingroup internals */
        void      initialize();
        /** @brief Find the polytope face nearest the origin (drives the next expansion). @ingroup internals */
        EPAFace  *find_closest_face();
        /** @brief Wire two faces together along a shared edge. @ingroup internals */
        void      bind_faces(EPAFace *f0, int e0, EPAFace *f1, int e1);

        /**
         * @brief Walk the adjacency graph from `f` along `e` to find the horizon silhouette.
         *
         * Faces visible from `w` are marked obsolete; for each horizon edge
         * a new face is created and appended to `horizon`.
         *
         * @ingroup internals
         */
        bool expand(unsigned int pass, SimplexVertex *w,
                    EPAFace *f, int e, EPAHorizon &horizon);

    private:
        /**
         * @brief Allocate a new face from the pool.
         *
         * @return `nullptr` if the face pool is exhausted or the candidate
         *         face is degenerate (zero area).
         */
        EPAFace *new_face(SimplexVertex *a, SimplexVertex *b, SimplexVertex *c);
    };

} // namespace rbc
