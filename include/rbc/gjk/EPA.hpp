#pragma once

#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

namespace rbc
{

    struct EPAFace
    {
        m3d::vec3 n;          // Face normal (outward from polytope, away from origin)
        m3d::scalar d;        // Signed distance from origin to the face plane (>= 0 for valid faces)
        SimplexVertex *v[3];  // The three vertices making up the face (CCW when viewed from outside)
        EPAFace *adjacent[3]; // Adjacent faces
        int edge_adj[3];      // Which edge index on the adjacent face connects back to this one
        bool obsolete;        // Marked true when the face is removed during expansion
        unsigned int pass;    // Pass counter used during horizon traversal to detect already-visited faces
    };

    // Tracks the "horizon" ring of new faces being stitched around the new support point
    // during polytope expansion. The horizon is a closed loop of faces in CCW order.
    struct EPAHorizon
    {
        EPAFace *cf;  // The most recently added horizon face (tail)
        EPAFace *ff;  // The first horizon face added (head, used to close the ring)
        int nf;       // Number of faces added to the horizon so far
        EPAHorizon() : cf(nullptr), ff(nullptr), nf(0) {}
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

        // Configuration
        unsigned int max_faces     = 256;
        unsigned int max_vertices  = 128;
        unsigned int max_iterations = 512;
        m3d::scalar  tolerance     = 1e-10;

        // Results
        m3d::vec3   normal;
        m3d::scalar depth;
        m3d::vec3   contact_point;

        // Internal state
        EPAFace       **faces;
        SimplexVertex **vertices;
        unsigned int    num_faces;
        unsigned int    num_vertices;

        EPA();
        ~EPA();

        Status evaluate(const GJK &gjk_solver, const MinkowskiDiff &shape);

        void      initialize();
        EPAFace  *find_closest_face();
        void      bind_faces(EPAFace *f0, int e0, EPAFace *f1, int e1);

        // Recursively traverses the adjacency graph from face f (through edge e) to find
        // the horizon silhouette. Faces visible from w are marked obsolete; for each
        // horizon edge a new face is created and appended to the horizon ring.
        bool expand(unsigned int pass, SimplexVertex *w,
                    EPAFace *f, int e, EPAHorizon &horizon);

    private:
        // Allocates and initialises one face from the pre-allocated pool.
        // Returns nullptr if the face pool is exhausted or the face is degenerate.
        EPAFace *new_face(SimplexVertex *a, SimplexVertex *b, SimplexVertex *c);
    };

} // namespace rbc