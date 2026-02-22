#pragma once

#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"

namespace rbc
{

    // Represents a triangular face in the EPA polytope
    struct EPAFace
    {
        m3d::vec3 n;          // Face normal
        m3d::scalar d;        // Distance from origin to the face
        SimplexVertex *v[3];  // The three vertices making up the face
        EPAFace *adjacent[3]; // Adjacent faces
        int edge_adj[3];      // Which edge on the adjacent face connects to this one
        bool obsolete;        // Flag to mark if the face should be removed
        int pass;             // Iteration pass ID for horizon finding
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
        unsigned int max_faces = 128;
        unsigned int max_vertices = 64;
        unsigned int max_iterations = 255;
        m3d::scalar tolerance = 1e-4;

        // Results
        m3d::vec3 normal;
        m3d::scalar depth;
        m3d::vec3 contact_point; // Approximated contact point

        // Internal State (Public per your request)
        EPAFace **faces;
        SimplexVertex **vertices;
        unsigned int num_faces;
        unsigned int num_vertices;

        EPA();
        ~EPA();

        // Main evaluation loop
        Status evaluate(const GJK &gjk_solver, const MinkowskiDiff &shape);

        // Helper methods (left public)
        void initialize();
        EPAFace *find_closest_face();
        bool expand_polytope(EPAFace *face, SimplexVertex *w);
        void bind_faces(EPAFace *f0, int e0, EPAFace *f1, int e1);
    };

} // namespace rbc