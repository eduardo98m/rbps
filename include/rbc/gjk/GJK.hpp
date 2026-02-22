#pragma once

#include "MinkowskiDiff.hpp"
#include <vector>

namespace rbc {
    
    struct SimplexVertex {
        m3d::vec3 w0, w1; // Original points on shape A and B
        m3d::vec3 w;      // Minkowski difference point (w0 - w1)
    };

    struct Simplex {
        SimplexVertex* vertex[4];
        int rank = 0;
    };

    struct GJK {
        enum Status { Valid, Inside, Failed } status;

        // Configuration
        unsigned int max_iterations = 128;
        m3d::scalar tolerance = 1e-8;

        // Results / State
        const MinkowskiDiff* current_shape;
        m3d::vec3 ray;
        m3d::scalar distance;
        
        Simplex simplices[2];
        SimplexVertex store_v[4];
        SimplexVertex* free_v[4];
        int nfree;
        int current;
        Simplex* active_simplex;

        GJK();

        // Evaluates the collision between the shapes defined in MinkowskiDiff
        Status evaluate(const MinkowskiDiff& shape, const m3d::vec3& initial_guess);
        
        // Simplex management
        void initialize();
        void append_vertex(Simplex& s, const m3d::vec3& v);
        void remove_vertex(Simplex& s);
        
        // Mathematical projections to determine Voronoi region
        bool enclose_origin();
        bool project_line_origin(const Simplex& curr, Simplex& next);
        bool project_triangle_origin(const Simplex& curr, Simplex& next);
        bool project_tetrahedron_origin(const Simplex& curr, Simplex& next);
    };

} // namespace rbc