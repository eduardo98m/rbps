#pragma once
// Internal-linkage helpers shared by Line.cpp / Triangle.cpp /
// Tetrahedron.cpp. Not part of the public GJK surface; do not
// include outside src/rbc/gjk/simplex/.

#include <math3d/math3d.hpp>

namespace rbc { namespace simplex_detail {

    // (a × b) × c — the standard "Voronoi-region direction" used by
    // Casey-style GJK to point from a feature toward the origin while
    // staying in the feature's plane.
    inline m3d::vec3 triple_cross(const m3d::vec3 &a,
                                  const m3d::vec3 &b,
                                  const m3d::vec3 &c)
    {
        return m3d::cross(m3d::cross(a, b), c);
    }

}} // namespace rbc::simplex_detail
