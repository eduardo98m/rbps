#pragma once
#include <math3d/math3d.hpp>

// Polygon clipping primitives used by the contact-manifold generator.
// Geometric only — knows nothing about ConvexHull adjacency.

namespace rbc { namespace clipping {

    struct Plane
    {
        m3d::vec3 normal; ///< Plane normal (need not be unit length).
        m3d::vec3 point;  ///< Any point on the plane.
    };

    /**
     * @brief Clip a polygon against a sequence of planes (Sutherland–Hodgman).
     *
     * @param in            Input polygon vertices, in CCW order around the
     *                      polygon's outward normal.
     * @param n_in          Number of input vertices.
     * @param planes        Clip planes. Points with `dot(p - plane.point,
     *                      plane.normal) < 0` are considered outside.
     * @param n_planes      Number of clip planes.
     * @param out           Output buffer (caller-owned).
     * @param out_capacity  Maximum vertices the caller's buffer holds.
     * @param remove_only   When true, vertices outside a plane are dropped
     *                      (no crossing point is computed). When false,
     *                      crossings are produced — the standard
     *                      Sutherland–Hodgman behaviour.
     * @return The number of vertices written to `out`. May exceed
     *         `n_in`; clamps to `out_capacity` (truncates excess).
     */
    int sutherland_hodgman(const m3d::vec3 *in, int n_in,
                           const Plane *planes, int n_planes,
                           m3d::vec3 *out, int out_capacity,
                           bool remove_only);

    /**
     * @brief Closest points between two skew (non-parallel) lines in 3D.
     *
     * Each line is given by a point + direction. Output `l1` lies on
     * line 1, `l2` on line 2. Returns false when the lines are parallel
     * within numerical tolerance.
     */
    bool skew_line_closest_points(const m3d::vec3 &p1, const m3d::vec3 &d1,
                                  const m3d::vec3 &p2, const m3d::vec3 &d2,
                                  m3d::vec3 &l1, m3d::vec3 &l2);

}} // namespace rbc::clipping
