#pragma once
#include <math3d/math3d.hpp>

namespace cdbg
{

    enum ShapeKind : int
    {
        Kind_Sphere = 0,
        Kind_Box,
        Kind_Capsule,
        Kind_Cylinder,
        Kind_Cone,
        Kind_Ellipsoid,
        Kind_ConvexHull,
        Kind_Count
    };

    enum HullPreset : int
    {
        Hull_Tetrahedron = 0,
        Hull_Octahedron,
        Hull_TriPrism,
        Hull_HexPrism,
        Hull_Count
    };

    struct ShapeParams
    {
        int kind = Kind_Box;

        m3d::vec3   box_half_ext{0.5, 0.5, 0.5};
        m3d::scalar sphere_radius   = 0.5;
        m3d::scalar capsule_radius  = 0.4;
        m3d::scalar capsule_half_h  = 0.5;
        m3d::scalar cyl_radius      = 0.5;
        m3d::scalar cyl_half_h      = 0.6;
        m3d::scalar cone_radius     = 0.5;
        m3d::scalar cone_half_h     = 0.5;
        m3d::vec3   ellipsoid_axes{0.5, 0.7, 0.5};

        int hull_preset = Hull_HexPrism;
    };

    struct PoseControl
    {
        m3d::vec3 position{0.0, 0.0, 0.0};
        m3d::vec3 rpy_deg{0.0, 0.0, 0.0};
    };

    struct VizFlags
    {
        bool shape_a           = true;
        bool shape_b           = true;
        bool gjk_simplex       = false;
        bool epa_polytope      = false;
        bool epa_normal        = true;
        bool epa_contact_point = true;
        bool ref_face          = true;
        bool inc_face          = true;
        bool clipped_polygon   = true;
        bool manifold_points   = true;
        bool md_space_axes     = false;
        bool world_grid        = true;

        m3d::vec3 md_space_offset{6.0, 0.0, 0.0};
    };

    struct DebuggerState
    {
        ShapeParams params_a;
        ShapeParams params_b;
        PoseControl pose_a;
        PoseControl pose_b;
        VizFlags    viz;
        bool        live_recompute = true;
        bool        recompute_now  = false;
    };

    inline m3d::tf pose_to_tf(const PoseControl &p)
    {
        const m3d::scalar deg2rad = static_cast<m3d::scalar>(0.017453292519943295);
        return m3d::tf{p.position,
                       m3d::quat::from_rpy(p.rpy_deg.x * deg2rad,
                                           p.rpy_deg.y * deg2rad,
                                           p.rpy_deg.z * deg2rad)};
    }

} // namespace cdbg
