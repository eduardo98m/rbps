#pragma once
#include "State.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include <cmath>

namespace cdbg
{

    inline const char *kind_label(int kind)
    {
        switch (kind)
        {
        case Kind_Sphere:     return "Sphere";
        case Kind_Box:        return "Box";
        case Kind_Capsule:    return "Capsule";
        case Kind_Cylinder:   return "Cylinder";
        case Kind_Cone:       return "Cone";
        case Kind_Ellipsoid:  return "Ellipsoid";
        case Kind_ConvexHull: return "ConvexHull";
        }
        return "?";
    }

    inline const char *hull_label(int preset)
    {
        switch (preset)
        {
        case Hull_Tetrahedron: return "Tetrahedron";
        case Hull_Octahedron:  return "Octahedron";
        case Hull_TriPrism:    return "Tri-prism";
        case Hull_HexPrism:    return "Hex-prism";
        }
        return "?";
    }

    // ── Hull preset providers (static lazy-init, lifetime = process) ──────

    inline rbc::ConvexHullData *get_tetrahedron_data()
    {
        static const m3d::vec3 verts[4] = {
            m3d::vec3( 0.6,  0.6,  0.6),
            m3d::vec3(-0.6, -0.6,  0.6),
            m3d::vec3(-0.6,  0.6, -0.6),
            m3d::vec3( 0.6, -0.6, -0.6),
        };
        static const uint32_t faces[4 * 3] = {
            0, 1, 2,
            0, 3, 1,
            0, 2, 3,
            1, 3, 2,
        };
        static rbc::ConvexHullData *data =
            rbc::convex_hull_data_create(verts, 4, faces, 4);
        return data;
    }

    inline rbc::ConvexHullData *get_octahedron_data()
    {
        static const m3d::vec3 verts[6] = {
            m3d::vec3( 1.0,  0.0,  0.0), m3d::vec3(-1.0,  0.0,  0.0),
            m3d::vec3( 0.0,  1.0,  0.0), m3d::vec3( 0.0, -1.0,  0.0),
            m3d::vec3( 0.0,  0.0,  1.0), m3d::vec3( 0.0,  0.0, -1.0),
        };
        static const uint32_t faces[8 * 3] = {
            0, 2, 4,  0, 4, 3,  0, 3, 5,  0, 5, 2,
            1, 4, 2,  1, 3, 4,  1, 5, 3,  1, 2, 5,
        };
        static rbc::ConvexHullData *data =
            rbc::convex_hull_data_create(verts, 6, faces, 8);
        return data;
    }

    inline rbc::ConvexHullData *get_tri_prism_data()
    {
        static const m3d::vec3 verts[6] = {
            m3d::vec3( 0.0,    1.0,  1.0),
            m3d::vec3(-0.866, -0.5,  1.0),
            m3d::vec3( 0.866, -0.5,  1.0),
            m3d::vec3( 0.0,    1.0, -1.0),
            m3d::vec3(-0.866, -0.5, -1.0),
            m3d::vec3( 0.866, -0.5, -1.0),
        };
        // 8 triangles: 2 caps (top 0,1,2 ; bottom 3,5,4) + 3 quads (each 2 tris)
        static const uint32_t faces[8 * 3] = {
            0, 1, 2,
            3, 5, 4,
            0, 3, 4,  0, 4, 1,
            1, 4, 5,  1, 5, 2,
            2, 5, 3,  2, 3, 0,
        };
        static rbc::ConvexHullData *data =
            rbc::convex_hull_data_create(verts, 6, faces, 8);
        return data;
    }

    inline rbc::ConvexHullData *get_hex_prism_data()
    {
        static m3d::vec3 verts[12];
        static uint32_t  faces[20 * 3];
        static bool initialized = false;

        if (!initialized)
        {
            for (int i = 0; i < 6; ++i)
            {
                const float angle = i * (6.283185307f / 6.0f);
                const float x = std::cos(angle) * 0.8f;
                const float z = std::sin(angle) * 0.8f;
                verts[i]     = m3d::vec3(x,  0.5f, z);
                verts[i + 6] = m3d::vec3(x, -0.5f, z);
            }

            int f = 0;
            // Top cap (tri-fan around vertex 0)
            for (int i = 1; i < 5; ++i)
            {
                faces[f++] = 0; faces[f++] = i; faces[f++] = i + 1;
            }
            // Bottom cap (reversed winding for outward normal)
            for (int i = 1; i < 5; ++i)
            {
                faces[f++] = 6; faces[f++] = i + 7; faces[f++] = i + 6;
            }
            // Sides: 6 quads = 12 triangles
            for (int i = 0; i < 6; ++i)
            {
                const int next = (i + 1) % 6;
                faces[f++] = i;    faces[f++] = i + 6;    faces[f++] = next;
                faces[f++] = next; faces[f++] = i + 6;    faces[f++] = next + 6;
            }
            initialized = true;
        }

        static rbc::ConvexHullData *data =
            rbc::convex_hull_data_create(verts, 12, faces, 20);
        return data;
    }

    inline rbc::ConvexHullData *get_hull_data(int preset)
    {
        switch (preset)
        {
        case Hull_Tetrahedron: return get_tetrahedron_data();
        case Hull_Octahedron:  return get_octahedron_data();
        case Hull_TriPrism:    return get_tri_prism_data();
        case Hull_HexPrism:    return get_hex_prism_data();
        }
        return get_tetrahedron_data();
    }

    inline rbc::Shape make_shape(const ShapeParams &p)
    {
        switch (p.kind)
        {
        case Kind_Sphere:
            return rbc::Shape(rbc::Sphere{p.sphere_radius});
        case Kind_Box:
            return rbc::Shape(rbc::Box{p.box_half_ext});
        case Kind_Capsule:
            return rbc::Shape(rbc::Capsule{p.capsule_half_h, p.capsule_radius});
        case Kind_Cylinder:
            return rbc::Shape(rbc::Cylinder(p.cyl_half_h, p.cyl_radius));
        case Kind_Cone:
            return rbc::Shape(rbc::Cone{p.cone_half_h, p.cone_radius});
        case Kind_Ellipsoid:
            return rbc::Shape(rbc::Ellipsoid{p.ellipsoid_axes});
        case Kind_ConvexHull:
            return rbc::Shape(rbc::ConvexHull(get_hull_data(p.hull_preset)));
        }
        return rbc::Shape(rbc::Box{p.box_half_ext});
    }

} // namespace cdbg
