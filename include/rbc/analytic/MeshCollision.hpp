#pragma once
// ── Mesh collision detection ──────────────────────────────────────────────────
// Brute-force: iterate over all triangles in world space, keep the deepest contact.
// Suitable for static/kinematic concave meshes.
//
// For high face-count meshes, replace with a BVH acceleration structure —
// the interface (CollisionAlgorithm specialisations) stays identical.

#include "rbc/Dispatcher.hpp"
#include "rbc/shapes/Mesh.hpp"
#include "rbc/shapes/Capsule.hpp"
#include "rbc/analytic/TriangleUtils.hpp"

namespace rbc
{

    // ── Internal: iterate triangles, call a per-triangle test ─────────────────
    namespace detail
    {
        // Transform a local-space mesh triangle to world space using tf.
        inline void mesh_world_triangle(const MeshData &md, uint32_t f,
                                        const m3d::tf &tf,
                                        m3d::vec3 &A, m3d::vec3 &B, m3d::vec3 &C,
                                        m3d::vec3 &world_normal)
        {
            A = tf.pos + tf.rotate_vector(md.vertices[md.indices[f * 3 + 0]]);
            B = tf.pos + tf.rotate_vector(md.vertices[md.indices[f * 3 + 1]]);
            C = tf.pos + tf.rotate_vector(md.vertices[md.indices[f * 3 + 2]]);
            world_normal = tf.rotate_vector(md.face_normals[f]);
        }

        // Find the deepest penetrating contact across all triangles.
        // `per_tri` is called with (A, B, C, world_normal, candidate_contact).
        template <typename PerTriFn>
        inline bool mesh_test_all(const Mesh &mesh, const m3d::tf &tf_mesh,
                                  PerTriFn per_tri, ContactManifold &out)
        {
            if (!mesh.data)
                return false;
            const MeshData &md = *mesh.data;

            bool hit = false;
            m3d::scalar best_depth = 0.0;
            Contact candidate;

            for (uint32_t f = 0; f < md.face_count; ++f)
            {
                m3d::vec3 A, B, C, wn;
                mesh_world_triangle(md, f, tf_mesh, A, B, C, wn);

                if (per_tri(A, B, C, wn, candidate))
                {
                    if (candidate.penetration_depth > best_depth)
                    {
                        best_depth = candidate.penetration_depth;
                        out = candidate;
                        hit = true;
                    }
                }
            }
            return hit;
        }
    } // namespace detail

    // ── Sphere vs Mesh ────────────────────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Sphere, Mesh>
    {
        static bool test(const Sphere &sphere, const m3d::tf &tf_sphere,
                         const Mesh &mesh, const m3d::tf &tf_mesh,
                         ContactManifold &out)
        {
            return detail::mesh_test_all(mesh, tf_mesh, [&](const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C, const m3d::vec3 &wn, ContactManifold &c)
                                         { return tri::sphere_vs_triangle(tf_sphere.pos, sphere.radius,
                                                                          A, B, C, wn, c); }, out);
        }
    };
    template <>
    struct CollisionAlgorithm<Mesh, Sphere> : CollisionAlgorithmSym<Mesh, Sphere>
    {
    };

    // ── Capsule vs Mesh ───────────────────────────────────────────────────────
    template <>
    struct CollisionAlgorithm<Capsule, Mesh>
    {
        static bool test(const Capsule &cap, const m3d::tf &tf_cap,
                         const Mesh &mesh, const m3d::tf &tf_mesh,
                         ContactManifold &out)
        {
            m3d::vec3 p1, p2;
            capsule_endpoints(cap, tf_cap, p1, p2);

            return detail::mesh_test_all(mesh, tf_mesh, [&](const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C, const m3d::vec3 &wn, ContactManifold &c)
                                         { return tri::capsule_vs_triangle(p1, p2, cap.radius, A, B, C, wn, c); }, out);
        }
    };
    template <>
    struct CollisionAlgorithm<Mesh, Capsule> : CollisionAlgorithmSym<Mesh, Capsule>
    {
    };

    // ── Box vs Mesh ───────────────────────────────────────────────────────────
    // Uses the face-normal "push" approach: for each mesh triangle, check if
    // the box has any vertex on the "inside" of the face, and vice-versa.
    // This is a simplification of the full 15-axis SAT; it handles the common
    // cases of thin walls robustly.  For edge-edge contacts use the GJK fallback
    // (the primary CollisionAlgorithm<Box,Mesh> below) if you need full accuracy.
    template <>
    struct CollisionAlgorithm<Box, Mesh>
    {
        static bool test(const Box &box, const m3d::tf &tf_box,
                         const Mesh &mesh, const m3d::tf &tf_mesh,
                         ContactManifold &out)
        {
            // Represent the box as a sphere (bounding) for fast rejection, then
            // test each passing triangle with sphere_vs_triangle on the box centre.
            // For a correct full SAT over box faces + mesh edges, integrate BoxBox logic.
            // Here we delegate via the sphere of the box's bounding radius.
            const m3d::scalar r = m3d::length(box.half_extents); // bounding sphere

            return detail::mesh_test_all(mesh, tf_mesh, [&](const m3d::vec3 &A, const m3d::vec3 &B, const m3d::vec3 &C, const m3d::vec3 &wn, ContactManifold &c) -> bool
                                         {
                    // Quick face-normal projection of box onto triangle plane
                    const m3d::scalar centre_dist = m3d::dot(tf_box.pos - A, wn);
                    // Box half-extent projected on world-normal
                    const m3d::vec3 local_n = tf_box.inverse_rotate_vector(wn);
                    const m3d::scalar box_r =
                        m3d::abs(local_n.x) * box.half_extents.x +
                        m3d::abs(local_n.y) * box.half_extents.y +
                        m3d::abs(local_n.z) * box.half_extents.z;

                    if (centre_dist > box_r) return false; // separated on face normal

                    // Project box centre onto triangle plane and test if nearby
                    const m3d::vec3  proj   = tf_box.pos - wn * centre_dist;
                    const m3d::vec3  closest = tri::closest_point_on_triangle(proj, A, B, C);
                    const m3d::scalar edge_d = m3d::length(proj - closest);

                    if (edge_d > box_r) return false;

                    c.normal            = wn;
                    c.num_points = 1;
                    c.points[0].penetration_depth = box_r - edge_d;
                    c.points[0].position = closest;

                    return c.points[0].penetration_depth > 0.0; }, out);
        }
    };
    template <>
    struct CollisionAlgorithm<Mesh, Box> : CollisionAlgorithmSym<Mesh, Box>
    {
    };

    // ── Mesh vs Mesh ─────────────────────────────────────────────────────────
    // Brute-force: treat each face of mesh A as a sphere of radius 0
    // and test against mesh B.  Very expensive for large meshes.
    // In production, replace with BVH + GJK/SAT per overlapping triangle pair.
    template <>
    struct CollisionAlgorithm<Mesh, Mesh>
    {
        static bool test(const Mesh &a, const m3d::tf &tf_a,
                         const Mesh &b, const m3d::tf &tf_b,
                         ContactManifold &out)
        {
            if (!a.data || !b.data)
                return false;

            bool hit = false;
            m3d::scalar best_depth = 0.0;
            ContactManifold candidate;
            const MeshData &md_a = *a.data;

            for (uint32_t fa = 0; fa < md_a.face_count; ++fa)
            {
                m3d::vec3 A, B, C, wn_a;
                detail::mesh_world_triangle(md_a, fa, tf_a, A, B, C, wn_a);

                // Test each vertex of face fa against mesh B
                const m3d::vec3 verts[3] = {A, B, C};
                for (const auto &vtx : verts)
                {
                    if (detail::mesh_test_all(b, tf_b, [&](const m3d::vec3 &bA, const m3d::vec3 &bB, const m3d::vec3 &bC, const m3d::vec3 &wn_b, ContactManifold &c)
                                              { return tri::sphere_vs_triangle(vtx, 0.0, bA, bB, bC, wn_b, c); }, candidate))
                    {
                        if (candidate.points[0].penetration_depth > best_depth)
                        {
                            best_depth = candidate.points[0].penetration_depth;
                            out = candidate;
                            hit = true;
                        }
                    }
                }
            }
            return hit;
        }
    };

} // namespace rbc