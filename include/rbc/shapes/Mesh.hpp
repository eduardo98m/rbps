#pragma once
#include <cstdint>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    // ── MeshData ──────────────────────────────────────────────────────────────
    // Triangle mesh in LOCAL space.  The shape transform m3d::tf moves it into world.
    // Precomputed per-face normals are required for SAT-based collision.
    //
    // Memory layout:
    //   vertices[i]               — local-space vertex positions
    //   indices[f*3 + {0,1,2}]    — vertex indices of face f
    //   face_normals[f]           — unit normal of face f (consistent winding)
    struct MeshData
    {
        const m3d::vec3 *vertices;
        const uint32_t *indices;
        const m3d::vec3 *face_normals; // one per face, precomputed
        uint32_t vert_count;
        uint32_t face_count;
        AABB local_aabb; // precomputed local-space AABB
    };

    // ── Mesh shape ────────────────────────────────────────────────────────────
    // Non-owning pointer. MeshData must outlive the Mesh.
    struct Mesh
    {
        const MeshData *data;

        Mesh() : data(nullptr) {}
        explicit Mesh(const MeshData *data) : data(data) {}

        inline bool operator==(const Mesh &o) const { return data == o.data; }
        inline bool operator!=(const Mesh &o) const { return data != o.data; }
    };

    // ── Construction helper ───────────────────────────────────────────────────
    // Computes face normals and local AABB from raw arrays.
    // Assumes counter-clockwise winding (outward normals).
    // Caller owns the returned pointer; call mesh_data_destroy() to free.
    inline MeshData *mesh_data_create(const m3d::vec3 *vertices, uint32_t vert_count,
                                      const uint32_t *indices, uint32_t face_count)
    {
        auto *md = new MeshData;
        auto *norms = new m3d::vec3[face_count];

        md->vertices = vertices;
        md->indices = indices;
        md->face_normals = norms;
        md->vert_count = vert_count;
        md->face_count = face_count;

        // Compute face normals + local AABB
        m3d::vec3 mn(1e30f, 1e30f, 1e30f);
        m3d::vec3 mx(-1e30f, -1e30f, -1e30f);

        for (uint32_t f = 0; f < face_count; ++f)
        {
            const m3d::vec3 &A = vertices[indices[f * 3 + 0]];
            const m3d::vec3 &B = vertices[indices[f * 3 + 1]];
            const m3d::vec3 &C = vertices[indices[f * 3 + 2]];
            norms[f] = m3d::normalize(m3d::cross(B - A, C - A));
        }

        for (uint32_t v = 0; v < vert_count; ++v)
        {
            mn = m3d::vec3(m3d::min(mn.x, vertices[v].x),
                           m3d::min(mn.y, vertices[v].y),
                           m3d::min(mn.z, vertices[v].z));
            mx = m3d::vec3(m3d::max(mx.x, vertices[v].x),
                           m3d::max(mx.y, vertices[v].y),
                           m3d::max(mx.z, vertices[v].z));
        }
        md->local_aabb = {mn, mx};
        return md;
    }

    inline void mesh_data_destroy(MeshData *md)
    {
        delete[] md->face_normals;
        delete md;
    }

    // ── Support (convex-hull support — only valid for CONVEX meshes) ──────────
    // For concave meshes, use the per-triangle analytic collision in MeshCollision.hpp.
    inline m3d::vec3 support(const Mesh &m, const m3d::vec3 &dir)
    {
        if (!m.data || m.data->vert_count == 0)
            return m3d::vec3{};
        m3d::vec3 best = m.data->vertices[0];
        m3d::scalar best_d = m3d::dot(best, dir);
        for (uint32_t i = 1; i < m.data->vert_count; ++i)
        {
            const m3d::scalar d = m3d::dot(m.data->vertices[i], dir);
            if (d > best_d)
            {
                best_d = d;
                best = m.data->vertices[i];
            }
        }
        return best;
    }

    // ── AABB from precomputed local bounds + transform ────────────────────────
    inline AABB compute_aabb(const Mesh &m, const m3d::tf &tf)
    {
        if (!m.data)
            return AABB{};
        // Rotate local AABB into world space (conservative, same method as Box)
        const AABB &local = m.data->local_aabb;
        const m3d::vec3 centre = (local.min + local.max) * 0.5;
        const m3d::vec3 half = (local.max - local.min) * 0.5;

        const m3d::mat3 R = m3d::mat3_cast(tf.rot);
        const m3d::vec3 world_centre = tf.pos + tf.rotate_vector(centre);
        const m3d::vec3 extent(
            m3d::abs(R[0][0]) * half.x + m3d::abs(R[1][0]) * half.y + m3d::abs(R[2][0]) * half.z,
            m3d::abs(R[0][1]) * half.x + m3d::abs(R[1][1]) * half.y + m3d::abs(R[2][1]) * half.z,
            m3d::abs(R[0][2]) * half.x + m3d::abs(R[1][2]) * half.y + m3d::abs(R[2][2]) * half.z);
        return {world_centre - extent, world_centre + extent};
    }

    // Marker for the dispatcher: Mesh is treated as non-convex (triangle soup).
    // Convex meshes work too, but the analytic specialisation in
    // MeshCollision.hpp is the supported path.
    constexpr bool is_gjk_convex(const Mesh *) { return false; }
    inline m3d::scalar representative_radius(const Mesh &) { return 0.0; }
    inline int face_corners(const Mesh &, const m3d::tf &,
                            const m3d::vec3 &, m3d::vec3[4]) { return 0; }
} // namespace rbc