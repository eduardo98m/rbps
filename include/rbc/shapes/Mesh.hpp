#pragma once
#include <cstdint>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

/**
 * @file Mesh.hpp
 * @brief Triangle-mesh collision shape (general — concave allowed).
 * @ingroup rbc
 */

namespace rbc
{
    /**
     * @brief Backing storage for a triangle mesh in local space.
     *
     * Per-face normals are precomputed and required by the SAT-based
     * collision routines in [analytic/MeshCollision.hpp](analytic/MeshCollision.hpp).
     *
     * Memory layout:
     * - `vertices[i]`              — local-space vertex positions.
     * - `indices[f*3 + {0,1,2}]`   — vertex indices of face `f`.
     * - `face_normals[f]`          — unit normal of face `f` (consistent winding).
     *
     * @ingroup rbc
     */
    struct MeshData
    {
        const m3d::vec3 *vertices;     ///< Vertex positions in local space.
        const uint32_t *indices;       ///< Triangle indices, 3 per face.
        const m3d::vec3 *face_normals; ///< Unit normal per face (precomputed).
        uint32_t vert_count;           ///< Number of entries in `vertices`.
        uint32_t face_count;           ///< Number of triangles.
        AABB local_aabb;               ///< Precomputed local-space AABB.
    };

    /**
     * @brief Mesh shape — wraps a non-owning pointer to `MeshData`.
     *
     * `MeshData` must outlive every `Mesh` that references it.
     *
     * @ingroup rbc
     */
    struct Mesh
    {
        const MeshData *data; ///< Non-owning pointer.

        /** @brief Default-construct with no data attached. */
        Mesh() : data(nullptr) {}
        /** @brief Wrap an existing `MeshData` (caller retains ownership). */
        explicit Mesh(const MeshData *data) : data(data) {}

        /** @brief Equality on the data pointer. */
        inline bool operator==(const Mesh &o) const { return data == o.data; }
        /** @brief Inequality on the data pointer. */
        inline bool operator!=(const Mesh &o) const { return data != o.data; }
    };

    /**
     * @brief Allocate a `MeshData` and precompute face normals + local AABB.
     *
     * Assumes counter-clockwise triangle winding (outward normals). Caller
     * owns the returned pointer; call `mesh_data_destroy` to free it. The
     * `vertices` and `indices` arrays are referenced, not copied.
     *
     * @ingroup rbc
     */
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

    /**
     * @brief Free a `MeshData` returned by `mesh_data_create`.
     *
     * Also frees the precomputed face-normals array.
     *
     * @ingroup rbc
     */
    inline void mesh_data_destroy(MeshData *md)
    {
        delete[] md->face_normals;
        delete md;
    }

    /**
     * @brief Convex-hull support: vertex of `m` farthest along `dir`.
     *
     * @warning Only valid for **convex** meshes. Concave meshes must go
     *          through the per-triangle analytic path in
     *          [analytic/MeshCollision.hpp](analytic/MeshCollision.hpp).
     *
     * @ingroup rbc
     */
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

    /**
     * @brief Conservative world AABB from the precomputed local AABB.
     *
     * Same OBB-projection technique as `Box::compute_aabb`. The result is
     * a conservative wrapper — for tighter bounds you'd need to project
     * every world vertex, which is no longer O(1).
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const Mesh &m, const m3d::tf &tf)
    {
        if (!m.data)
            return AABB{};
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

    /**
     * @brief Tag-dispatched marker: Mesh is treated as non-convex (false).
     *
     * Convex meshes work too, but the supported path is the analytic
     * specialisation in [analytic/MeshCollision.hpp](analytic/MeshCollision.hpp).
     *
     * @ingroup rbc
     */
    constexpr bool is_gjk_convex(const Mesh *) { return false; }
    /** @brief Returns 0 — mesh has no single representative size. @ingroup rbc */
    inline m3d::scalar representative_radius(const Mesh &) { return 0.0; }
    /** @brief Stub — mesh pairs go through analytic algorithms. @ingroup rbc */
    inline int face_corners(const Mesh &, const m3d::tf &,
                            const m3d::vec3 &, m3d::vec3[4]) { return 0; }
} // namespace rbc
