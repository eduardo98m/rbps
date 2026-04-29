#pragma once
#include <cstdint>
#include <cstdio>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"
#include "rbc/shapes/FaceHelpers.hpp"

/**
 * @file ConvexHull.hpp
 * @brief Convex polytope collision shape.
 * @ingroup rbc
 *
 * `ConvexHull` is the first-class collider for arbitrary convex
 * polytopes. Unlike `Mesh` (which carries `is_gjk_convex == false` and
 * always routes through the per-triangle analytic path), `ConvexHull`
 * advertises convexity to the dispatcher and rides the existing GJK +
 * EPA + manifold pipeline used by every other convex shape.
 *
 * @par Storage
 * Non-owning, mirroring `Mesh` and `Heightmap`: a `ConvexHull` is a
 * pointer to a `ConvexHullData`, which itself holds non-owning pointers
 * to a caller-managed vertex array (and optionally face indices). Use
 * `convex_hull_data_create` to allocate a `ConvexHullData` and
 * pre-compute the per-face normals + local AABB; release it with
 * `convex_hull_data_destroy`.
 *
 * @par Building hulls
 * Out of scope here. Users either pre-hull externally (qhull, libccd,
 * Bullet) or pass procedurally-generated vertices that they know are
 * convex. There is no convexity validator at this layer — the caller
 * is responsible.
 *
 * @par Faces are optional
 * Vertex-only hulls work; they fall back to single-vertex contact
 * features and an AABB-of-hull inertia approximation. Pass face data
 * (CCW-wound triangles) for true face/face manifolds and an exact
 * tetrahedral inertia tensor.
 */

namespace rbc
{
    /**
     * @brief Backing storage for a convex polytope in local space.
     *
     * Non-owning over `vertices` and `face_indices` (caller keeps those
     * arrays alive); owning over `face_normals` (allocated by
     * `convex_hull_data_create` when face data is supplied).
     *
     * Memory layout:
     * - `vertices[i]`              — local-space vertex positions.
     * - `face_indices[f*3 + {0,1,2}]` — vertex indices of triangle face `f` (CCW).
     * - `face_normals[f]`          — unit outward normal of face `f`, precomputed.
     *
     * Set `face_indices == nullptr` and `face_count == 0` for a
     * vertex-only hull. `face_offsets` is reserved for future
     * polygon-face support and currently unused (only triangulated
     * faces are consumed).
     *
     * @ingroup rbc
     */
    struct ConvexHullData
    {
        const m3d::vec3 *vertices;     ///< Vertex positions in local space (non-owning).
        const uint32_t  *face_indices; ///< Triangle indices, 3 per face (non-owning); may be `nullptr`.
        const uint32_t  *face_offsets; ///< Reserved for non-triangulated faces; currently unused.
        const m3d::vec3 *face_normals; ///< Unit outward normal per face (owned by helper when faces given).
        uint32_t         vert_count;   ///< Number of entries in `vertices`.
        uint32_t         face_count;   ///< Number of triangles; 0 when `face_indices == nullptr`.
        AABB             local_aabb;   ///< Precomputed local-space AABB.
    };

    /**
     * @brief Convex hull shape — wraps a non-owning pointer to `ConvexHullData`.
     *
     * `ConvexHullData` must outlive every `ConvexHull` that references it.
     *
     * @ingroup rbc
     */
    struct ConvexHull
    {
        const ConvexHullData *data; ///< Non-owning pointer.

        /** @brief Default-construct with no data attached. */
        ConvexHull() : data(nullptr) {}
        /** @brief Wrap an existing `ConvexHullData` (caller retains ownership). */
        explicit ConvexHull(const ConvexHullData *data) : data(data) {}

        /** @brief Equality on the data pointer. */
        inline bool operator==(const ConvexHull &o) const { return data == o.data; }
        /** @brief Inequality on the data pointer. */
        inline bool operator!=(const ConvexHull &o) const { return data != o.data; }
    };

    /**
     * @brief Allocate a `ConvexHullData` and precompute face normals + local AABB.
     *
     * The caller-supplied `vertices` and `face_indices` arrays are
     * referenced, not copied — they must outlive the returned
     * `ConvexHullData`. The `face_normals` buffer is allocated and
     * populated by this function; release it with
     * `convex_hull_data_destroy`.
     *
     * Pass `face_indices == nullptr` and `face_count == 0` for a
     * vertex-only hull; in that case no normals are allocated and
     * `face_normals` is left null.
     *
     * @warning Assumes counter-clockwise triangle winding (outward
     *          normals). The caller is responsible for ensuring the
     *          input vertices form a convex polytope; there is no
     *          validation at this layer.
     *
     * @param vertices     Local-space vertex positions (referenced, not copied).
     * @param vert_count   Number of entries in `vertices`.
     * @param face_indices Triangle indices, 3 per face (CCW); may be `nullptr`.
     * @param face_count   Number of triangles; pass 0 when `face_indices == nullptr`.
     * @return Heap-allocated `ConvexHullData` owned by the caller.
     *
     * @ingroup rbc
     */
    inline ConvexHullData *convex_hull_data_create(const m3d::vec3 *vertices, uint32_t vert_count,
                                                   const uint32_t *face_indices = nullptr,
                                                   uint32_t face_count = 0)
    {
        auto *hd = new ConvexHullData;
        hd->vertices     = vertices;
        hd->face_indices = face_indices;
        hd->face_offsets = nullptr;
        hd->face_normals = nullptr;
        hd->vert_count   = vert_count;
        hd->face_count   = (face_indices ? face_count : 0u);

        if (hd->face_count > 0)
        {
            auto *norms = new m3d::vec3[hd->face_count];
            for (uint32_t f = 0; f < hd->face_count; ++f)
            {
                const m3d::vec3 &A = vertices[face_indices[f * 3 + 0]];
                const m3d::vec3 &B = vertices[face_indices[f * 3 + 1]];
                const m3d::vec3 &C = vertices[face_indices[f * 3 + 2]];
                norms[f] = m3d::normalize(m3d::cross(B - A, C - A));
            }
            hd->face_normals = norms;
        }

        m3d::vec3 mn(1e30f, 1e30f, 1e30f);
        m3d::vec3 mx(-1e30f, -1e30f, -1e30f);
        for (uint32_t v = 0; v < vert_count; ++v)
        {
            mn = m3d::vec3(m3d::min(mn.x, vertices[v].x),
                           m3d::min(mn.y, vertices[v].y),
                           m3d::min(mn.z, vertices[v].z));
            mx = m3d::vec3(m3d::max(mx.x, vertices[v].x),
                           m3d::max(mx.y, vertices[v].y),
                           m3d::max(mx.z, vertices[v].z));
        }
        hd->local_aabb = {mn, mx};
        return hd;
    }

    /**
     * @brief Free a `ConvexHullData` returned by `convex_hull_data_create`.
     *
     * Releases the precomputed `face_normals` array (when present) and
     * the `ConvexHullData` itself. The caller-supplied `vertices` and
     * `face_indices` arrays are NOT freed — they remain owned by the
     * caller.
     *
     * @ingroup rbc
     */
    inline void convex_hull_data_destroy(ConvexHullData *hd)
    {
        if (!hd)
            return;
        delete[] hd->face_normals;
        delete hd;
    }

    /**
     * @brief Convex-hull support: vertex of `h` farthest along `dir`.
     *
     * Vertex scan over the precomputed local vertex array. Linear in the
     * vertex count, which is fine for typical hulls (≤ 32 vertices).
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const ConvexHull &h, const m3d::vec3 &dir)
    {
        if (!h.data || h.data->vert_count == 0)
            return m3d::vec3{};
        m3d::vec3 best = h.data->vertices[0];
        m3d::scalar best_d = m3d::dot(best, dir);
        for (uint32_t i = 1; i < h.data->vert_count; ++i)
        {
            const m3d::scalar d = m3d::dot(h.data->vertices[i], dir);
            if (d > best_d)
            {
                best_d = d;
                best = h.data->vertices[i];
            }
        }
        return best;
    }

    /**
     * @brief Conservative world AABB from the precomputed local AABB.
     *
     * Same OBB-projection technique as `Box::compute_aabb` and
     * `Mesh::compute_aabb`. Conservative — for tight bounds you would
     * project every world vertex, which is no longer O(1).
     *
     * @ingroup rbc
     */
    inline AABB compute_aabb(const ConvexHull &h, const m3d::tf &tf)
    {
        if (!h.data)
            return AABB{};
        const AABB &local = h.data->local_aabb;
        const m3d::vec3 centre = (local.min + local.max) * 0.5;
        const m3d::vec3 half   = (local.max - local.min) * 0.5;

        const m3d::mat3 R = m3d::mat3_cast(tf.rot);
        const m3d::vec3 world_centre = tf.pos + tf.rotate_vector(centre);
        const m3d::vec3 extent(
            m3d::abs(R[0][0]) * half.x + m3d::abs(R[1][0]) * half.y + m3d::abs(R[2][0]) * half.z,
            m3d::abs(R[0][1]) * half.x + m3d::abs(R[1][1]) * half.y + m3d::abs(R[2][1]) * half.z,
            m3d::abs(R[0][2]) * half.x + m3d::abs(R[1][2]) * half.y + m3d::abs(R[2][2]) * half.z);
        return {world_centre - extent, world_centre + extent};
    }

    /**
     * @brief Tag-dispatched marker: ConvexHull rides the GJK fast path (true).
     *
     * Flipping this from `false` (as `Mesh` carries it) to `true` is the
     * single change that promotes a vertex array from "treated as a
     * triangle soup" to "first-class convex collider." The pointer is
     * never dereferenced; this is a compile-time predicate.
     *
     * @ingroup rbc
     */
    constexpr bool is_gjk_convex(const ConvexHull *) { return true; }

    /**
     * @brief Representative size = max `|v|` over the local vertices.
     *
     * Linear scan; cheap relative to a collision step but consider
     * caching in `ConvexHullData` if profiling shows it matters.
     *
     * @ingroup rbc
     */
    inline m3d::scalar representative_radius(const ConvexHull &h)
    {
        if (!h.data || h.data->vert_count == 0)
            return 0.0;
        m3d::scalar best2 = 0.0;
        for (uint32_t i = 0; i < h.data->vert_count; ++i)
        {
            const m3d::scalar d2 = m3d::length_sq(h.data->vertices[i]);
            if (d2 > best2)
                best2 = d2;
        }
        return m3d::sqrt(best2);
    }

    /**
     * @brief Face polygon most aligned with `dir`.
     *
     * - With face data: pick the face whose local outward normal is most
     *   aligned with `dir` (transformed to local space), transform its 3
     *   vertices to world space, return 3 corners.
     * - Without face data: fall back to the disc-approximation patch
     *   (same as Sphere/Capsule), centred on the world support point.
     *
     * @ingroup rbc
     */
    inline int face_corners(const ConvexHull &h, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 out[4])
    {
        if (!h.data || h.data->vert_count == 0)
            return 0;

        if (h.data->face_count == 0 || !h.data->face_indices || !h.data->face_normals)
        {
            const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
            const m3d::vec3 sup_world = tf.transform_point(support(h, local_dir));
            return get_generic_face_corners(sup_world, dir, representative_radius(h), out);
        }

        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
        uint32_t best = 0;
        m3d::scalar best_d = m3d::dot(h.data->face_normals[0], local_dir);
        for (uint32_t f = 1; f < h.data->face_count; ++f)
        {
            const m3d::scalar d = m3d::dot(h.data->face_normals[f], local_dir);
            if (d > best_d)
            {
                best_d = d;
                best = f;
            }
        }

        out[0] = tf.transform_point(h.data->vertices[h.data->face_indices[best * 3 + 0]]);
        out[1] = tf.transform_point(h.data->vertices[h.data->face_indices[best * 3 + 1]]);
        out[2] = tf.transform_point(h.data->vertices[h.data->face_indices[best * 3 + 2]]);
        return 3;
    }

    /**
     * @brief Volume of the hull.
     *
     * - With face data: sum of signed tetrahedron volumes
     *   `(O, A, B, C)` over all triangles, with origin as the apex.
     *   Exact for any closed CCW-wound triangulation.
     * - Without face data: volume of the local AABB (loose upper bound).
     *
     * @ingroup rbc
     */
    inline m3d::scalar compute_volume(const ConvexHull &h)
    {
        if (!h.data)
            return 0.0;

        if (h.data->face_count == 0 || !h.data->face_indices)
        {
            const m3d::vec3 ext = h.data->local_aabb.max - h.data->local_aabb.min;
            return ext.x * ext.y * ext.z;
        }

        m3d::scalar v6 = 0.0;
        for (uint32_t f = 0; f < h.data->face_count; ++f)
        {
            const m3d::vec3 &A = h.data->vertices[h.data->face_indices[f * 3 + 0]];
            const m3d::vec3 &B = h.data->vertices[h.data->face_indices[f * 3 + 1]];
            const m3d::vec3 &C = h.data->vertices[h.data->face_indices[f * 3 + 2]];
            v6 += m3d::dot(A, m3d::cross(B, C));
        }
        return v6 / 6.0;
    }

    /**
     * @brief Inertia tensor of a uniform-density hull about its centre of mass.
     *
     * - With face data: tetrahedral decomposition from the origin
     *   following the standard polynomial moment integrals (see
     *   Mirtich 1996), then translated to the centroid via the
     *   parallel-axis theorem. Mass is taken as the volume (unit
     *   density) — multiply by your body's density at construction.
     * - Without face data: inertia of the local AABB (Box formula).
     *   A one-shot debug message is emitted to `stderr` so the caller
     *   knows the result is approximate. The fallback is silenced
     *   under `NDEBUG`.
     *
     * @ingroup rbc
     */
    inline m3d::smat3 compute_inertia_tensor(const ConvexHull &h)
    {
        if (!h.data)
            return m3d::smat3();

        if (h.data->face_count == 0 || !h.data->face_indices)
        {
#ifndef NDEBUG
            static bool warned = false;
            if (!warned)
            {
                warned = true;
                std::fprintf(stderr,
                             "[rbc] ConvexHull inertia: face data missing; "
                             "using AABB approximation.\n");
            }
#endif
            const m3d::vec3 ext = h.data->local_aabb.max - h.data->local_aabb.min;
            const m3d::scalar mass = ext.x * ext.y * ext.z;
            const m3d::scalar x2 = ext.x * ext.x;
            const m3d::scalar y2 = ext.y * ext.y;
            const m3d::scalar z2 = ext.z * ext.z;
            return m3d::smat3(
                (1.0 / 12.0) * mass * (y2 + z2),
                (1.0 / 12.0) * mass * (x2 + z2),
                (1.0 / 12.0) * mass * (x2 + y2),
                0.0, 0.0, 0.0);
        }

        m3d::scalar total_v6 = 0.0;
        m3d::vec3   first_moment(0.0, 0.0, 0.0);
        m3d::scalar Ixx_o = 0.0, Iyy_o = 0.0, Izz_o = 0.0;
        m3d::scalar Ixy_o = 0.0, Ixz_o = 0.0, Iyz_o = 0.0;

        for (uint32_t f = 0; f < h.data->face_count; ++f)
        {
            const m3d::vec3 &A = h.data->vertices[h.data->face_indices[f * 3 + 0]];
            const m3d::vec3 &B = h.data->vertices[h.data->face_indices[f * 3 + 1]];
            const m3d::vec3 &C = h.data->vertices[h.data->face_indices[f * 3 + 2]];

            const m3d::scalar det = m3d::dot(A, m3d::cross(B, C));
            const m3d::scalar dV  = det / 6.0;
            total_v6 += det;

            first_moment = first_moment + (A + B + C) * (dV * 0.25);

            auto sum_sq = [det](m3d::scalar a, m3d::scalar b, m3d::scalar c) {
                return (det / 60.0) * (a * a + b * b + c * c + a * b + a * c + b * c);
            };
            auto sum_prod = [det](m3d::scalar ax, m3d::scalar ay,
                                  m3d::scalar bx, m3d::scalar by,
                                  m3d::scalar cx, m3d::scalar cy) {
                return (det / 120.0) * (
                    2.0 * (ax * ay + bx * by + cx * cy)
                    + ax * by + ay * bx
                    + ax * cy + ay * cx
                    + bx * cy + by * cx);
            };

            const m3d::scalar mxx = sum_sq(A.x, B.x, C.x);
            const m3d::scalar myy = sum_sq(A.y, B.y, C.y);
            const m3d::scalar mzz = sum_sq(A.z, B.z, C.z);
            const m3d::scalar mxy = sum_prod(A.x, A.y, B.x, B.y, C.x, C.y);
            const m3d::scalar mxz = sum_prod(A.x, A.z, B.x, B.z, C.x, C.z);
            const m3d::scalar myz = sum_prod(A.y, A.z, B.y, B.z, C.y, C.z);

            Ixx_o += myy + mzz;
            Iyy_o += mxx + mzz;
            Izz_o += mxx + myy;
            Ixy_o -= mxy;
            Ixz_o -= mxz;
            Iyz_o -= myz;
        }

        const m3d::scalar total_vol = total_v6 / 6.0;
        if (m3d::abs(total_vol) < m3d::EPSILON)
            return m3d::smat3();

        const m3d::vec3   centroid = first_moment / total_vol;
        const m3d::scalar mass     = total_vol; // unit density
        const m3d::scalar cx = centroid.x, cy = centroid.y, cz = centroid.z;

        return m3d::smat3(
            Ixx_o - mass * (cy * cy + cz * cz),
            Iyy_o - mass * (cx * cx + cz * cz),
            Izz_o - mass * (cx * cx + cy * cy),
            Ixy_o + mass * cx * cy,
            Ixz_o + mass * cx * cz,
            Iyz_o + mass * cy * cz);
    }

} // namespace rbc
