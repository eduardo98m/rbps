#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>
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
     * arrays alive); owning over `face_normals` and the merged-polygon
     * arrays (allocated by `convex_hull_data_create` when face data is
     * supplied).
     *
     * Memory layout:
     * - `vertices[i]`              — local-space vertex positions.
     * - `face_indices[f*3 + {0,1,2}]` — vertex indices of triangle face `f` (CCW).
     * - `face_normals[f]`          — unit outward normal of triangle `f`, precomputed.
     *
     * @par Merged polygon faces
     * Adjacent coplanar triangles are grouped into a single logical face
     * polygon at create time. The contact-manifold generator uses these
     * polygons (not the per-triangle slices) so that fan-triangulated
     * faces like a hex-prism cap clip as a single hexagon instead of one
     * of its sub-triangles. For a hull where every face is a triangle
     * (e.g. tetrahedron) the polygon set is identical to the triangle set.
     *
     * - `poly_indices[poly_offsets[p] .. poly_offsets[p+1]-1]` — vertex
     *   indices of polygon `p`, in CCW order around `poly_normals[p]`.
     * - `poly_offsets` length == `poly_count + 1`; the last entry equals
     *   the total number of indices in `poly_indices`.
     * - `poly_normals[p]` — averaged unit outward normal of polygon `p`.
     *
     * Set `face_indices == nullptr` and `face_count == 0` for a
     * vertex-only hull; `poly_*` will all be null with `poly_count == 0`.
     *
     * @ingroup rbc
     */
    struct ConvexHullData
    {
        const m3d::vec3 *vertices;     ///< Vertex positions in local space (non-owning).
        const uint32_t  *face_indices; ///< Triangle indices, 3 per face (non-owning); may be `nullptr`.
        const uint32_t  *face_offsets; ///< Reserved (currently unused); kept for ABI stability.
        const m3d::vec3 *face_normals; ///< Unit outward normal per triangle (owned).
        uint32_t         vert_count;   ///< Number of entries in `vertices`.
        uint32_t         face_count;   ///< Number of triangles; 0 when `face_indices == nullptr`.
        AABB             local_aabb;   ///< Precomputed local-space AABB.

        const uint32_t  *poly_indices; ///< Packed vertex indices of merged polygons (owned).
        const uint32_t  *poly_offsets; ///< Length `poly_count + 1`; ranges into `poly_indices` (owned).
        const m3d::vec3 *poly_normals; ///< Unit outward normal per polygon (owned).
        uint32_t         poly_count;   ///< Number of merged polygon faces.

        // Adjacency tables used by the manifold pipeline. Built once at
        // construction; null when the hull has no merged polygons.
        const uint32_t  *vertex_to_polys_offsets;     ///< Length `vert_count + 1` (owned).
        const uint32_t  *vertex_to_polys_indices;     ///< Polygons containing each vertex (owned).
        const uint32_t  *vertex_to_neighbors_offsets; ///< Length `vert_count + 1` (owned).
        const uint32_t  *vertex_to_neighbors_indices; ///< Polygon-edge neighbors per vertex (owned).
        const uint32_t  *poly_to_polys_offsets;       ///< Length `poly_count + 1` (owned).
        const uint32_t  *poly_to_polys_indices;       ///< Polygons sharing an edge with each polygon (owned).
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
        hd->poly_indices = nullptr;
        hd->poly_offsets = nullptr;
        hd->poly_normals = nullptr;
        hd->poly_count   = 0;
        hd->vertex_to_polys_offsets     = nullptr;
        hd->vertex_to_polys_indices     = nullptr;
        hd->vertex_to_neighbors_offsets = nullptr;
        hd->vertex_to_neighbors_indices = nullptr;
        hd->poly_to_polys_offsets       = nullptr;
        hd->poly_to_polys_indices       = nullptr;

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

            // ── Merge coplanar triangles into logical polygon faces ───────
            // Two triangles belong to the same face when their outward
            // normals point in the same direction within `kCoplanarDot`.
            // For a convex polytope, parallel-normal triangles ARE coplanar
            // (no two distinct planes share a normal direction on a convex
            // hull), so the normal test is sufficient — no plane-distance
            // check needed.
            constexpr m3d::scalar kCoplanarDot = 0.99995; // ~0.57°
            std::vector<int> parent(face_count);
            for (uint32_t i = 0; i < face_count; ++i) parent[i] = static_cast<int>(i);
            auto find = [&parent](int x) {
                while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
                return x;
            };
            for (uint32_t i = 0; i < face_count; ++i)
                for (uint32_t j = i + 1; j < face_count; ++j)
                    if (m3d::dot(norms[i], norms[j]) > kCoplanarDot)
                    {
                        const int ri = find(static_cast<int>(i));
                        const int rj = find(static_cast<int>(j));
                        if (ri != rj) parent[ri] = rj;
                    }

            // Collect triangles per group and average normals.
            std::vector<std::vector<uint32_t>> groups;
            std::vector<int> root_to_group(face_count, -1);
            for (uint32_t i = 0; i < face_count; ++i)
            {
                const int r = find(static_cast<int>(i));
                if (root_to_group[r] < 0)
                {
                    root_to_group[r] = static_cast<int>(groups.size());
                    groups.emplace_back();
                }
                groups[root_to_group[r]].push_back(i);
            }

            // For each group: walk the boundary of the union by collecting
            // edges that appear in only one triangle of the group, then
            // chain them tip-to-tail into one CCW cycle.
            std::vector<uint32_t> all_indices;
            std::vector<uint32_t> offsets;
            std::vector<m3d::vec3> poly_norms_vec;
            offsets.push_back(0);

            for (const auto &group : groups)
            {
                // Gather directed edges; index by source vertex for chaining.
                struct DirEdge { uint32_t a, b; };
                std::vector<DirEdge> tri_edges;
                tri_edges.reserve(group.size() * 3);
                for (const uint32_t f : group)
                {
                    const uint32_t a = face_indices[f * 3 + 0];
                    const uint32_t b = face_indices[f * 3 + 1];
                    const uint32_t c = face_indices[f * 3 + 2];
                    tri_edges.push_back({a, b});
                    tri_edges.push_back({b, c});
                    tri_edges.push_back({c, a});
                }

                // An edge (a→b) is INTERIOR iff (b→a) appears elsewhere.
                std::vector<DirEdge> boundary;
                boundary.reserve(tri_edges.size());
                for (size_t i = 0; i < tri_edges.size(); ++i)
                {
                    bool reversed_present = false;
                    for (size_t j = 0; j < tri_edges.size(); ++j)
                    {
                        if (i == j) continue;
                        if (tri_edges[j].a == tri_edges[i].b &&
                            tri_edges[j].b == tri_edges[i].a)
                        { reversed_present = true; break; }
                    }
                    if (!reversed_present) boundary.push_back(tri_edges[i]);
                }

                if (boundary.empty()) continue; // pathological — drop polygon

                // Chain edges: start with boundary[0], find next whose `a`
                // matches current `b`, repeat until we close the loop.
                std::vector<bool> used(boundary.size(), false);
                std::vector<uint32_t> loop;
                loop.reserve(boundary.size());

                used[0] = true;
                loop.push_back(boundary[0].a);
                uint32_t current = boundary[0].b;
                for (size_t step = 1; step < boundary.size(); ++step)
                {
                    bool found = false;
                    for (size_t k = 0; k < boundary.size(); ++k)
                    {
                        if (used[k]) continue;
                        if (boundary[k].a == current)
                        {
                            used[k] = true;
                            loop.push_back(boundary[k].a);
                            current = boundary[k].b;
                            found = true;
                            break;
                        }
                    }
                    if (!found) break; // open chain — keep what we have
                }

                if (loop.size() < 3) continue;

                // Average the group's triangle normals to reduce FP noise.
                m3d::vec3 avg_n(0, 0, 0);
                for (const uint32_t f : group) avg_n = avg_n + norms[f];
                const m3d::scalar an_len = m3d::length(avg_n);
                if (an_len < m3d::EPSILON) continue;
                avg_n = avg_n / an_len;

                for (uint32_t idx : loop) all_indices.push_back(idx);
                offsets.push_back(static_cast<uint32_t>(all_indices.size()));
                poly_norms_vec.push_back(avg_n);
            }

            const uint32_t pcount = static_cast<uint32_t>(poly_norms_vec.size());
            if (pcount > 0)
            {
                auto *pi = new uint32_t[all_indices.size()];
                auto *po = new uint32_t[pcount + 1];
                auto *pn = new m3d::vec3[pcount];
                for (size_t k = 0; k < all_indices.size(); ++k) pi[k] = all_indices[k];
                for (uint32_t k = 0; k <= pcount; ++k) po[k] = offsets[k];
                for (uint32_t k = 0; k < pcount; ++k) pn[k] = poly_norms_vec[k];
                hd->poly_indices = pi;
                hd->poly_offsets = po;
                hd->poly_normals = pn;
                hd->poly_count   = pcount;

                // ── Adjacency tables used by the manifold pipeline ────────
                // O(P²) edge-pairing pass on poly count P (typically ≤ 32).
                // Rebuild with hash maps if vert_count > ~256.
                std::vector<std::vector<uint32_t>> v2p(vert_count);
                std::vector<std::vector<uint32_t>> v2n(vert_count);
                std::vector<std::vector<uint32_t>> p2p(pcount);

                for (uint32_t p = 0; p < pcount; ++p)
                {
                    const uint32_t s = po[p];
                    const uint32_t e = po[p + 1];
                    const uint32_t n = e - s;
                    for (uint32_t i = 0; i < n; ++i)
                    {
                        const uint32_t a = pi[s + i];
                        const uint32_t b = pi[s + (i + 1) % n];
                        auto &vp = v2p[a];
                        if (std::find(vp.begin(), vp.end(), p) == vp.end())
                            vp.push_back(p);
                        auto &va = v2n[a];
                        if (std::find(va.begin(), va.end(), b) == va.end())
                            va.push_back(b);
                        auto &vb = v2n[b];
                        if (std::find(vb.begin(), vb.end(), a) == vb.end())
                            vb.push_back(a);
                    }
                }

                // Two CCW polygons share an edge when one has (a,b) and the
                // other has (b,a). Scan all polygon pairs.
                for (uint32_t p = 0; p < pcount; ++p)
                {
                    const uint32_t s_p = po[p];
                    const uint32_t n_p = po[p + 1] - s_p;
                    for (uint32_t q = 0; q < pcount; ++q)
                    {
                        if (q == p) continue;
                        const uint32_t s_q = po[q];
                        const uint32_t n_q = po[q + 1] - s_q;
                        bool shared = false;
                        for (uint32_t i = 0; i < n_p && !shared; ++i)
                        {
                            const uint32_t a = pi[s_p + i];
                            const uint32_t b = pi[s_p + (i + 1) % n_p];
                            for (uint32_t j = 0; j < n_q; ++j)
                            {
                                const uint32_t c = pi[s_q + j];
                                const uint32_t d = pi[s_q + (j + 1) % n_q];
                                if (a == d && b == c) { shared = true; break; }
                            }
                        }
                        if (shared) p2p[p].push_back(q);
                    }
                }

                auto flatten = [](const std::vector<std::vector<uint32_t>> &src,
                                  uint32_t **out_offs, uint32_t **out_idx) {
                    const uint32_t n = static_cast<uint32_t>(src.size());
                    auto *offs = new uint32_t[n + 1];
                    offs[0] = 0;
                    uint32_t total = 0;
                    for (uint32_t i = 0; i < n; ++i)
                    {
                        total += static_cast<uint32_t>(src[i].size());
                        offs[i + 1] = total;
                    }
                    uint32_t *idx = (total > 0) ? new uint32_t[total] : nullptr;
                    uint32_t k = 0;
                    for (uint32_t i = 0; i < n; ++i)
                        for (uint32_t v : src[i])
                            idx[k++] = v;
                    *out_offs = offs;
                    *out_idx  = idx;
                };

                uint32_t *v2p_o = nullptr, *v2p_i = nullptr;
                uint32_t *v2n_o = nullptr, *v2n_i = nullptr;
                uint32_t *p2p_o = nullptr, *p2p_i = nullptr;
                flatten(v2p, &v2p_o, &v2p_i);
                flatten(v2n, &v2n_o, &v2n_i);
                flatten(p2p, &p2p_o, &p2p_i);
                hd->vertex_to_polys_offsets     = v2p_o;
                hd->vertex_to_polys_indices     = v2p_i;
                hd->vertex_to_neighbors_offsets = v2n_o;
                hd->vertex_to_neighbors_indices = v2n_i;
                hd->poly_to_polys_offsets       = p2p_o;
                hd->poly_to_polys_indices       = p2p_i;
            }
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
        delete[] hd->poly_indices;
        delete[] hd->poly_offsets;
        delete[] hd->poly_normals;
        delete[] hd->vertex_to_polys_offsets;
        delete[] hd->vertex_to_polys_indices;
        delete[] hd->vertex_to_neighbors_offsets;
        delete[] hd->vertex_to_neighbors_indices;
        delete[] hd->poly_to_polys_offsets;
        delete[] hd->poly_to_polys_indices;
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
     * - With merged polygons: pick the polygon whose averaged outward
     *   normal is most aligned with `dir` (transformed to local space),
     *   transform its vertices to world space, return up to `capacity`
     *   corners. This is the path that supplies the contact-manifold
     *   generator with a true logical-face polygon (e.g. the full hexagon
     *   of a hex-prism cap rather than one of its 4 fan triangles).
     * - With only triangle data and no merged polygons (vert-only hull
     *   or merge produced nothing): fall back to picking the single
     *   best-aligned triangle, returning 3 corners.
     * - Without any face data: disc-approximation patch (same as
     *   Sphere/Capsule), centred on the world support point.
     *
     * @param capacity Capacity of `out`; the returned count is clamped
     *                 to this. Pass `kMaxFaceCorners`-sized buffers in
     *                 manifold-generation callers.
     *
     * @ingroup rbc
     */
    inline int face_corners(const ConvexHull &h, const m3d::tf &tf,
                            const m3d::vec3 &dir, m3d::vec3 *out, int capacity)
    {
        if (!h.data || h.data->vert_count == 0 || capacity <= 0)
            return 0;

        if (h.data->face_count == 0 || !h.data->face_indices || !h.data->face_normals)
        {
            const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);
            const m3d::vec3 sup_world = tf.transform_point(support(h, local_dir));
            return get_generic_face_corners(sup_world, dir, representative_radius(h), out, capacity);
        }

        const m3d::vec3 local_dir = tf.inverse_rotate_vector(dir);

        // Prefer the merged polygon set when available — these are the
        // actual logical faces and avoid triangulation seams becoming
        // false silhouette edges in Sutherland-Hodgman clipping.
        if (h.data->poly_count > 0 && h.data->poly_normals && h.data->poly_indices && h.data->poly_offsets)
        {
            uint32_t best = 0;
            m3d::scalar best_d = m3d::dot(h.data->poly_normals[0], local_dir);
            for (uint32_t p = 1; p < h.data->poly_count; ++p)
            {
                const m3d::scalar d = m3d::dot(h.data->poly_normals[p], local_dir);
                if (d > best_d) { best_d = d; best = p; }
            }
            const uint32_t start = h.data->poly_offsets[best];
            const uint32_t end   = h.data->poly_offsets[best + 1];
            const int n = static_cast<int>(end - start);
            const int written = (n < capacity) ? n : capacity;
            for (int i = 0; i < written; ++i)
                out[i] = tf.transform_point(h.data->vertices[h.data->poly_indices[start + i]]);
            return written;
        }

        // No merged polygons (degenerate hull or merge failed): fall
        // back to a single best-aligned triangle.
        if (capacity < 3) return 0;
        uint32_t best = 0;
        m3d::scalar best_d = m3d::dot(h.data->face_normals[0], local_dir);
        for (uint32_t f = 1; f < h.data->face_count; ++f)
        {
            const m3d::scalar d = m3d::dot(h.data->face_normals[f], local_dir);
            if (d > best_d) { best_d = d; best = f; }
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
