#pragma once
#include <cstdint>
#include <cmath>
#include <limits>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

/**
 * @file Heightmap.hpp
 * @brief Heightmap (terrain grid) collision shape.
 * @ingroup rbc
 *
 * @warning The heightmap collider is currently incomplete. Sphere–heightmap
 *          and capsule–heightmap pairs have analytic implementations and
 *          tests; box–heightmap and mesh–heightmap are not yet wired up.
 *          See `docs/design/heightmap.md` for the planned rewrite.
 */

namespace rbc
{
    /**
     * @brief Backing storage for a row-major grid of height samples.
     *
     * The world position of vertex `(row, col)` is:
     * - `pos.x = origin.x + col * scale.x`
     * - `pos.y = heights[row * cols + col] * scale.y + origin.y`
     * - `pos.z = origin.z + row * scale.z`
     *
     * Each grid cell `(row, col)` is split into two triangles:
     * - tri0: `(r,c), (r,c+1), (r+1,c)`
     * - tri1: `(r,c+1), (r+1,c+1), (r+1,c)`
     *
     * @note `Heightmap` holds a non-owning pointer to `HeightmapData`. The
     *       data must outlive every `Heightmap` that references it.
     *
     * @ingroup rbc
     */
    struct HeightmapData
    {
        const float *heights;    ///< [rows × cols] height samples (Y, before scale).
        uint32_t rows;           ///< Number of grid rows.
        uint32_t cols;           ///< Number of grid columns.
        m3d::vec3 scale;         ///< (cell_size_x, height_scale, cell_size_z).
        m3d::vec3 origin;        ///< World position of the (0, 0) vertex.
        m3d::scalar min_height;  ///< Precomputed: `min(heights) * scale.y + origin.y`.
        m3d::scalar max_height;  ///< Precomputed: `max(heights) * scale.y + origin.y`.
    };

    /**
     * @brief Heightmap shape — wraps a non-owning pointer to `HeightmapData`.
     *
     * The body's `m3d::tf` is intentionally ignored for heightmap queries;
     * world position is baked into `HeightmapData::origin` so the terrain
     * is always axis-aligned.
     *
     * @ingroup rbc
     */
    struct Heightmap
    {
        const HeightmapData *data; ///< Non-owning pointer; must outlive this shape.

        /** @brief Default-construct with no data attached. */
        Heightmap() : data(nullptr) {}
        /** @brief Wrap an existing `HeightmapData` (caller retains ownership). */
        explicit Heightmap(const HeightmapData *data) : data(data) {}

        /** @brief Equality on the data pointer. */
        inline bool operator==(const Heightmap &o) const { return data == o.data; }
        /** @brief Inequality on the data pointer. */
        inline bool operator!=(const Heightmap &o) const { return data != o.data; }
    };

    /**
     * @brief Allocate a `HeightmapData` on the heap and precompute min/max height.
     *
     * Caller owns the returned pointer and must call `heightmap_data_destroy`
     * to free it. The `heights` array is referenced, not copied — keep it
     * alive for as long as the returned `HeightmapData` is in use.
     *
     * @ingroup rbc
     */
    inline HeightmapData *heightmap_data_create(const float *heights,
                                                uint32_t rows,
                                                uint32_t cols,
                                                const m3d::vec3 &scale,
                                                const m3d::vec3 &origin)
    {
        auto *hd = new HeightmapData;
        hd->heights = heights;
        hd->rows = rows;
        hd->cols = cols;
        hd->scale = scale;
        hd->origin = origin;

        float mn = std::numeric_limits<float>::max();
        float mx = std::numeric_limits<float>::lowest();
        const uint32_t n = rows * cols;
        for (uint32_t i = 0; i < n; ++i)
        {
            mn = std::min(mn, heights[i]);
            mx = std::max(mx, heights[i]);
        }
        hd->min_height = static_cast<m3d::scalar>(mn) * scale.y + origin.y;
        hd->max_height = static_cast<m3d::scalar>(mx) * scale.y + origin.y;
        return hd;
    }

    /** @brief Free a `HeightmapData` returned by `heightmap_data_create`. @ingroup rbc */
    inline void heightmap_data_destroy(HeightmapData *hd) { delete hd; }

    /**
     * @brief World-space position of the grid vertex at `(row, col)`.
     * @ingroup rbc
     */
    inline m3d::vec3 heightmap_vertex(const HeightmapData &hd, uint32_t row, uint32_t col)
    {
        return m3d::vec3(
            hd.origin.x + static_cast<m3d::scalar>(col) * hd.scale.x,
            static_cast<m3d::scalar>(hd.heights[row * hd.cols + col]) * hd.scale.y + hd.origin.y,
            hd.origin.z + static_cast<m3d::scalar>(row) * hd.scale.z);
    }

    /**
     * @brief Stub support function — heightmap is not used through GJK.
     *
     * Heightmap pairs go through analytic algorithms in
     * [analytic/HeightmapCollision.hpp](analytic/HeightmapCollision.hpp).
     *
     * @ingroup rbc
     */
    inline m3d::vec3 support(const Heightmap & /*h*/, const m3d::vec3 & /*dir*/)
    {
        return m3d::vec3(0.0, 0.0, 0.0);
    }

    /** @brief World AABB built from precomputed `min_height` / `max_height`. @ingroup rbc */
    inline AABB compute_aabb(const Heightmap &h, const m3d::tf & /*tf*/)
    {
        if (!h.data)
            return AABB{};
        const HeightmapData &hd = *h.data;
        return {
            m3d::vec3(hd.origin.x,
                      hd.min_height,
                      hd.origin.z),
            m3d::vec3(hd.origin.x + static_cast<m3d::scalar>(hd.cols - 1) * hd.scale.x,
                      hd.max_height,
                      hd.origin.z + static_cast<m3d::scalar>(hd.rows - 1) * hd.scale.z)};
    }

    /** @brief Tag-dispatched marker: Heightmap is non-convex (false). @ingroup rbc */
    constexpr bool is_gjk_convex(const Heightmap *) { return false; }
    /** @brief Returns 0 — terrain has no representative size. @ingroup rbc */
    inline m3d::scalar representative_radius(const Heightmap &) { return 0.0; }
    /** @brief Stub — heightmap pairs go through analytic algorithms. @ingroup rbc */
    inline int face_corners(const Heightmap &, const m3d::tf &,
                            const m3d::vec3 &, m3d::vec3[4]) { return 0; }
} // namespace rbc
