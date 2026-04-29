#pragma once
#include <cstdint>
#include <cmath>
#include <limits>
#include <math3d/math3d.hpp>
#include "rbc/AABB.hpp"

namespace rbc
{
    // ── HeightmapData ─────────────────────────────────────────────────────────
    // Owns (or references) a row-major grid of height samples.
    // Vertex world position at grid index (row, col):
    //   pos.x = origin.x + col * scale.x
    //   pos.y = heights[row * cols + col] * scale.y
    //   pos.z = origin.z + row * scale.z
    //
    // Two triangles per cell (row,col) → (row+1,col+1):
    //   tri0: (r,c), (r,c+1), (r+1,c)
    //   tri1: (r,c+1), (r+1,c+1), (r+1,c)
    struct HeightmapData
    {
        const float *heights; // [rows × cols] height samples (Y before scale)
        uint32_t rows;
        uint32_t cols;
        m3d::vec3 scale;        // (cell_size_x, height_scale, cell_size_z)
        m3d::vec3 origin;       // world position of the (0,0) vertex
        m3d::scalar min_height; // precomputed: min(heights)*scale.y + origin.y
        m3d::scalar max_height; // precomputed: max(heights)*scale.y + origin.y
    };

    // ── Heightmap shape ───────────────────────────────────────────────────────
    // Holds a POINTER — HeightmapData must outlive the Heightmap.
    // The transform tf is intentionally ignored for heightmap queries;
    // world position is baked into HeightmapData::origin.
    struct Heightmap
    {
        const HeightmapData *data; // non-owning pointer

        Heightmap() : data(nullptr) {}
        explicit Heightmap(const HeightmapData *data) : data(data) {}

        inline bool operator==(const Heightmap &o) const { return data == o.data; }
        inline bool operator!=(const Heightmap &o) const { return data != o.data; }
    };

    // ── Construction helper ───────────────────────────────────────────────────
    // Creates a HeightmapData on the heap and precomputes min/max height.
    // Caller owns the returned pointer and must call heightmap_data_destroy().
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

    inline void heightmap_data_destroy(HeightmapData *hd) { delete hd; }

    // ── World position of a grid vertex ──────────────────────────────────────
    inline m3d::vec3 heightmap_vertex(const HeightmapData &hd, uint32_t row, uint32_t col)
    {
        return m3d::vec3(
            hd.origin.x + static_cast<m3d::scalar>(col) * hd.scale.x,
            static_cast<m3d::scalar>(hd.heights[row * hd.cols + col]) * hd.scale.y + hd.origin.y,
            hd.origin.z + static_cast<m3d::scalar>(row) * hd.scale.z);
    }

    // ── Support (not meaningful for GJK — use analytic collisions) ───────────
    inline m3d::vec3 support(const Heightmap & /*h*/, const m3d::vec3 & /*dir*/)
    {
        return m3d::vec3(0.0, 0.0, 0.0); // placeholder; not used directly
    }

    // ── AABB from precomputed min/max height ──────────────────────────────────
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

    // Marker for the dispatcher: Heightmap is non-convex (terrain field).
    // Pairs go through analytic specialisations; these stubs only exist so
    // the variant-level visit/table compiles for every shape kind.
    constexpr bool is_gjk_convex(const Heightmap *) { return false; }
    inline m3d::scalar representative_radius(const Heightmap &) { return 0.0; }
    inline int face_corners(const Heightmap &, const m3d::tf &,
                            const m3d::vec3 &, m3d::vec3[4]) { return 0; }
} // namespace rbc