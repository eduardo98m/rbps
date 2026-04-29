#pragma once
#include "rbc/shapes/Heightmap.hpp"

// ── Heightmap factories for tests ──────────────────────────────────────────
// Replaces make_flat_heightmap / make_hill_heightmap duplicated across
// test_heightmap_sphere.cpp and test_heightmap_capsule.cpp, and adds a
// ScopedHeightmap RAII guard so tests don't have to call destroy at the end
// of every case.
namespace test
{
    // 3x3 flat heightmap. Cell size 1x1 in XZ. Spans X=[0,2], Z=[0,2].
    // The static array is shared across calls within a TU (matches the prior
    // test-local pattern); the returned HeightmapData is heap-allocated and
    // owned by the caller (use ScopedHeightmap below to manage it).
    inline rbc::HeightmapData *make_flat_heightmap_3x3(float height = 0.0f)
    {
        static float h[9];
        for (int i = 0; i < 9; ++i)
            h[i] = height;
        return rbc::heightmap_data_create(h, 3, 3,
                                          m3d::vec3(1.0, 1.0, 1.0),
                                          m3d::vec3(0.0, 0.0, 0.0));
    }

    // 3x3 heightmap with a unit-height hill at the centre vertex (index [1,1]=1).
    inline rbc::HeightmapData *make_hill_heightmap_3x3()
    {
        static float h[9] = {
            0, 0, 0,
            0, 1, 0,
            0, 0, 0,
        };
        return rbc::heightmap_data_create(h, 3, 3,
                                          m3d::vec3(1.0, 1.0, 1.0),
                                          m3d::vec3(0.0, 0.0, 0.0));
    }

    // RAII guard so tests can write `test::ScopedHeightmap hd(make_flat_…())`
    // and forget about heightmap_data_destroy().
    struct ScopedHeightmap
    {
        rbc::HeightmapData *data;
        ScopedHeightmap(rbc::HeightmapData *d) : data(d) {}
        ~ScopedHeightmap() { rbc::heightmap_data_destroy(data); }
        ScopedHeightmap(const ScopedHeightmap &)            = delete;
        ScopedHeightmap &operator=(const ScopedHeightmap &) = delete;
        operator rbc::HeightmapData *() const { return data; }
    };
} // namespace test
