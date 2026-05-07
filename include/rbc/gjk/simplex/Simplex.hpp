#pragma once
#include <math3d/math3d.hpp>

namespace rbc
{
    // One support sample of the Minkowski difference. `w` is the
    // difference point used by GJK; `w0` and `w1` are the per-shape
    // support points retained for EPA polytope expansion and for the
    // visual debugger to render the originating samples.
    struct SimplexVertex
    {
        m3d::vec3 w0;
        m3d::vec3 w1;
        m3d::vec3 w;
    };

    // GJK working simplex. `vertex[0]` is always the most recently
    // added support; older entries trail behind. `rank` ranges 0..4.
    //
    // Storage is owned in-place: `storage[4]` holds the SimplexVertex
    // values, `vertex[]` holds pointers into `storage[]`. Reductions
    // shuffle pointers; `add_to_simplex` writes into a free slot.
    struct Simplex
    {
        SimplexVertex   storage[4];
        SimplexVertex  *vertex[4];
        int             rank;

        inline void reset()
        {
            rank = 0;
            for (int i = 0; i < 4; ++i)
                vertex[i] = &storage[i];
        }

        // Slot in storage[] not currently referenced by vertex[0..rank-1].
        inline SimplexVertex *unused_slot()
        {
            for (int i = 0; i < 4; ++i)
            {
                bool used = false;
                for (int j = 0; j < rank; ++j)
                    if (vertex[j] == &storage[i]) { used = true; break; }
                if (!used) return &storage[i];
            }
            return &storage[0];
        }
    };
} // namespace rbc
