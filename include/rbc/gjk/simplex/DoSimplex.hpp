#pragma once
#include <math3d/math3d.hpp>
#include "rbc/gjk/simplex/Simplex.hpp"

namespace rbc
{
    // Push a new support onto the simplex. `vertex[0]` becomes the
    // newest entry; older entries shift right. Rank caps at 4.
    void add_to_simplex(Simplex &s, const SimplexVertex &v);

    // Reduce the simplex to the sub-simplex containing the origin's
    // closest point and write the next search direction (toward the
    // origin) into `direction`. Returns true when the simplex
    // encloses the origin (rank-4 case only).
    bool do_simplex(Simplex &s, m3d::vec3 &direction);

    // Per-rank handlers. Public so the dispatcher can forward; not
    // intended to be called from outside the GJK module.
    bool do_simplex_2(Simplex &s, m3d::vec3 &direction);
    bool do_simplex_3(Simplex &s, m3d::vec3 &direction);
    bool do_simplex_4(Simplex &s, m3d::vec3 &direction);
} // namespace rbc
