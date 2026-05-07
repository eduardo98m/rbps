#include "rbc/gjk/simplex/DoSimplex.hpp"

namespace rbc
{
    void add_to_simplex(Simplex &s, const SimplexVertex &v)
    {
        SimplexVertex *slot = s.unused_slot();
        *slot = v;

        const int n = (s.rank < 3) ? s.rank : 3;
        for (int i = n; i > 0; --i)
            s.vertex[i] = s.vertex[i - 1];
        s.vertex[0] = slot;

        if (s.rank < 4) ++s.rank;
    }

    bool do_simplex(Simplex &s, m3d::vec3 &direction)
    {
        switch (s.rank)
        {
            case 2: return do_simplex_2(s, direction);
            case 3: return do_simplex_3(s, direction);
            case 4: return do_simplex_4(s, direction);
        }
        return false;
    }
} // namespace rbc
