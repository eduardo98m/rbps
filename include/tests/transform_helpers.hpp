#pragma once
#include <math3d/math3d.hpp>

// ── Transform shortcuts ────────────────────────────────────────────────────
// Used by both the rbc/ collision tests (Shape vs Shape with explicit
// transforms) and the rbps/ pipeline tests (collider creation needs an
// initial world tf). Lives at the tests/ root so both domains can include
// it without pulling in unrelated rbc/rbps dependencies.
namespace test
{
    inline m3d::tf tf_at(m3d::scalar x, m3d::scalar y, m3d::scalar z)
    {
        m3d::tf t;
        t.pos = m3d::vec3(x, y, z);
        return t;
    }
    inline m3d::tf tf_at(const m3d::vec3 &p)
    {
        m3d::tf t;
        t.pos = p;
        return t;
    }
} // namespace test
