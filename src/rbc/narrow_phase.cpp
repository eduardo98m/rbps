#include "rbc/narrow_phase.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"

namespace rbc
{

    bool test_narrow_phase(const CollisionShape *shape_a, const m3d::tf &tf_a,
                           const CollisionShape *shape_b, const m3d::tf &tf_b,
                           rbc::Contact &out_contact)
    {
        // 1. Setup the Minkowski Difference wrapper
        MinkowskiDiff md(shape_a, shape_b, tf_a, tf_b);

        // 2. Initial guess (vector from center of A to center of B)
        m3d::vec3 guess = tf_b.pos - tf_a.pos;

        // If centers perfectly overlap, provide a default arbitrary axis
        if (m3d::length_sq(guess) < m3d::EPSILON)
        {
            guess = m3d::vec3(1.0, 0.0, 0.0);
        }

        // 3. Evaluate GJK
        GJK gjk_solver;
        GJK::Status gjk_status = gjk_solver.evaluate(md, guess);

        if (gjk_status == GJK::Inside)
        {
            // Shapes are overlapping! Use EPA to find penetration depth and normal
            EPA epa_solver;
            EPA::Status epa_status = epa_solver.evaluate(gjk_solver, md);

            if (epa_status == EPA::Valid)
            {
                // Populate the contact manifold constraint
                out_contact.normal = epa_solver.normal;
                out_contact.penetration_depth = epa_solver.depth;
                out_contact.pos = epa_solver.contact_point;
                return true;
            }
        }

        return false; // No collision or failed to resolve
    }

} // namespace rbc