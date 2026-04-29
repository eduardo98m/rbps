#pragma once
#include "rbps/Body.hpp"

// ── BodyCollection test helpers ─────────────────────────────────────────────
// Replaces the 14-field create_test_bodies() / create_bodies() factory
// that was copy-pasted across 4 test files (test_body, test_constraint,
// test_joint, test_contact).
//
// Mutation-style: takes an existing BodyCollection by reference and adds to
// it. Returns the index of the newly-added body (or void for the bulk init).
// Tests can override individual fields after the call.
namespace test
{
    // Add one default-initialised dynamic body to bc.
    //   mass = inverse_mass = 1.0
    //   type = DYNAMIC
    //   position, velocity, angular_velocity = (0,0,0)
    //   orientation = prev_orientation = identity
    //   inertia_tensor (and the inverse / world variants) = identity smat3
    inline int32_t add_default_body(rbps::BodyCollection &bc)
    {
        const int32_t i = bc.index_of(bc.add());
        bc.mass[i]                          = 1.0;
        bc.inverse_mass[i]                  = 1.0;
        bc.type[i]                          = rbps::BodyType::DYNAMIC;
        bc.position[i]                      = m3d::vec3(0, 0, 0);
        bc.linear_velocity[i]               = m3d::vec3(0, 0, 0);
        bc.orientation[i]                   = m3d::quat(1, 0, 0, 0);
        bc.angular_velocity[i]              = m3d::vec3(0, 0, 0);
        bc.prev_orientation[i]              = m3d::quat(1, 0, 0, 0);
        bc.inertia_tensor[i]                = m3d::smat3(1, 1, 1, 0, 0, 0);
        bc.inverse_inertia_tensor[i]        = m3d::smat3(1, 1, 1, 0, 0, 0);
        bc.inertia_tensor_world[i]          = m3d::smat3(1, 1, 1, 0, 0, 0);
        bc.inverse_inertia_tensor_world[i]  = m3d::smat3(1, 1, 1, 0, 0, 0);
        return i;
    }

    // Convenience: add `n` default bodies in one call. Replaces the
    // create_test_bodies(n) / create_bodies(n) factories. Returns void —
    // tests typically use bc[0..n-1] directly.
    inline void init_test_bodies(rbps::BodyCollection &bc, uint32_t n)
    {
        for (uint32_t k = 0; k < n; ++k)
            add_default_body(bc);
    }
} // namespace test
