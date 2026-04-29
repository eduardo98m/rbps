#pragma once
#include "rbps/constraints/Constraint.hpp"
#include "rbps/constraints/Contact.hpp"
#include "rbps/constraints/Joint.hpp"
#include <limits>
#include <vector>

// ── Constraint / Contact / Joint test helpers ──────────────────────────────
// Replaces:
//   - create_constraints() in test_constraint.cpp
//   - create_constraint_collection() and add_test_constraints() in test_joint.cpp
//   - create_contact_list() in test_contact.cpp
//   - create_prismatic_joint / create_revolute_joint / create_fixed_joint in test_joint.cpp
//
// Mutation-style: caller supplies the collection, helper appends to it.
// Joint helpers wrap the raw SoA (matches the unit-test layer; the public
// API is exercised separately by tests/rbps/API/test_jointAPI.cpp).
namespace test
{
    // ── ConstraintCollection ────────────────────────────────────────────────
    inline uint32_t add_default_constraint(rbps::ConstraintCollection &cc)
    {
        const uint32_t id = cc.add();
        const int32_t  i  = cc.index_of(id);
        cc.body_1[i]      = 0;
        cc.body_2[i]      = 0;
        cc.r_1[i]         = m3d::vec3(0, 0, 0);
        cc.r_2[i]         = m3d::vec3(0, 0, 0);
        cc.direction[i]   = m3d::vec3(1, 0, 0);
        cc.magnitude[i]   = 0.0;
        cc.lambda[i]      = 0.0;
        cc.force[i]       = m3d::vec3(0, 0, 0);
        cc.torque[i]      = m3d::vec3(0, 0, 0);
        cc.compliance[i]  = 0.0;
        cc.type[i]        = rbps::ConstraintType::POSITIONAL;
        cc.impulse[i]     = m3d::vec3(0, 0, 0);
        return id;
    }

    inline void init_test_constraints(rbps::ConstraintCollection &cc, uint32_t n)
    {
        for (uint32_t k = 0; k < n; ++k)
            (void)add_default_constraint(cc);
    }

    // Same, but returns a vector of stable IDs in insertion order. Used by
    // tests that need to look up entries via index_of() after the collection
    // has been mutated.
    inline std::vector<uint32_t> add_test_constraints(rbps::ConstraintCollection &cc,
                                                     uint32_t n)
    {
        std::vector<uint32_t> ids;
        ids.reserve(n);
        for (uint32_t k = 0; k < n; ++k)
            ids.push_back(add_default_constraint(cc));
        return ids;
    }

    // ── ContactList ─────────────────────────────────────────────────────────
    // Pre-populated ContactList with n contacts. body_a defaults to slot 0,
    // body_b to slot 1 (matches the test_contact.cpp local helper).
    inline rbps::ContactList make_contact_list(size_t n)
    {
        rbps::ContactList cl;
        cl.n_contacts = static_cast<uint32_t>(n);
        cl.body_a              .resize(n, 0);
        cl.body_b              .resize(n, 1);
        cl.collider_a          .resize(n, 0);
        cl.collider_b          .resize(n, 1);
        cl.r_a_local           .resize(n, m3d::vec3(0, 0, 0));
        cl.r_b_local           .resize(n, m3d::vec3(0, 0, 0));
        cl.normal              .resize(n, m3d::vec3(0, 1, 0));
        cl.point_on_a          .resize(n, m3d::vec3(0, 0, 0));
        cl.point_on_b          .resize(n, m3d::vec3(0, 0, 0));
        cl.penetration_depth   .resize(n, 0.0);
        cl.static_friction     .resize(n, 0.5);
        cl.dynamic_friction    .resize(n, 0.3);
        cl.restitution         .resize(n, 0.5);
        cl.collision           .resize(n, false);
        cl.relative_velocity   .resize(n, 0.0);
        cl.normal_lambda       .resize(n, 0.0);
        cl.tangent_lambda      .resize(n, 0.0);
        cl.normal_force        .resize(n, m3d::vec3(0, 0, 0));
        cl.tangent_force       .resize(n, m3d::vec3(0, 0, 0));
        cl.use_dynamic_friction.resize(n, false);
        return cl;
    }

    // ── Joint factories (raw SoA) ───────────────────────────────────────────
    // Wrappers around direct JointCollection field assignment. The public API
    // equivalents live in rbps/API and are tested separately in
    // tests/rbps/API/test_jointAPI.cpp; these helpers are for the unit tests
    // that want to bypass the API and exercise SoA mechanics directly.

    inline uint32_t add_prismatic_joint(rbps::JointCollection &jc,
                                        uint32_t body1, uint32_t body2,
                                        const m3d::vec3 &axis)
    {
        const uint32_t jid = jc.add();
        const uint32_t i   = jc.index_of(jid);
        jc.type[i]               = rbps::PRISMATIC;
        jc.body_1[i]             = body1;
        jc.body_2[i]             = body2;
        jc.main_axis[i]          = axis;
        jc.target_position[i]    = 0.0;
        jc.target_speed[i]       = 0.0;
        jc.lower_limit[i]        = -std::numeric_limits<double>::infinity();
        jc.upper_limit[i]        =  std::numeric_limits<double>::infinity();
        jc.constraint_count[i]   = 3; // 1 rotation lock + 2 positional
        return jid;
    }

    inline uint32_t add_revolute_joint(rbps::JointCollection &jc,
                                       uint32_t body1, uint32_t body2,
                                       const m3d::vec3 &axis)
    {
        const uint32_t jid = jc.add();
        const uint32_t i   = jc.index_of(jid);
        jc.type[i]               = rbps::REVOLUTE;
        jc.body_1[i]             = body1;
        jc.body_2[i]             = body2;
        jc.main_axis[i]          = axis;
        jc.target_position[i]    = 0.0;
        jc.target_speed[i]       = 0.0;
        jc.lower_limit[i]        = -std::numeric_limits<double>::infinity();
        jc.upper_limit[i]        =  std::numeric_limits<double>::infinity();
        jc.constraint_count[i]   = 4; // 2 rotation lock + 2 positional
        return jid;
    }

    inline uint32_t add_fixed_joint(rbps::JointCollection &jc,
                                    uint32_t body1, uint32_t body2)
    {
        const uint32_t jid = jc.add();
        const uint32_t i   = jc.index_of(jid);
        jc.type[i]               = rbps::FIXED;
        jc.body_1[i]             = body1;
        jc.body_2[i]             = body2;
        jc.constraint_count[i]   = 2; // 1 rotational + 1 positional full lock
        return jid;
    }
} // namespace test
