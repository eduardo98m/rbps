#include "rbps/constraints/Contact.hpp"

// ============================================================================
//  Contact.cpp  —  XPBD position-level and velocity-level contact solver.
//
//  Key fixes vs previous version:
//
//   1. Capture pre-solve relative_velocity BEFORE calling solve_normal_constraint
//      so restitution uses the actual pre-correction normal velocity.
//
//   2. Tangent Coulomb cone uses the correct sign convention:
//      lambdas are negative (impulse opposes penetration / slip), so the
//      cone inequality is  λ_t >= μ_s * λ_n  (both sides negative, λ_n < 0).
//
//   3. Velocity-level dynamic friction magnitude = μ_d * |λ_n| / h, NOT
//      μ_d * λ_n (which is negative).  Clamped to |v_t| so it cannot
//      reverse the sliding direction.
//
//   4. Velocity-level friction is always computed when collision is active —
//      the use_dynamic_friction gate only determined WHICH coefficient to use,
//      not whether to apply friction at all.  Now both static and dynamic
//      contacts receive velocity-level friction.
//
//   5. The combined velocity impulse is decomposed so normal and tangential
//      components each use the correct generalized inverse mass.
// ============================================================================

namespace rbps
{

    // ============================================================================
    //  Position-level entry point
    // ============================================================================

    void apply_constraint_position_level(ContactList &cl,
                                         uint32_t i,
                                         BodyCollection &bc,
                                         m3d::scalar inv_h)
    {
        const uint32_t ba = cl.body_a[i];
        const uint32_t bb = cl.body_b[i];

        // Reconstruct world-space contact points from body-local lever arms
        // and current body transforms — eq. (26), Müller et al. 2020.
        const m3d::vec3 p_a = bc.position[ba] + m3d::rotate(bc.orientation[ba], cl.r_a_local[i]);
        const m3d::vec3 p_b = bc.position[bb] + m3d::rotate(bc.orientation[bb], cl.r_b_local[i]);

        cl.point_on_a[i] = p_a;
        cl.point_on_b[i] = p_b;

        // Signed penetration depth along the contact normal.
        // Positive → still penetrating.
        const m3d::scalar d = m3d::dot(p_a - p_b, cl.normal[i]);
        if (d <= 0.0)
        {
            cl.collision[i] = false;
            return;
        }
        cl.collision[i] = true;

        const m3d::vec3 r_a_wc = p_a - bc.position[ba];
        const m3d::vec3 r_b_wc = p_b - bc.position[bb];

        // ── FIX 1: capture pre-solve velocity BEFORE applying the normal impulse ──
        // This is the velocity that restitution must "reflect".
        {
            const m3d::vec3 v_rel =
                (bc.linear_velocity[ba] + m3d::cross(bc.angular_velocity[ba], r_a_wc)) -
                (bc.linear_velocity[bb] + m3d::cross(bc.angular_velocity[bb], r_b_wc));
            cl.relative_velocity[i] = m3d::dot(v_rel, cl.normal[i]);
        }

        solve_normal_constraint(cl, i, bc, inv_h, d, r_a_wc, r_b_wc);
        solve_tangent_constraint(cl, i, bc, inv_h);
    }

    // ============================================================================
    //  Normal (non-penetration) constraint
    // ============================================================================

    void solve_normal_constraint(ContactList &cl,
                                 uint32_t i,
                                 BodyCollection &bc,
                                 m3d::scalar inv_h,
                                 m3d::scalar penetration,
                                 m3d::vec3 r_a_wc,
                                 m3d::vec3 r_b_wc)
    {
        const uint32_t ba = cl.body_a[i];
        const uint32_t bb = cl.body_b[i];

        const m3d::scalar w_a = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, cl.normal[i]);
        const m3d::scalar w_b = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, cl.normal[i]);

        const m3d::scalar denom = w_a + w_b;
        if (denom < EPSILON)
            return; // both static — nothing to do

        // XPBD position-level normal impulse (α = 0 for hard contact):
        //   Δλ = (−C − α·λ) / (w_a + w_b + α)
        // C = penetration depth (positive), so Δλ is negative.
        const m3d::scalar delta_lambda = -penetration / denom;

        cl.normal_lambda[i] += delta_lambda;

        // Clamp: normal lambda must be ≤ 0 (can only push, never pull).
        if (cl.normal_lambda[i] > 0.0)
            cl.normal_lambda[i] = 0.0;

        const m3d::vec3 impulse = delta_lambda * cl.normal[i];
        cl.normal_force[i] = cl.normal[i] * cl.normal_lambda[i] * inv_h * inv_h;

        apply_positional_constraint_impulse(bc, ba, impulse, r_a_wc);
        apply_positional_constraint_impulse(bc, bb, -impulse, r_b_wc);
    }

    // ============================================================================
    //  Tangential (friction) constraint
    // ============================================================================

    void solve_tangent_constraint(ContactList &cl,
                                  uint32_t i,
                                  BodyCollection &bc,
                                  m3d::scalar inv_h)
    {
        const uint32_t ba = cl.body_a[i];
        const uint32_t bb = cl.body_b[i];

        // Recompute lever arms from current (post-normal-correction) state.
        const m3d::vec3 r_a_wc = m3d::rotate(bc.orientation[ba], cl.r_a_local[i]);
        const m3d::vec3 r_b_wc = m3d::rotate(bc.orientation[bb], cl.r_b_local[i]);
        const m3d::vec3 p_a_cur = bc.position[ba] + r_a_wc;
        const m3d::vec3 p_b_cur = bc.position[bb] + r_b_wc;

        cl.point_on_a[i] = p_a_cur;
        cl.point_on_b[i] = p_b_cur;

        // Previous-substep contact points — eq. (26) with prev transforms.
        const m3d::vec3 prev_pa =
            bc.prev_position[ba] + m3d::rotate(bc.prev_orientation[ba], cl.r_a_local[i]);
        const m3d::vec3 prev_pb =
            bc.prev_position[bb] + m3d::rotate(bc.prev_orientation[bb], cl.r_b_local[i]);

        // Relative tangential displacement — eq. (27)/(28).
        const m3d::vec3 delta_p = (p_a_cur - prev_pa) - (p_b_cur - prev_pb);
        const m3d::vec3 delta_p_t = delta_p - m3d::dot(delta_p, cl.normal[i]) * cl.normal[i];

        const m3d::scalar slip = m3d::magnitude(delta_p_t);
        if (slip < EPSILON)
            return;

        const m3d::vec3 t = m3d::normalize(delta_p_t);

        const m3d::scalar w_a = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, t);
        const m3d::scalar w_b = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, t);

        const m3d::scalar denom = w_a + w_b;
        if (denom < EPSILON)
            return;

        // Unclamped tangential impulse magnitude (negative — opposes slip).
        const m3d::scalar delta_lambda = -slip / denom;
        const m3d::scalar old_lambda = cl.tangent_lambda[i];
        cl.tangent_lambda[i] += delta_lambda;

        // ── FIX 2: Coulomb cone with correct sign convention ─────────────────────
        // λ_n ≤ 0  and  λ_t ≤ 0.
        // Static friction holds while:   λ_t  ≥  μ_s * λ_n
        //                         i.e.: |λ_t| ≤  μ_s * |λ_n|
        // When exceeded, clamp to the cone boundary.
        const m3d::scalar cone_limit = cl.static_friction[i] * cl.normal_lambda[i];
        // cone_limit is ≤ 0 (μ_s * λ_n, both negative).

        if (cl.tangent_lambda[i] < cone_limit)
        {
            // Exceeded static friction — clamp to cone boundary.
            cl.tangent_lambda[i] = cone_limit;
            cl.use_dynamic_friction[i] = true;
        }
        else
        {
            cl.use_dynamic_friction[i] = true;
        }

        // Apply the delta that was actually used (after clamping).
        const m3d::scalar actual_delta = cl.tangent_lambda[i] - old_lambda;
        if (m3d::abs(actual_delta) < EPSILON)
            return;

        const m3d::vec3 impulse = actual_delta * t;
        cl.tangent_force[i] = t * cl.tangent_lambda[i] * inv_h * inv_h;

        apply_positional_constraint_impulse(bc, ba, impulse, r_a_wc);
        apply_positional_constraint_impulse(bc, bb, -impulse, r_b_wc);
    }

    // ============================================================================
    //  Velocity-level solve (restitution + dynamic friction)
    //
    //  Called once per substep AFTER the position solve and velocity update.
    //  Handles:
    //    • Restitution along the normal (bounce).
    //    • Dynamic Coulomb friction in the tangential plane.
    // ============================================================================

    void apply_constraint_velocity_level(ContactList &cl,
                                         uint32_t i,
                                         BodyCollection &bc,
                                         m3d::scalar h)
    {
        if (!cl.collision[i])
            return;
        // if (cl.normal_lambda[i] >= -EPSILON)
        //     return;

        const uint32_t ba = cl.body_a[i];
        const uint32_t bb = cl.body_b[i];

        const m3d::vec3 r_a_wc = cl.point_on_a[i] - bc.position[ba];
        const m3d::vec3 r_b_wc = cl.point_on_b[i] - bc.position[bb];

        const m3d::vec3 v_rel =
            (bc.linear_velocity[ba] + m3d::cross(bc.angular_velocity[ba], r_a_wc)) -
            (bc.linear_velocity[bb] + m3d::cross(bc.angular_velocity[bb], r_b_wc));

        const m3d::scalar v_n = m3d::dot(v_rel, cl.normal[i]);
        const m3d::vec3 v_t = v_rel - cl.normal[i] * v_n;
        const m3d::scalar v_t_mag = m3d::magnitude(v_t);

        // ── Restitution ───────────────────────────────────────────────────────────
        // Damp for slow impacts (resting contact threshold = 2g·h).
        m3d::scalar e = cl.restitution[i];
        if (m3d::abs(cl.relative_velocity[i]) <= 2.0 * 9.8 * h)
            e = 0.0;

        // Target normal velocity after bounce:
        //   v_n_target = max(−e * v_n_pre, 0) so we never pull bodies together.
        const m3d::scalar v_n_target = m3d::min(-e * cl.relative_velocity[i], 0.0);

        // Normal velocity correction needed:  Δv_n = v_n_target − v_n_current
        // (will be negative when we need to push bodies apart / kill approach)
        const m3d::scalar delta_v_n = v_n_target - v_n;

        // ── FIX 3 & 4: Dynamic friction magnitude = μ_d * |λ_n| / h ─────────────
        // λ_n is the accumulated normal impulse (force × h²), so
        //   normal force magnitude  = |λ_n| * inv_h²
        //   friction impulse limit  = μ_d * |λ_n| / h   (has units of impulse)
        // We apply friction regardless of whether static/dynamic was active at
        // position level; velocity-level friction always uses the dynamic coeff.
        m3d::scalar delta_v_t_mag = 0.0;
        if (v_t_mag > EPSILON)
        {
            // ── FIX 3: use |λ_n|, not λ_n (which is negative) ───────────────────
            const m3d::scalar friction_limit =
                cl.dynamic_friction[i] * m3d::abs(cl.normal_lambda[i]) / h;

            // Clamp to [0, v_t_mag] — cannot reverse sliding direction.
            delta_v_t_mag = -m3d::min(friction_limit, v_t_mag);
        }

        // ── Apply normal and tangential corrections separately ───────────────────
        // Normal correction: use normal direction for generalized inverse mass.
        const m3d::scalar w_a_n = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, cl.normal[i]);
        const m3d::scalar w_b_n = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, cl.normal[i]);
        const m3d::scalar denom_n = w_a_n + w_b_n;

        if (denom_n > EPSILON && m3d::abs(delta_v_n) > EPSILON)
        {
            const m3d::vec3 impulse_n = (delta_v_n / denom_n) * cl.normal[i];
            apply_positional_velocity_constraint_impulse(bc, ba, impulse_n, r_a_wc);
            apply_positional_velocity_constraint_impulse(bc, bb, -impulse_n, r_b_wc);
        }

        // Tangential correction: use tangent direction for generalized inverse mass.
        if (v_t_mag > EPSILON && m3d::abs(delta_v_t_mag) > EPSILON)
        {
            const m3d::vec3 t = v_t / v_t_mag;
            const m3d::scalar w_a_t = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, t);
            const m3d::scalar w_b_t = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, t);
            const m3d::scalar denom_t = w_a_t + w_b_t;

            if (denom_t > EPSILON)
            {
                const m3d::vec3 impulse_t = (delta_v_t_mag / denom_t) * t;
                apply_positional_velocity_constraint_impulse(bc, ba, impulse_t, r_a_wc);
                apply_positional_velocity_constraint_impulse(bc, bb, -impulse_t, r_b_wc);
            }
        }
    }

    // ============================================================================
    //  Batch wrappers
    // ============================================================================

    void solve_contacts_position_level(ContactList &cl,
                                       BodyCollection &bc,
                                       m3d::scalar inv_h)
    {
        for (uint32_t i = 0; i < cl.n_contacts; ++i)
            apply_constraint_position_level(cl, i, bc, inv_h);
    }

    void solve_contacts_velocity_level(ContactList &cl,
                                       BodyCollection &bc,
                                       m3d::scalar h)
    {
        for (uint32_t i = 0; i < cl.n_contacts; ++i)
            apply_constraint_velocity_level(cl, i, bc, h);
    }

} // namespace rbps