#include "rbps/constraints/Contact.hpp"

// ============================================================================
//  Contact.cpp
//
//  Field mapping from old ContactCollection → new ContactList
//  ─────────────────────────────────────────────────────────
//  body_1 / body_2                          → body_a / body_b
//  p_1    / p_2                             → point_on_a / point_on_b
//  normal_constraint_lagrange_multiplier    → normal_lambda
//  tangencial_constraint_lagrange_multiplier→ tangent_lambda
//  tangencial_force                         → tangent_force
//  (penetration_depth now stored explicitly on ContactList)
//
//  Body indices
//  ─────────────────────────────────────────────────────────
//  cl.body_a[i] / cl.body_b[i] are BodyCollection *slot indices* (not user
//  IDs).  They are safe to use directly as bc.position[body_a[i]].
// ============================================================================

namespace rbps
{

// ============================================================================
//  Position-level entry point
// ============================================================================

void apply_constraint_position_level(ContactList    &cl,
                                     uint32_t        i,
                                     BodyCollection &bc,
                                     m3d::scalar     inv_h)
{
    const uint32_t ba = cl.body_a[i];
    const uint32_t bb = cl.body_b[i];
 
    // Reconstruct world-space contact points from body-local lever arms
    // and the bodies' CURRENT transforms — eq. (26), Müller et al. 2020.
    //   p = x_body + rotate(q_body, r_local)
    // This correctly tracks both translation and rotation of the contact
    // surface between substeps, unlike a frozen world-space snapshot.
    const m3d::vec3 p_a = bc.position[ba] + m3d::rotate(bc.orientation[ba], cl.r_a_local[i]);
    const m3d::vec3 p_b = bc.position[bb] + m3d::rotate(bc.orientation[bb], cl.r_b_local[i]);
 
    // Write back so the tangent solver and velocity-level solver see the
    // same up-to-date points.
    cl.point_on_a[i] = p_a;
    cl.point_on_b[i] = p_b;
 
    // Signed penetration depth along the (fixed) contact normal.
    // Positive → still penetrating; negative → separated, skip.
    const m3d::scalar d = m3d::dot(p_a - p_b, cl.normal[i]);
    if (d <= 0.0)
    {
        cl.collision[i] = false;
        return;
    }
    cl.collision[i] = true;

    // World-space lever arms for this substep (== rotate(q, r_local)).
    const m3d::vec3 r_a_wc = p_a - bc.position[ba];
    const m3d::vec3 r_b_wc = p_b - bc.position[bb];

    // Capture pre-solve normal relative velocity for restitution term.
    solve_normal_constraint(cl, i, bc, inv_h, d, r_a_wc, r_b_wc);

    const m3d::vec3 v_rel =
        (bc.linear_velocity[ba] + m3d::cross(bc.angular_velocity[ba], r_a_wc)) -
        (bc.linear_velocity[bb] + m3d::cross(bc.angular_velocity[bb], r_b_wc));
    cl.relative_velocity[i] = m3d::dot(v_rel, cl.normal[i]);
 
    solve_tangent_constraint(cl, i, bc, inv_h); // If commented the behaviour is stable
}

// ============================================================================
//  Normal constraint
// ============================================================================

void solve_normal_constraint(ContactList    &cl,
                             uint32_t        i,
                             BodyCollection &bc,
                             m3d::scalar     inv_h,
                             m3d::scalar     penetration,
                             m3d::vec3       r_a_wc,
                             m3d::vec3       r_b_wc)
{
    const uint32_t ba = cl.body_a[i];
    const uint32_t bb = cl.body_b[i];

    const m3d::scalar w_a = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, cl.normal[i]);
    const m3d::scalar w_b = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, cl.normal[i]);

    // α = 0 for hard contacts; leave the slot for soft contacts later.
    constexpr m3d::scalar alpha = 0.0;

    const m3d::scalar delta_lambda =
        (-penetration - alpha * cl.normal_lambda[i]) / (w_a + w_b + alpha);

    cl.normal_lambda[i] += delta_lambda;

    const m3d::vec3 impulse = delta_lambda * cl.normal[i];

    // Force for debug / force-feedback readout (λ * n / h²)
    cl.normal_force[i] = cl.normal[i] * cl.normal_lambda[i] * inv_h * inv_h;

    apply_positional_constraint_impulse(bc, ba,  impulse, r_a_wc);
    apply_positional_constraint_impulse(bc, bb, -impulse, r_b_wc);
}

// ============================================================================
//  Tangential (friction) constraint
// ============================================================================

void solve_tangent_constraint(ContactList    &cl,
                              uint32_t        i,
                              BodyCollection &bc,
                              m3d::scalar     inv_h)
{
    const uint32_t ba = cl.body_a[i];
    const uint32_t bb = cl.body_b[i];

    // Recompute from CURRENT (post-normal-correction) body state.
    // r_a_wc = rotate(q, r_local) — body position cancels out.
    const m3d::vec3 r_a_wc     = m3d::rotate(bc.orientation[ba], cl.r_a_local[i]);
    const m3d::vec3 r_b_wc     = m3d::rotate(bc.orientation[bb], cl.r_b_local[i]);
    const m3d::vec3 p_a_cur    = bc.position[ba] + r_a_wc;
    const m3d::vec3 p_b_cur    = bc.position[bb] + r_b_wc;

    // Update stored contact points so velocity solver sees post-normal points.
    cl.point_on_a[i] = p_a_cur;
    cl.point_on_b[i] = p_b_cur;

    // Previous-substep contact points — eq. (26): p̄ = x_prev + q_prev * r_local
    const m3d::vec3 prev_pa = bc.prev_position[ba] + m3d::rotate(bc.prev_orientation[ba], cl.r_a_local[i]);
    const m3d::vec3 prev_pb = bc.prev_position[bb] + m3d::rotate(bc.prev_orientation[bb], cl.r_b_local[i]);

    // Relative displacement projected onto tangent plane — eq. (27), (28).
    const m3d::vec3 delta_p   = (p_a_cur - prev_pa) - (p_b_cur - prev_pb);
    const m3d::vec3 delta_p_t = delta_p - m3d::dot(delta_p, cl.normal[i]) * cl.normal[i];

    const m3d::scalar slip = m3d::magnitude(delta_p_t);
    if (slip < EPSILON)
        return;

    const m3d::vec3 t = m3d::normalize(delta_p_t);

    const m3d::scalar w_a = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, t);
    const m3d::scalar w_b = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, t);

    const m3d::scalar delta_lambda    = -slip / (w_a + w_b);
    const m3d::scalar old_tangent_lambda = cl.tangent_lambda[i];
    cl.tangent_lambda[i] += delta_lambda;

    // Coulomb cone — lambdas are negative so inequality is flipped vs. paper.
    if (cl.tangent_lambda[i] < cl.normal_lambda[i] * cl.static_friction[i])
    {
        cl.tangent_lambda[i]       = cl.normal_lambda[i] * cl.static_friction[i];
        cl.use_dynamic_friction[i] = true;
        return;
    }

    cl.use_dynamic_friction[i]       = false;
    const m3d::scalar actual_delta   = cl.tangent_lambda[i] - old_tangent_lambda;
    const m3d::vec3   impulse        = actual_delta * t;
    cl.tangent_force[i]              = t * cl.tangent_lambda[i] * inv_h * inv_h;

    apply_positional_constraint_impulse(bc, ba,  impulse, r_a_wc);
    apply_positional_constraint_impulse(bc, bb, -impulse, r_b_wc);
}

// ============================================================================
//  Velocity-level solve (restitution + dynamic friction)
// ============================================================================

void apply_constraint_velocity_level(ContactList    &cl,
                                     uint32_t        i,
                                     BodyCollection &bc,
                                     m3d::scalar     h)
{
    if (!cl.collision[i])
        return;

    const uint32_t ba = cl.body_a[i];
    const uint32_t bb = cl.body_b[i];

    const m3d::vec3 r_a_wc = cl.point_on_a[i] - bc.position[ba];
    const m3d::vec3 r_b_wc = cl.point_on_b[i] - bc.position[bb];

    const m3d::vec3 v_rel =
        (bc.linear_velocity[ba] + m3d::cross(bc.angular_velocity[ba], r_a_wc)) -
        (bc.linear_velocity[bb] + m3d::cross(bc.angular_velocity[bb], r_b_wc));

    const m3d::scalar v_n = m3d::dot(v_rel, cl.normal[i]);
    const m3d::vec3   v_t = v_rel - cl.normal[i] * v_n;

    m3d::vec3 delta_v = {0.0, 0.0, 0.0};

    // ── Dynamic Coulomb friction (eq. 30) ────────────────────────────────────
    // friction = min(μ_d * |f_n| * h, |v_t|)  where f_n = λ_n / h²
    //          = min(μ_d * |λ_n| / h, |v_t|)
    if (m3d::magnitude(v_t) > EPSILON && cl.use_dynamic_friction[i])
    {
        const m3d::scalar friction_mag =
            m3d::min(- cl.dynamic_friction[i] * cl.normal_lambda[i],
                     m3d::magnitude(v_t));

        delta_v += -m3d::normalize(v_t) * friction_mag ;   // ← was commented out!
    }

    // ── Restitution (eq. 34) ─────────────────────────────────────────────────
    // Damp restitution for slow impacts (resting contact: threshold = 2g·h).
    m3d::scalar e = cl.restitution[i];
    if (m3d::abs(v_n) <= 2.0 * 9.8 * h)
        e = 0.0;

    // relative_velocity[i] = v̄_n captured before velocity update (pre-solve normal vel).
    delta_v += cl.normal[i] * (-v_n + m3d::min(-e * cl.relative_velocity[i], 0.0));

    if (m3d::magnitude(delta_v) < EPSILON)
        return;

    // ── Apply combined impulse ───────────────────────────────────────────────
    // Use contact NORMAL for generalized inverse mass, not the combined delta_v
    // direction. This matches the reference implementation and is stable because
    // the normal gives the correct "worst case" inertia for both components.
    const m3d::scalar w_a = get_positional_generalized_inverse_mass(bc, ba, r_a_wc, cl.normal[i]);
    const m3d::scalar w_b = get_positional_generalized_inverse_mass(bc, bb, r_b_wc, cl.normal[i]);

    const m3d::vec3 impulse = delta_v / (w_a + w_b);

    apply_positional_velocity_constraint_impulse(bc, ba,  impulse, r_a_wc);
    apply_positional_velocity_constraint_impulse(bc, bb, -impulse, r_b_wc);
}

// ============================================================================
//  Batch wrappers
// ============================================================================

void solve_contacts_position_level(ContactList    &cl,
                                   BodyCollection &bc,
                                   m3d::scalar     inv_h)
{
    for (uint32_t i = 0; i < cl.n_contacts; ++i)
        apply_constraint_position_level(cl, i, bc, inv_h);
}

void solve_contacts_velocity_level(ContactList    &cl,
                                   BodyCollection &bc,
                                   m3d::scalar     h)
{
    for (uint32_t i = 0; i < cl.n_contacts; ++i)
        apply_constraint_velocity_level(cl, i, bc, h);
}

} // namespace rbps