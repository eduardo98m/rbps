#pragma once
#include <raylib.h>
#include <rlgl.h>

// ── Raylib defines PI as a plain macro which stomps m3d::PI (a constexpr).
//    Undefine it here, before any math3d header is pulled in transitively.
#ifdef PI
#  undef PI
#endif

#include <variant>
#include <cmath>
#include "visr/Snapshot.hpp"    // pulls in math3d — must come AFTER the undef

/**
 * @file RaylibDraw.hpp
 * @brief Stateless draw helpers — convert every `*Snap` to raylib draw calls.
 * @ingroup visr
 *
 * @par Colour scheme
 * - Dynamic wireframe — `SKYBLUE`
 * - Static  wireframe — `GRAY`
 * - Selected body     — `GREEN`
 * - Contact point A   — `BLUE` sphere on the surface of body A
 * - Contact point B   — `RED`  sphere on the surface of body B
 * - Contact normal    — gradient arrow (depth-coded green → red)
 * - Contact separation — `MAGENTA` line between `p_a` and `p_b`
 * - Joint anchor      — `YELLOW` sphere + connector line
 * - Joint axis        — `PURPLE` arrow
 * - Velocity vector   — `LIME` arrow
 * - Angular velocity  — `ORANGE` arc (approximated as arrow)
 *
 * Top-level entry point is `draw_scene` (defined further down).
 */

namespace visr::draw
{
    // ── math3d → raylib (explicit casts: m3d::scalar is double, Raylib uses float) ──
    static inline Vector3 to_rl(const m3d::vec3 &v)
    {
        return { (float)v.x, (float)v.y, (float)v.z };
    }

    // Quaternion → Raylib axis-angle (degrees) for rlRotatef
    static inline void quat_to_axis_angle(const m3d::quat &q,
                                          Vector3 &axis_out,
                                          float   &angle_deg_out)
    {
        float w = (float)q.w;
        if      (w >  1.0f) w =  1.0f;
        else if (w < -1.0f) w = -1.0f;

        const float angle_rad = 2.0f * std::acos(w);
        const float s         = std::sqrt(std::max(0.0f, 1.0f - w * w));

        if (s < 1e-6f)
            axis_out = {0, 1, 0};
        else
            axis_out = { (float)q.x / s, (float)q.y / s, (float)q.z / s };

        angle_deg_out = angle_rad * (180.0f / 3.14159265f);
    }

    // ── Depth-coded colour: green (shallow) → yellow → red (deep) ─────────
    static inline Color depth_color_rl(float depth, float max_depth = 0.15f)
    {
        const float t = std::min(depth / max_depth, 1.0f);
        if (t < 0.5f)
        {
            const uint8_t r = (uint8_t)(t * 2.0f * 255);
            return { r, 255, 0, 255 };
        }
        const uint8_t g = (uint8_t)((1.0f - (t - 0.5f) * 2.0f) * 255);
        return { 255, g, 0, 255 };
    }

    // ── Lambda-coded colour: blue (large compression) → white → pink (tension) ─
    static inline Color lambda_color_rl(float lambda)
    {
        const float mag = std::fabs(lambda);
        const float t   = std::min(mag / 5.0f, 1.0f);
        if (lambda < 0.0f)
        {
            const uint8_t c = (uint8_t)((1.0f - t * 0.6f) * 255);
            return { c, c, 255, 220 };
        }
        const uint8_t c = (uint8_t)((1.0f - t * 0.6f) * 255);
        return { 255, c, c, 220 };
    }

    // ── Draw a 3-D arrow from `origin` in direction `dir` ──────────────────
    //    head_frac: fraction of total length that is the arrowhead cone
    static inline void draw_arrow(Vector3 origin, Vector3 dir,
                                  float length, Color col,
                                  float head_frac = 0.25f,
                                  float shaft_radius = 0.01f)
    {
        const float head_len  = length * head_frac;
        const float shaft_len = length - head_len;
        const float head_r    = shaft_radius * 3.5f;

        const Vector3 tip  = { origin.x + dir.x * length,
                                origin.y + dir.y * length,
                                origin.z + dir.z * length };
        const Vector3 neck = { origin.x + dir.x * shaft_len,
                                origin.y + dir.y * shaft_len,
                                origin.z + dir.z * shaft_len };

        // Shaft as a thin cylinder
        DrawCylinder(origin, shaft_radius, shaft_radius, shaft_len, 6, col);
        // Head as a cone
        DrawCylinder(neck, head_r, 0.001f, head_len, 8, col);
        (void)tip; // suppress unused-variable warning
    }

    // ── Draw a world-aligned vector from a point (cheap line version) ───────
    static inline void draw_vec_arrow(const m3d::vec3 &origin,
                                      const m3d::vec3 &vec,
                                      Color col)
    {
        const m3d::vec3 tip = origin + vec;
        DrawLine3D(to_rl(origin), to_rl(tip), col);
        DrawSphere(to_rl(tip), 0.025f, col);
    }

    // =========================================================================
    //  Shape wireframes
    // =========================================================================

    // =========================================================================
    //  Shape wireframes  (drop-in replacement for the draw_shape() function
    //  in include/visr/ui/RaylibDraw.hpp)
    // =========================================================================

    static inline void draw_shape(const ColliderSnap &c, Color col)
    {
        const Vector3 pos = to_rl(c.world_pos);

        std::visit([&](auto &&shape)
        {
            using T = std::decay_t<decltype(shape)>;

            // ── Sphere ───────────────────────────────────────────────────────
            if constexpr (std::is_same_v<T, SpherSnap>)
            {
                Vector3 axis; float deg;
                quat_to_axis_angle(c.world_rot, axis, deg);
                rlPushMatrix();
                rlTranslatef(pos.x, pos.y, pos.z);
                rlRotatef(deg, axis.x, axis.y, axis.z);
                DrawSphereWires({0.0, 0.0, 0.0}, (float)shape.radius, 10, 10, col);
                rlPopMatrix();
            }
            // ── Box ──────────────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, BoxSnap>)
            {
                Vector3 axis; float deg;
                quat_to_axis_angle(c.world_rot, axis, deg);
                rlPushMatrix();
                rlTranslatef(pos.x, pos.y, pos.z);
                rlRotatef(deg, axis.x, axis.y, axis.z);
                DrawCubeWires({0, 0, 0},
                              (float)shape.half_extents.x * 2.0f,
                              (float)shape.half_extents.y * 2.0f,
                              (float)shape.half_extents.z * 2.0f, col);
                rlPopMatrix();
            }
            // ── Capsule ──────────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, CapsuleSnap>)
            {
                const float r  = (float)shape.radius;
                const float hh = (float)shape.half_height;
                Vector3 axis; float deg;
                quat_to_axis_angle(c.world_rot, axis, deg);
                rlPushMatrix();
                rlTranslatef(pos.x, pos.y, pos.z);
                rlRotatef(deg, axis.x, axis.y, axis.z);
                DrawCylinderWires({0, -hh, 0}, r, r, hh * 2.0f, 10, col);
                DrawSphereWires({0,  hh, 0}, r, 6, 6, col);
                DrawSphereWires({0, -hh, 0}, r, 6, 6, col);
                rlPopMatrix();
            }
            // ── Plane ────────────────────────────────────────────────────────
            // Draw an oriented grid in the plane's tangent space, plus a
            // normal arrow.  The grid is centered at the plane's world origin
            // (c.world_pos), shifted by shape.distance along the normal.
            else if constexpr (std::is_same_v<T, PlaneSnap>)
            {
                // World-space normal: rotate local normal by the body's orientation.
                const m3d::vec3 raw_n = m3d::rotate(c.world_rot, shape.normal);

                // Build an orthonormal tangent frame in the plane.
                // Choose the axis least aligned with raw_n to avoid degeneracy.
                m3d::vec3 tangent;
                if (std::fabs(raw_n.y) < 0.9)
                    tangent = m3d::normalize(m3d::cross(raw_n, m3d::vec3(0, 1, 0)));
                else
                    tangent = m3d::normalize(m3d::cross(raw_n, m3d::vec3(1, 0, 0)));
                const m3d::vec3 bitangent = m3d::cross(raw_n, tangent);

                // Plane origin: body position shifted by the plane's d offset.
                // shape.distance stores the local-space d value; the body
                // rotation is already applied via raw_n, so we shift along it.
                const m3d::vec3 origin = c.world_pos + raw_n * (double)shape.distance;

                // Draw a 20×80 grid of lines (1-unit spacing).
                constexpr int   HALF  = 50;
                constexpr float STEP  = 1.0f;
                constexpr float EXT   = HALF * STEP;

                for (int k = -HALF; k <= HALF; ++k)
                {
                    const double t = k * STEP;

                    // Lines along the tangent direction
                    const m3d::vec3 a1 = origin + tangent * t + bitangent * (double)(-EXT);
                    const m3d::vec3 b1 = origin + tangent * t + bitangent * (double)( EXT);
                    DrawLine3D(to_rl(a1), to_rl(b1), col);

                    // Lines along the bitangent direction
                    const m3d::vec3 a2 = origin + bitangent * t + tangent * (double)(-EXT);
                    const m3d::vec3 b2 = origin + bitangent * t + tangent * (double)( EXT);
                    DrawLine3D(to_rl(a2), to_rl(b2), col);
                }

                // Normal arrow so the plane orientation is immediately visible.
                const m3d::vec3 tip = origin + raw_n * 0.75;
                DrawLine3D(to_rl(origin), to_rl(tip), PURPLE);
                DrawSphere(to_rl(tip), 0.05f, PURPLE);
            }
            // ── Cone ─────────────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, ConeSnap>)
            {
                DrawCylinderWires(pos, 0.0f, (float)shape.radius,
                                  (float)shape.height, 10, col);
            }
            // ── Ellipsoid ────────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, EllipsoidSnap>)
            {
                rlPushMatrix();
                rlTranslatef(pos.x, pos.y, pos.z);
                rlScalef((float)shape.semi_axes.x,
                         (float)shape.semi_axes.y,
                         (float)shape.semi_axes.z);
                DrawSphereWires({0, 0, 0}, 1.0f, 10, 10, col);
                rlPopMatrix();
            }
            // ── Heightmap ────────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, HeightmapSnap>)
            {
                const float w = shape.cols * (float)shape.cell_size;
                const float d = shape.rows * (float)shape.cell_size;
                DrawCubeWires(pos, w, 0.1f, d, col);
            }
            // ── Mesh ─────────────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, MeshSnap>)
            {
                DrawSphereWires(pos, 0.2f, 4, 4, col);
            }
            // ── ConvexHull ───────────────────────────────────────────────────
            else if constexpr (std::is_same_v<T, ConvexHullSnap>)
            {
                Vector3 axis; float deg;
                quat_to_axis_angle(c.world_rot, axis, deg);
                rlPushMatrix();
                rlTranslatef(pos.x, pos.y, pos.z);
                rlRotatef(deg, axis.x, axis.y, axis.z);

                if (!shape.face_indices.empty())
                {
                    // Draw each triangle edge once (no dedup — small hulls).
                    for (std::size_t f = 0; f + 2 < shape.face_indices.size(); f += 3)
                    {
                        const auto &v0 = shape.vertices[shape.face_indices[f + 0]];
                        const auto &v1 = shape.vertices[shape.face_indices[f + 1]];
                        const auto &v2 = shape.vertices[shape.face_indices[f + 2]];
                        DrawLine3D(to_rl(v0), to_rl(v1), col);
                        DrawLine3D(to_rl(v1), to_rl(v2), col);
                        DrawLine3D(to_rl(v2), to_rl(v0), col);
                    }
                }
                else
                {
                    // Vertex-only fallback: tiny markers at each vertex.
                    for (const auto &v : shape.vertices)
                        DrawSphereWires(to_rl(v), 0.02f, 4, 4, col);
                }
                rlPopMatrix();
            }
        }, c.shape);
    }

    // =========================================================================
    //  Colliders
    // =========================================================================

    inline void draw_colliders(const FrameSnapshot &snap,
                               uint32_t selected_body_id = UINT32_MAX)
    {
        for (const auto &c : snap.colliders)
        {
            Color col = c.is_static ? GRAY : SKYBLUE;
            if (c.body_id == selected_body_id) col = GREEN;
            draw_shape(c, col);
        }
    }

    // =========================================================================
    //  Contacts — the main debugging surface
    //
    //  Rendering layers (from least to most prominent):
    //    1. Penetration cylinder  — thick translucent bar along the normal,
    //                               length = penetration_depth, colour = depth-coded.
    //    2. Separation line       — thin MAGENTA line from p_a to p_b.
    //    3. Contact point A       — BLUE  sphere at p_a  (body A's surface point)
    //    4. Contact point B       — RED   sphere at p_b  (body B's surface point)
    //    5. Normal arrow          — depth-coded colour from midpoint outward.
    //    6. Selected contact      — extra highlight ring around midpoint.
    // =========================================================================

    struct ContactDrawConfig
    {
        float  normal_scale        = 0.25f;  // normal arrow length
        float  contact_sphere_r    = 0.035f; // radius of p_a / p_b spheres
        bool   show_penetration_bar= true;   // thick depth bar
        bool   show_separation_line= true;   // p_a → p_b connector
        bool   show_lambda_tint    = false;  // tint normal arrow by lambda
        bool   show_inactive       = false;
    };

    inline void draw_contacts(const FrameSnapshot      &snap,
                              const ContactDrawConfig   &cfg        = {},
                              uint32_t                   selected_i = UINT32_MAX)
    {
        for (uint32_t i = 0; i < (uint32_t)snap.contacts.size(); ++i)
        {
            const ContactSnap &ct = snap.contacts[i];
            if (!ct.active && !cfg.show_inactive) continue;

            const float alpha    = ct.active ? 1.0f : 0.35f;
            const m3d::vec3 mid  = (ct.point_on_a + ct.point_on_b) * 0.5;
            const float depth    = (float)ct.penetration_depth;

            // ── 1. Penetration depth bar ────────────────────────────────────
            if (cfg.show_penetration_bar && depth > 1e-5f)
            {
                Color dc = depth_color_rl(depth);
                dc.a     = (uint8_t)(alpha * 160);
                const m3d::vec3 bar_end = mid - ct.normal * (double)depth;
                DrawLine3D(to_rl(mid), to_rl(bar_end), dc);
            }

            // ── 2. Separation line p_a → p_b ───────────────────────────────
            if (cfg.show_separation_line)
            {
                Color lc = MAGENTA;
                lc.a     = (uint8_t)(alpha * 180);
                DrawLine3D(to_rl(ct.point_on_a), to_rl(ct.point_on_b), lc);
            }

            // ── 3. Contact point A (blue) ───────────────────────────────────
            {
                Color ca = { 80, 140, 255, (uint8_t)(alpha * 255) };
                DrawSphere(to_rl(ct.point_on_a), cfg.contact_sphere_r, ca);
            }

            // ── 4. Contact point B (red) ────────────────────────────────────
            {
                Color cb = { 255, 80, 80, (uint8_t)(alpha * 255) };
                DrawSphere(to_rl(ct.point_on_b), cfg.contact_sphere_r, cb);
            }

            // ── 5. Normal arrow from midpoint ───────────────────────────────
            {
                Color nc = cfg.show_lambda_tint
                    ? lambda_color_rl((float)ct.normal_lambda)
                    : depth_color_rl(depth);
                nc.a = (uint8_t)(alpha * 255);

                const m3d::vec3 normal_tip =
                    mid + ct.normal * (double)cfg.normal_scale;
                DrawLine3D(to_rl(mid), to_rl(normal_tip), nc);
                DrawSphere(to_rl(normal_tip), cfg.contact_sphere_r * 0.7f, nc);
            }

            // ── 6. Selection highlight ──────────────────────────────────────
            if (i == selected_i)
            {
                DrawSphereWires(to_rl(mid), cfg.contact_sphere_r * 3.5f,
                                6, 6, YELLOW);
                // Draw normal force vector if non-zero
                const float fn_len = (float)(
                    std::sqrt(ct.normal_force.x * ct.normal_force.x +
                              ct.normal_force.y * ct.normal_force.y +
                              ct.normal_force.z * ct.normal_force.z));
                if (fn_len > 1e-4f)
                {
                    const float scale = std::min(fn_len * 0.01f, 0.5f);
                    draw_vec_arrow(mid, ct.normal_force * (double)scale, GOLD);
                }
                // Draw tangent force vector if non-zero
                const float ft_len = (float)(
                    std::sqrt(ct.tangent_force.x * ct.tangent_force.x +
                              ct.tangent_force.y * ct.tangent_force.y +
                              ct.tangent_force.z * ct.tangent_force.z));
                if (ft_len > 1e-4f)
                {
                    const float scale = std::min(ft_len * 0.01f, 0.5f);
                    draw_vec_arrow(mid, ct.tangent_force * (double)scale, ORANGE);
                }
            }
        }
    }

    // =========================================================================
    //  Joints
    // =========================================================================

    inline void draw_joints(const FrameSnapshot &snap,
                            uint32_t selected_joint_id = UINT32_MAX)
    {
        for (const auto &j : snap.joints)
        {
            const bool sel = (j.id == selected_joint_id);
            const Vector3 a = to_rl(j.anchor_a);
            const Vector3 b = to_rl(j.anchor_b);

            // Anchors
            DrawSphere(a, sel ? 0.10f : 0.06f, sel ? LIME   : YELLOW);
            DrawSphere(b, sel ? 0.10f : 0.06f, sel ? LIME   : YELLOW);
            DrawLine3D(a, b, YELLOW);

            // Main axis arrow
            const m3d::vec3 axis_tip = j.anchor_a + j.main_axis * 0.35;
            DrawLine3D(a, to_rl(axis_tip), PURPLE);
            DrawSphere(to_rl(axis_tip), 0.025f, PURPLE);

            // Limit indicator when active
            if (j.limited)
            {
                const float pos = (float)j.current_position;
                const float lo  = (float)j.lower_limit;
                const float hi  = (float)j.upper_limit;
                const bool  out = (pos < lo || pos > hi);
                // Draw a small indicator sphere on the axis proportional to position
                const float t        = hi > lo ? (pos - lo) / (hi - lo) : 0.5f;
                const m3d::vec3 ind  = j.anchor_a + j.main_axis * (double)(0.05 + t * 0.3);
                DrawSphere(to_rl(ind), 0.04f, out ? RED : LIME);
            }
        }
    }

    // =========================================================================
    //  Velocity vectors
    // =========================================================================

    inline void draw_velocity_vectors(const FrameSnapshot &snap,
                                      float lin_scale = 0.08f,
                                      float ang_scale = 0.05f)
    {
        for (const auto &b : snap.bodies)
        {
            if (b.is_static) continue;

            // Linear velocity — LIME
            const float lv = (float)std::sqrt(
                b.linear_velocity.x * b.linear_velocity.x +
                b.linear_velocity.y * b.linear_velocity.y +
                b.linear_velocity.z * b.linear_velocity.z);
            if (lv > 0.01f)
                draw_vec_arrow(b.position, b.linear_velocity * (double)lin_scale, LIME);

            // Angular velocity — ORANGE
            const float av = (float)std::sqrt(
                b.angular_velocity.x * b.angular_velocity.x +
                b.angular_velocity.y * b.angular_velocity.y +
                b.angular_velocity.z * b.angular_velocity.z);
            if (av > 0.01f)
                draw_vec_arrow(b.position, b.angular_velocity * (double)ang_scale, ORANGE);
        }
    }

    // =========================================================================
    //  Selected body highlight — draw axes (R=X, G=Y, B=Z) + outline
    // =========================================================================

    inline void draw_body_axes(const FrameSnapshot &snap,
                               uint32_t selected_body_id)
    {
        if (selected_body_id == UINT32_MAX) return;

        for (const auto &b : snap.bodies)
        {
            if (b.id != selected_body_id) continue;

            const float L = 0.5f;
            // X-axis (red)
            const m3d::vec3 xdir = m3d::rotate(b.orientation, m3d::vec3(L, 0, 0));
            // Y-axis (green)
            const m3d::vec3 ydir = m3d::rotate(b.orientation, m3d::vec3(0, L, 0));
            // Z-axis (blue)
            const m3d::vec3 zdir = m3d::rotate(b.orientation, m3d::vec3(0, 0, L));

            draw_vec_arrow(b.position, xdir, RED);
            draw_vec_arrow(b.position, ydir, GREEN);
            draw_vec_arrow(b.position, zdir, BLUE);
            break;
        }
    }

    /**
     * @brief Toggles for which overlays `draw_scene` renders.
     * @ingroup visr
     */
    struct DrawFlags
    {
        bool colliders          = true;  ///< Wireframes for every collider.
        bool contacts           = true;  ///< Contact points + normal arrows.
        bool joints             = true;  ///< Joint anchors and axes.
        bool velocities         = false; ///< LIME linear / ORANGE angular arrows.
        bool body_axes          = true;  ///< Local-frame axes on the selected body.
        bool inactive_contacts  = false; ///< Show separated contacts (`active == false`).
        bool lambda_tint        = false; ///< Tint normal arrows by accumulated impulse.

        ContactDrawConfig contact_cfg{}; ///< Fine-grained contact-overlay options.
    };

    /**
     * @brief Render the full scene from a snapshot in one call.
     *
     * Iterates colliders, contacts, joints, and (when selected) the
     * highlighted body's axes. Highlight IDs default to `UINT32_MAX` —
     * pass an actual stable ID to draw the body in `GREEN`.
     *
     * @ingroup visr
     */
    inline void draw_scene(const FrameSnapshot &snap,
                           const DrawFlags     &flags    = {},
                           uint32_t             sel_body = UINT32_MAX,
                           uint32_t             sel_contact = UINT32_MAX,
                           uint32_t             sel_joint   = UINT32_MAX)
    {
        if (flags.colliders)
            draw_colliders(snap, sel_body);

        if (flags.contacts)
        {
            ContactDrawConfig cfg = flags.contact_cfg;
            cfg.show_inactive  = flags.inactive_contacts;
            cfg.show_lambda_tint = flags.lambda_tint;
            draw_contacts(snap, cfg, sel_contact);
        }

        if (flags.joints)
            draw_joints(snap, sel_joint);

        if (flags.velocities)
            draw_velocity_vectors(snap);

        if (flags.body_axes)
            draw_body_axes(snap, sel_body);
    }

} // namespace visr::draw