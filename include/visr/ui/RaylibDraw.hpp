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

// ============================================================================
//  visr/ui/RaylibDraw.hpp
//
//  Stateless draw functions for every visr snapshot type.
//  Each function takes const refs to snapshot data and a Camera3D.
//  No physics headers are included here.
//
//  Colour scheme:
//    Dynamic wireframe  — SKYBLUE
//    Static  wireframe  — GRAY
//    Selected body      — GREEN
//    Contact point      — RED sphere
//    Contact normal     — ORANGE arrow
//    Joint anchor       — YELLOW sphere + line
//    Joint axis         — PURPLE arrow
//    Velocity vector    — LIME arrow
// ============================================================================

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
        // Manual clamp — std::clamp needs <algorithm> and same types;
        // m3d::clamp exists but wants m3d::scalar. Just inline it.
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

    // ── Shape wireframes ──────────────────────────────────────────────────────

    static inline void draw_shape(const ColliderSnap &c, Color col)
    {
        const Vector3 pos = to_rl(c.world_pos);

        std::visit([&](auto &&shape)
        {
            using T = std::decay_t<decltype(shape)>;

            if constexpr (std::is_same_v<T, SpherSnap>)
            {
                DrawSphereWires(pos, (float)shape.radius, 8, 8, col);
            }
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
                              (float)shape.half_extents.z * 2.0f,
                              col);
                rlPopMatrix();
            }
            else if constexpr (std::is_same_v<T, CapsuleSnap>)
            {
                const float r = (float)shape.radius;
                const float hh = (float)shape.half_height;
                const Vector3 top = { pos.x, pos.y + hh, pos.z };
                const Vector3 bot = { pos.x, pos.y - hh, pos.z };
                DrawCylinderWires(bot, r, r, hh * 2.0f, 10, col);
                DrawSphereWires(top, r, 6, 6, col);
                DrawSphereWires(bot, r, 6, 6, col);
            }
            else if constexpr (std::is_same_v<T, PlaneSnap>)
            {
                DrawGrid(20, 1.0f);
            }
            else if constexpr (std::is_same_v<T, ConeSnap>)
            {
                DrawCylinderWires(pos, 0.0f, (float)shape.radius,
                                  (float)shape.height, 10, col);
            }
            else if constexpr (std::is_same_v<T, EllipsoidSnap>)
            {
                rlPushMatrix();
                rlTranslatef(pos.x, pos.y, pos.z);
                rlScalef((float)shape.semi_axes.x,
                         (float)shape.semi_axes.y,
                         (float)shape.semi_axes.z);
                DrawSphereWires({0, 0, 0}, 1.0f, 8, 8, col);
                rlPopMatrix();
            }
            else if constexpr (std::is_same_v<T, HeightmapSnap>)
            {
                const float w = shape.cols * (float)shape.cell_size;
                const float d = shape.rows * (float)shape.cell_size;
                DrawCubeWires(pos, w, 0.1f, d, col);
            }
            else if constexpr (std::is_same_v<T, MeshSnap>)
            {
                DrawSphereWires(pos, 0.2f, 4, 4, col);
            }
        }, c.shape);
    }

    // ── Draw all colliders ────────────────────────────────────────────────────

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

    // ── Draw contacts + normals ───────────────────────────────────────────────

    inline void draw_contacts(const FrameSnapshot &snap,
                              float normal_scale   = 0.3f,
                              bool  show_inactive  = false)
    {
        for (const auto &ct : snap.contacts)
        {
            if (!ct.active && !show_inactive) continue;

            const Color col  = ct.active ? RED : Fade(RED, 0.3f);
            const m3d::vec3 mid = (ct.point_on_a + ct.point_on_b) * 0.5;
            DrawSphere(to_rl(mid), 0.04f, col);

            const m3d::vec3 tip = mid + ct.normal * (double)normal_scale;
            DrawLine3D(to_rl(mid), to_rl(tip), ORANGE);
            DrawSphere(to_rl(tip), 0.02f, ORANGE);
        }
    }

    // ── Draw joint anchors ────────────────────────────────────────────────────

    inline void draw_joints(const FrameSnapshot &snap)
    {
        for (const auto &j : snap.joints)
        {
            const Vector3 a = to_rl(j.anchor_a);
            const Vector3 b = to_rl(j.anchor_b);
            DrawSphere(a, 0.06f, YELLOW);
            DrawSphere(b, 0.06f, YELLOW);
            DrawLine3D(a, b, YELLOW);

            const m3d::vec3 axis_tip = j.anchor_a + j.main_axis * 0.3;
            DrawLine3D(a, to_rl(axis_tip), PURPLE);
        }
    }

    // ── Velocity vectors ──────────────────────────────────────────────────────

    inline void draw_velocity_vectors(const FrameSnapshot &snap,
                                      float scale = 0.1f)
    {
        for (const auto &b : snap.bodies)
        {
            if (b.is_static) continue;
            const m3d::vec3 tip = b.position + b.linear_velocity * (double)scale;
            DrawLine3D(to_rl(b.position), to_rl(tip), LIME);
        }
    }

    // ── Full scene ────────────────────────────────────────────────────────────

    struct DrawFlags
    {
        bool colliders         = true;
        bool contacts          = true;
        bool joints            = true;
        bool velocities        = false;
        bool inactive_contacts = false;
    };

    inline void draw_scene(const FrameSnapshot &snap,
                           const DrawFlags     &flags    = {},
                           uint32_t             sel_body = UINT32_MAX)
    {
        if (flags.colliders) draw_colliders(snap, sel_body);
        if (flags.contacts)  draw_contacts(snap, 0.3f, flags.inactive_contacts);
        if (flags.joints)    draw_joints(snap);
        if (flags.velocities)draw_velocity_vectors(snap);
    }

} // namespace visr::draw