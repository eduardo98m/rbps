#pragma once
#include <raylib.h>
#include <limits>
#include <cmath>
#include "visr/Snapshot.hpp"
#include "visr/Command.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/ui/Panels.hpp"

// ============================================================================
//  visr/systems/SelectionSystem.hpp
//
//  Left-click picking against collider bounding volumes.
//  Works entirely off the FrameSnapshot — no physics types needed.
//
//  Strategy: for each collider, compute a conservative world-space AABB
//  from the ShapeSnap, test against the mouse ray, keep the closest hit.
//  Sphere and capsule get tight AABBs; box uses its half_extents directly.
// ============================================================================

namespace visr
{
    // ── Per-shape conservative AABB ──────────────────────────────────────────
    struct AABB3 { float min_x, min_y, min_z, max_x, max_y, max_z; };

    static inline AABB3 snap_aabb(const ColliderSnap &c)
    {
        const float px = (float)c.world_pos.x;
        const float py = (float)c.world_pos.y;
        const float pz = (float)c.world_pos.z;

        return std::visit([&](auto &&sh) -> AABB3
        {
            using T = std::decay_t<decltype(sh)>;
            float r = 0.5f;

            if constexpr (std::is_same_v<T, SpherSnap>)
                r = (float)sh.radius;
            else if constexpr (std::is_same_v<T, BoxSnap>)
                // Bounding sphere radius of the box
                r = std::sqrt((float)(sh.half_extents.x * sh.half_extents.x +
                                      sh.half_extents.y * sh.half_extents.y +
                                      sh.half_extents.z * sh.half_extents.z));
            else if constexpr (std::is_same_v<T, CapsuleSnap>)
                r = (float)sh.radius + (float)sh.half_height;
            else if constexpr (std::is_same_v<T, ConeSnap>)
                r = std::max((float)sh.radius, (float)sh.height);
            else if constexpr (std::is_same_v<T, EllipsoidSnap>)
                r = (float)std::max({sh.semi_axes.x, sh.semi_axes.y, sh.semi_axes.z});
            else if constexpr (std::is_same_v<T, HeightmapSnap>)
                r = std::max(sh.cols * (float)sh.cell_size,
                             sh.rows * (float)sh.cell_size) * 0.5f;
            else if constexpr (std::is_same_v<T, PlaneSnap>)
                r = 50.0f; // planes are infinite — large proxy
            else
                r = 0.5f;

            return { px - r, py - r, pz - r, px + r, py + r, pz + r };
        }, c.shape);
    }

    // ── Ray-AABB intersection ─────────────────────────────────────────────────
    static inline bool ray_aabb_hit(Ray ray, const AABB3 &box, float &t_out)
    {
        float tmin = -std::numeric_limits<float>::max();
        float tmax =  std::numeric_limits<float>::max();

        const float origins[3]  = { ray.position.x, ray.position.y, ray.position.z };
        const float dirs[3]     = { ray.direction.x, ray.direction.y, ray.direction.z };
        const float bmin[3]     = { box.min_x, box.min_y, box.min_z };
        const float bmax[3]     = { box.max_x, box.max_y, box.max_z };

        for (int i = 0; i < 3; ++i)
        {
            if (std::fabs(dirs[i]) < 1e-8f)
            {
                if (origins[i] < bmin[i] || origins[i] > bmax[i]) return false;
            }
            else
            {
                float t1 = (bmin[i] - origins[i]) / dirs[i];
                float t2 = (bmax[i] - origins[i]) / dirs[i];
                if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmax < tmin) return false;
            }
        }

        if (tmax < 0.0f) return false; // behind camera
        t_out = tmin > 0.0f ? tmin : tmax;
        return true;
    }

    // ── SelectionSystem ───────────────────────────────────────────────────────

    struct SelectionSystem
    {
        // Call once per render frame from the render thread.
        // Updates sel and pushes a CmdSelectBody if something was hit.
        void update(const FrameSnapshot   &snap,
                    const Camera3D        &camera,
                    visr::ui::SelectionState    &sel,
                    InProcessTransport    &transport) const
        {
            if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) return;

            // ImGui may have consumed this click
            // (check via ImGui::GetIO().WantCaptureMouse if imgui.h is available)

            const Ray ray = GetMouseRay(GetMousePosition(), camera);

            float        best_t    = std::numeric_limits<float>::max();
            uint32_t     best_body = UINT32_MAX;

            for (const auto &c : snap.colliders)
            {
                const AABB3 box = snap_aabb(c);
                float t = 0.0f;
                if (ray_aabb_hit(ray, box, t) && t < best_t)
                {
                    best_t    = t;
                    best_body = c.body_id;
                }
            }

            if (best_body != UINT32_MAX)
            {
                sel.body_id = best_body;
                transport.push_command(CmdSelectBody{best_body});
            }
            else
            {
                sel.body_id = UINT32_MAX;
                transport.push_command(CmdClearSelection{});
            }
        }
    };

} // namespace visr