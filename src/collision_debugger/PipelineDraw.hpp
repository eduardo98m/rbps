#pragma once
#include <raylib.h>
#include <rlgl.h>

#ifdef PI
#  undef PI
#endif

#include "State.hpp"
#include "PipelineRun.hpp"
#include "visr/Snapshot.hpp"
#include "visr/SnapshotBuilder.hpp"
#include "visr/ui/RaylibDraw.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

namespace cdbg
{

    // Build a transient ColliderSnap so we can hand any rbc::Shape to
    // visr::draw::draw_shape without duplicating the per-shape rendering
    // code that already lives in include/visr/ui/RaylibDraw.hpp.
    inline visr::ColliderSnap make_fake_collider_snap(const rbc::Shape &s,
                                                      const m3d::tf &tf)
    {
        visr::ColliderSnap c{};
        c.id        = 0;
        c.body_id   = 0;
        c.is_static = false;
        c.world_pos = tf.pos;
        c.world_rot = tf.rot;
        c.shape     = visr::detail::to_shape_snap(s);
        return c;
    }

    inline void draw_polygon_outline(const m3d::vec3 *pts, int n, Color col)
    {
        if (n < 2) return;
        for (int i = 0; i < n; ++i)
        {
            const m3d::vec3 &a = pts[i];
            const m3d::vec3 &b = pts[(i + 1) % n];
            DrawLine3D(visr::draw::to_rl(a), visr::draw::to_rl(b), col);
        }
    }

    inline void draw_polygon_fan(const m3d::vec3 *pts, int n, Color col)
    {
        if (n < 3) return;
        const Vector3 c0 = visr::draw::to_rl(pts[0]);
        for (int i = 1; i < n - 1; ++i)
        {
            const Vector3 a = visr::draw::to_rl(pts[i]);
            const Vector3 b = visr::draw::to_rl(pts[i + 1]);
            // Two-sided: draw both winding orders so the fill is visible from
            // either camera side without depending on backface culling.
            DrawTriangle3D(c0, a, b, col);
            DrawTriangle3D(c0, b, a, col);
        }
    }

    inline void draw_world_grid(int half, float step, Color col)
    {
        const float ext = half * step;
        for (int k = -half; k <= half; ++k)
        {
            const float t = k * step;
            DrawLine3D({-ext, 0.0f, t}, {ext, 0.0f, t}, col);
            DrawLine3D({t, 0.0f, -ext}, {t, 0.0f, ext}, col);
        }
    }

    inline void draw_pipeline(const rbc::Shape &shape_a, const m3d::tf &tf_a,
                              const rbc::Shape &shape_b, const m3d::tf &tf_b,
                              const PipelineResult &r,
                              const VizFlags &v)
    {
        if (v.world_grid)
            draw_world_grid(20, 1.0f, Color{60, 60, 60, 120});

        // ── 1. Shape wireframes ─────────────────────────────────────────
        if (v.shape_a)
        {
            const visr::ColliderSnap c = make_fake_collider_snap(shape_a, tf_a);
            visr::draw::draw_shape(c, SKYBLUE);
        }
        if (v.shape_b)
        {
            const visr::ColliderSnap c = make_fake_collider_snap(shape_b, tf_b);
            visr::draw::draw_shape(c, ORANGE);
        }

        // Everything below here only makes sense when the pipeline reached
        // the relevant stage.
        if (!r.overlap)
            return;

        // ── 2. GJK simplex (MD-space, drawn at offset) ──────────────────
        if (v.gjk_simplex && r.gjk_simplex_rank > 0)
        {
            const m3d::vec3 off = v.md_space_offset;
            for (int i = 0; i < r.gjk_simplex_rank; ++i)
            {
                const m3d::vec3 p = r.gjk_simplex[i].w + off;
                DrawSphere(visr::draw::to_rl(p), 0.04f, MAROON);
            }
            for (int i = 0; i < r.gjk_simplex_rank; ++i)
                for (int j = i + 1; j < r.gjk_simplex_rank; ++j)
                {
                    const m3d::vec3 a = r.gjk_simplex[i].w + off;
                    const m3d::vec3 b = r.gjk_simplex[j].w + off;
                    DrawLine3D(visr::draw::to_rl(a),
                               visr::draw::to_rl(b), MAROON);
                }
            // Origin marker (where GJK found the inclusion)
            DrawSphereWires(visr::draw::to_rl(off), 0.06f, 6, 6, YELLOW);
        }

        // ── 3. EPA polytope (MD-space, drawn at offset) ─────────────────
        if (v.epa_polytope && r.epa_converged)
        {
            const m3d::vec3 off = v.md_space_offset;
            for (const EPAFaceCopy &f : r.epa_faces)
            {
                m3d::vec3 v0 = f.v[0] + off;
                m3d::vec3 v1 = f.v[1] + off;
                m3d::vec3 v2 = f.v[2] + off;
                Color edge = {180, 180, 220, 200};
                DrawLine3D(visr::draw::to_rl(v0), visr::draw::to_rl(v1), edge);
                DrawLine3D(visr::draw::to_rl(v1), visr::draw::to_rl(v2), edge);
                DrawLine3D(visr::draw::to_rl(v2), visr::draw::to_rl(v0), edge);
                // Tiny outward-normal stub
                const m3d::vec3 cen = (v0 + v1 + v2) * (1.0 / 3.0);
                const m3d::vec3 tip = cen + f.normal * 0.10;
                DrawLine3D(visr::draw::to_rl(cen),
                           visr::draw::to_rl(tip), Color{120, 120, 255, 220});
            }
            DrawSphereWires(visr::draw::to_rl(off), 0.06f, 6, 6, YELLOW);
        }

        // ── 4. MD-space axes (orientation aid for the offset polytope) ──
        if (v.md_space_axes && (v.gjk_simplex || v.epa_polytope))
        {
            const m3d::vec3 o = v.md_space_offset;
            visr::draw::draw_vec_arrow(o, m3d::vec3(0.4, 0.0, 0.0), RED);
            visr::draw::draw_vec_arrow(o, m3d::vec3(0.0, 0.4, 0.0), GREEN);
            visr::draw::draw_vec_arrow(o, m3d::vec3(0.0, 0.0, 0.4), BLUE);
        }

        if (!r.epa_converged)
            return;

        // ── 5. EPA normal (world space) ─────────────────────────────────
        if (v.epa_normal)
        {
            const float L = (float)r.epa_depth + 0.4f;
            visr::draw::draw_arrow(visr::draw::to_rl(r.epa_contact_point),
                                   visr::draw::to_rl(r.epa_normal),
                                   L, PURPLE, 0.20f, 0.012f);
        }
        if (v.epa_contact_point)
        {
            DrawSphere(visr::draw::to_rl(r.epa_contact_point), 0.05f, YELLOW);
        }

        if (!r.manifold_built)
            return;

        // ── 6. Reference face (on shape A) — translucent green fill + outline ──
        if (v.ref_face && r.ref_face_n > 0)
        {
            const Color fill   = {  0, 200,  80,  90};
            const Color border = {  0, 255, 120, 230};
            draw_polygon_fan    (r.ref_face, r.ref_face_n, fill);
            draw_polygon_outline(r.ref_face, r.ref_face_n, border);
            for (int i = 0; i < r.ref_face_n; ++i)
                DrawSphere(visr::draw::to_rl(r.ref_face[i]), 0.02f, border);
        }

        // ── 7. Incident face (on shape B) — translucent red fill + outline ─────
        if (v.inc_face && r.inc_face_n > 0)
        {
            const Color fill   = {220,  60,  60,  80};
            const Color border = {255,  90,  90, 230};
            draw_polygon_fan    (r.inc_face, r.inc_face_n, fill);
            draw_polygon_outline(r.inc_face, r.inc_face_n, border);
            for (int i = 0; i < r.inc_face_n; ++i)
                DrawSphere(visr::draw::to_rl(r.inc_face[i]), 0.02f, border);
        }

        // ── 8. Clipped polygon (post Sutherland-Hodgman) ────────────────
        if (v.clipped_polygon && !r.post_clip_polygon.empty())
        {
            const Color border = {255, 230, 60, 255};
            draw_polygon_outline(r.post_clip_polygon.data(),
                                 (int)r.post_clip_polygon.size(), border);
            for (const m3d::vec3 &p : r.post_clip_polygon)
                DrawSphere(visr::draw::to_rl(p), 0.018f, border);
        }

        // ── 9. Final manifold points ────────────────────────────────────
        if (v.manifold_points)
        {
            for (int i = 0; i < r.manifold.num_points; ++i)
            {
                const auto &pt = r.manifold.points[i];
                const float depth = (float)pt.penetration_depth;
                const Color c = visr::draw::depth_color_rl(depth);
                const float radius = 0.04f + 0.10f * std::min(depth / 0.05f, 1.0f);
                DrawSphere(visr::draw::to_rl(pt.position), radius, c);
                DrawSphereWires(visr::draw::to_rl(pt.position),
                                radius * 1.1f, 6, 6, WHITE);
            }
        }
    }

} // namespace cdbg
