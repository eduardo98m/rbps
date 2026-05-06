#pragma once
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "rbc/gjk/ContactManifoldGenerator.hpp"
#include "rbc/Contact.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include <math3d/math3d.hpp>
#include <vector>

namespace cdbg
{

    struct EPAFaceCopy
    {
        m3d::vec3   v[3];     // Minkowski-difference space (SimplexVertex.w)
        m3d::vec3   v0_a[3];  // World-space support points on shape A
        m3d::vec3   v1_b[3];  // World-space support points on shape B
        m3d::vec3   normal;
        m3d::scalar distance;
    };

    struct PipelineResult
    {
        // ── GJK ─────────────────────────────────────────────────────────
        rbc::GJK::Status   gjk_status     = rbc::GJK::Failed;
        int                gjk_simplex_rank = 0;
        rbc::SimplexVertex gjk_simplex[4]{};
        m3d::vec3          gjk_ray{};
        m3d::scalar        gjk_distance   = 0.0;

        // ── EPA ─────────────────────────────────────────────────────────
        rbc::EPA::Status   epa_status     = rbc::EPA::Failed;
        m3d::vec3          epa_normal{};
        m3d::scalar        epa_depth      = 0.0;
        m3d::vec3          epa_contact_point{};
        std::vector<EPAFaceCopy> epa_faces;

        // ── Manifold-gen intermediates (world space) ────────────────────
        int        ref_face_n = 0;
        m3d::vec3  ref_face[rbc::kMaxFaceCorners]{};
        int        inc_face_n = 0;
        m3d::vec3  inc_face[rbc::kMaxFaceCorners]{};
        std::vector<m3d::vec3>   post_clip_polygon;   // after Sutherland-Hodgman
        std::vector<m3d::vec3>   kept_points;         // after depth-test, pre-reduce
        std::vector<m3d::scalar> kept_depths;

        // ── Final manifold ──────────────────────────────────────────────
        rbc::ContactManifold manifold{};

        // ── Status flags ────────────────────────────────────────────────
        bool overlap        = false; // GJK reported Inside
        bool epa_converged  = false; // EPA returned Valid
        bool manifold_built = false; // generate_manifold ran to completion
    };

    // Run the full GJK → EPA → manifold pipeline and capture every
    // intermediate stage into `out`. This duplicates the body of
    // gjk_epa_manifold + generate_manifold from
    // include/rbc/gjk/ContactManifoldGenerator.hpp because those
    // functions don't expose intermediates — the duplication is
    // intentional and is the only piece of mirrored logic in the tool.
    inline void run_pipeline(const rbc::Shape &sa, const m3d::tf &tf_a,
                             const rbc::Shape &sb, const m3d::tf &tf_b,
                             PipelineResult &out)
    {
        out = PipelineResult{};

        rbc::MinkowskiDiff md(&sa, &sb, tf_a, tf_b);

        m3d::vec3 guess = tf_b.pos - tf_a.pos;
        if (m3d::length_sq(guess) < m3d::EPSILON)
            guess = m3d::vec3(1.0, 0.0, 0.0);

        // ── 1. GJK ──────────────────────────────────────────────────────
        rbc::GJK gjk;
        out.gjk_status   = gjk.evaluate(md, guess);
        out.gjk_ray      = gjk.ray;
        out.gjk_distance = gjk.distance;

        if (gjk.active_simplex)
        {
            const rbc::Simplex &sx = *gjk.active_simplex;
            out.gjk_simplex_rank = sx.rank;
            for (int i = 0; i < sx.rank && i < 4; ++i)
                out.gjk_simplex[i] = *sx.vertex[i];
        }

        if (out.gjk_status != rbc::GJK::Inside)
            return;
        out.overlap = true;

        // ── 2. EPA ──────────────────────────────────────────────────────
        rbc::EPA epa;
        out.epa_status = epa.evaluate(gjk, md);
        if (out.epa_status != rbc::EPA::Valid)
            return;
        out.epa_converged     = true;
        out.epa_normal        = epa.normal;
        out.epa_depth         = epa.depth;
        out.epa_contact_point = epa.contact_point;

        out.epa_faces.reserve(epa.num_faces);
        for (unsigned int i = 0; i < epa.num_faces; ++i)
        {
            const rbc::EPAFace *f = epa.faces[i];
            if (!f || f->obsolete) continue;
            EPAFaceCopy c;
            for (int k = 0; k < 3; ++k)
            {
                c.v[k]    = f->v[k]->w;
                c.v0_a[k] = f->v[k]->w0;
                c.v1_b[k] = f->v[k]->w1;
            }
            c.normal   = f->n;
            c.distance = f->d;
            out.epa_faces.push_back(c);
        }

        // ── 3. Manifold generation (mirror generate_manifold, capturing intermediates) ─
        const m3d::vec3 epa_n     = out.epa_normal;
        const m3d::scalar epa_d   = out.epa_depth;
        const m3d::vec3 epa_pt    = out.epa_contact_point;

        out.manifold.normal     = epa_n;
        out.manifold.num_points = 0;

        out.ref_face_n = rbc::shape_face_corners(sa, tf_a,  epa_n, out.ref_face, rbc::kMaxFaceCorners);
        out.inc_face_n = rbc::shape_face_corners(sb, tf_b, -epa_n, out.inc_face, rbc::kMaxFaceCorners);

        const m3d::vec3   ref_face_n = epa_n;
        const m3d::scalar ref_d_pln  = m3d::dot(out.ref_face[0], ref_face_n);

        // Sutherland-Hodgman against each side plane of the reference face
        constexpr int kClipBuf = 2 * rbc::kMaxFaceCorners;
        m3d::vec3 buf0[kClipBuf], buf1[kClipBuf];
        int cnt = out.inc_face_n;
        for (int i = 0; i < out.inc_face_n; ++i)
            buf0[i] = out.inc_face[i];

        for (int i = 0; i < out.ref_face_n && cnt > 0; ++i)
        {
            const m3d::vec3 edge = out.ref_face[(i + 1) % out.ref_face_n] - out.ref_face[i];
            const m3d::vec3 side_n = m3d::normalize(m3d::cross(ref_face_n, edge));
            cnt = rbc::manifold_detail::clip_polygon_by_plane(
                buf0, cnt, side_n, out.ref_face[i], buf1);
            for (int k = 0; k < cnt; ++k)
                buf0[k] = buf1[k];
        }

        out.post_clip_polygon.assign(buf0, buf0 + cnt);

        // Depth-test: keep points on or below the reference face plane
        m3d::vec3   keep_pts[kClipBuf];
        m3d::scalar keep_dep[kClipBuf];
        int keep_n = 0;
        for (int i = 0; i < cnt; ++i)
        {
            const m3d::scalar sd = m3d::dot(buf0[i], ref_face_n) - ref_d_pln;
            if (sd <= m3d::EPSILON)
            {
                keep_pts[keep_n] = buf0[i];
                keep_dep[keep_n] = (sd < 0.0) ? (-sd) : epa_d;
                ++keep_n;
            }
        }

        out.kept_points .assign(keep_pts, keep_pts + keep_n);
        out.kept_depths .assign(keep_dep, keep_dep + keep_n);

        if (keep_n == 0)
        {
            out.manifold.num_points = 1;
            out.manifold.points[0].position          = epa_pt;
            out.manifold.points[0].penetration_depth = epa_d;
            out.manifold_built = true;
            return;
        }

        m3d::vec3   red_pts[4];
        m3d::scalar red_dep[4];
        const int red_n = rbc::manifold_detail::reduce_to_4(
            keep_pts, keep_dep, keep_n, red_pts, red_dep);

        out.manifold.num_points = red_n;
        for (int i = 0; i < red_n; ++i)
        {
            out.manifold.points[i].position          = red_pts[i];
            out.manifold.points[i].penetration_depth = red_dep[i];
        }
        out.manifold_built = true;
    }

} // namespace cdbg
