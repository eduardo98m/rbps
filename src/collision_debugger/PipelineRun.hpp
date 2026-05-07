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
        // Mirrors `rbc::ManifoldDebugCapture`. The reference + incident
        // polygons may carry up to ~16 vertices, but for legacy reasons
        // we keep them as fixed-size buffers of length kMaxFaceCorners
        // so the existing UI code reads them as plain arrays.
        int        ref_face_n = 0;
        m3d::vec3  ref_face[rbc::kMaxFaceCorners]{};
        int        inc_face_n = 0;
        m3d::vec3  inc_face[rbc::kMaxFaceCorners]{};
        std::vector<m3d::vec3>   post_clip_polygon;
        std::vector<m3d::vec3>   kept_points;
        std::vector<m3d::scalar> kept_depths;
        bool                     edge_edge = false;

        // ── Final manifold ──────────────────────────────────────────────
        rbc::ContactManifold manifold{};

        // ── Status flags ────────────────────────────────────────────────
        bool overlap        = false; // GJK reported Inside
        bool epa_converged  = false; // EPA returned Valid
        bool manifold_built = false; // generate_manifold ran to completion
    };

    // Run the full GJK → EPA → manifold pipeline and capture every
    // intermediate stage into `out`. The manifold stage delegates to
    // `rbc::generate_manifold` with a `ManifoldDebugCapture`, so the
    // visualization always tracks the actual algorithm rather than a
    // shadow copy that drifts.
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

        // ── 3. Manifold generation (delegates to the real implementation) ─
        rbc::ManifoldDebugCapture cap;
        rbc::generate_manifold(epa.normal, epa.depth, epa.contact_point,
                               sa, tf_a, sb, tf_b, out.manifold, &cap);
        out.manifold_built = true;
        out.edge_edge      = cap.edge_edge;

        const int ref_n = static_cast<int>(cap.ref_face.size());
        out.ref_face_n = (ref_n < rbc::kMaxFaceCorners) ? ref_n : rbc::kMaxFaceCorners;
        for (int i = 0; i < out.ref_face_n; ++i)
            out.ref_face[i] = cap.ref_face[i];

        const int inc_n = static_cast<int>(cap.inc_face.size());
        out.inc_face_n = (inc_n < rbc::kMaxFaceCorners) ? inc_n : rbc::kMaxFaceCorners;
        for (int i = 0; i < out.inc_face_n; ++i)
            out.inc_face[i] = cap.inc_face[i];

        out.post_clip_polygon = std::move(cap.post_clip_polygon);
        out.kept_points       = std::move(cap.kept_points);
        out.kept_depths       = std::move(cap.kept_depths);
    }

} // namespace cdbg
