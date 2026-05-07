// Contact manifold generation — literal port of the reference's
// `convex_convex_contact_manifold` (clipping.cpp in the C reference),
// adapted to ConvexHullData adjacency tables and m3d::tf transforms.
//
// Pipeline for ConvexHull vs ConvexHull (the only path we treat as
// fully implemented; other pairs fall back to a single-point contact at
// the EPA witness):
//
//   1. Pick support vertex on each hull along ±EPA normal.
//   2. From each support vertex's `vertex_to_polys`, pick the polygon
//      whose world-space normal is most aligned with the EPA normal.
//   3. Enumerate (deg_a × deg_b) edge-pair candidates from each support's
//      `vertex_to_neighbors` and pick the pair whose cross-product is
//      most aligned with the EPA normal.
//   4. Compare alignment scores; if the edge score beats both face
//      scores by EPSILON, take the edge-edge feature path (skew-line
//      closest points → 1 contact). Otherwise face-face.
//   5. Face-face: build boundary planes from `poly_to_polys[ref_poly]`
//      (each neighbor polygon contributes a plane whose normal is the
//      inverted neighbor face normal — this is the bit that fixes the
//      manifold bug; the previous pipeline used cross(ref_n, edge)
//      which is not the correct boundary plane for non-orthogonal
//      hulls). Sutherland-Hodgman the incident polygon, then clip
//      against the reference plane (remove only). Per surviving point,
//      compute penetration along the EPA normal and emit if positive.
//   6. Reduce to <= 4 contacts.
//
// No alignment-threshold short-circuit. Faces near-orthogonal to the
// EPA normal route through the edge-edge branch in step 4.

#include "rbc/gjk/ContactManifoldGenerator.hpp"
#include "rbc/gjk/Clipping.hpp"
#include "rbc/shapes/ConvexHull.hpp"
#include <cfloat>
#include <vector>

namespace rbc
{
    namespace
    {
        const ConvexHullData *as_convex_hull(const Shape &s)
        {
            if (const ConvexHull *h = s.as<ConvexHull>())
                return h->data;
            return nullptr;
        }

        bool hull_has_adjacency(const ConvexHullData *h)
        {
            return h && h->poly_count > 0
                && h->poly_to_polys_offsets && h->poly_to_polys_indices
                && h->vertex_to_polys_offsets && h->vertex_to_polys_indices
                && h->vertex_to_neighbors_offsets && h->vertex_to_neighbors_indices;
        }

        uint32_t support_vertex_index(const ConvexHullData *h,
                                       const m3d::tf &tf,
                                       const m3d::vec3 &world_dir)
        {
            const m3d::vec3 local_dir = tf.inverse_rotate_vector(world_dir);
            uint32_t best = 0;
            m3d::scalar best_d = m3d::dot(h->vertices[0], local_dir);
            for (uint32_t i = 1; i < h->vert_count; ++i)
            {
                const m3d::scalar d = m3d::dot(h->vertices[i], local_dir);
                if (d > best_d) { best_d = d; best = i; }
            }
            return best;
        }

        // Among the polygons touching `support_vertex_idx`, return the
        // one whose world-space normal is most aligned with `world_dir`.
        uint32_t pick_best_polygon(const ConvexHullData *h,
                                    const m3d::tf &tf,
                                    uint32_t support_vertex_idx,
                                    const m3d::vec3 &world_dir,
                                    m3d::scalar &out_alignment)
        {
            const uint32_t s = h->vertex_to_polys_offsets[support_vertex_idx];
            const uint32_t e = h->vertex_to_polys_offsets[support_vertex_idx + 1];

            uint32_t best = (s < e) ? h->vertex_to_polys_indices[s] : 0;
            m3d::scalar best_d = -DBL_MAX;
            for (uint32_t i = s; i < e; ++i)
            {
                const uint32_t p = h->vertex_to_polys_indices[i];
                const m3d::vec3 wn = tf.rotate_vector(h->poly_normals[p]);
                const m3d::scalar d = m3d::dot(wn, world_dir);
                if (d > best_d) { best_d = d; best = p; }
            }
            out_alignment = best_d;
            return best;
        }

        struct EdgePair
        {
            uint32_t a0, a1; ///< Vertex indices on hull A.
            uint32_t b0, b1; ///< Vertex indices on hull B.
        };

        // Best (deg_a × deg_b) edge pair. Score is the dot of the unit
        // cross of the two edges with `world_normal` (also tested with
        // the negated cross, since edge orientation is arbitrary).
        bool pick_best_edge_pair(const ConvexHullData *ha, const m3d::tf &tfa, uint32_t sa,
                                 const ConvexHullData *hb, const m3d::tf &tfb, uint32_t sb,
                                 const m3d::vec3 &world_normal,
                                 EdgePair &out, m3d::vec3 &out_normal,
                                 m3d::scalar &out_alignment)
        {
            const m3d::vec3 wsa = tfa.transform_point(ha->vertices[sa]);
            const m3d::vec3 wsb = tfb.transform_point(hb->vertices[sb]);

            const uint32_t na_s = ha->vertex_to_neighbors_offsets[sa];
            const uint32_t na_e = ha->vertex_to_neighbors_offsets[sa + 1];
            const uint32_t nb_s = hb->vertex_to_neighbors_offsets[sb];
            const uint32_t nb_e = hb->vertex_to_neighbors_offsets[sb + 1];

            if (na_s >= na_e || nb_s >= nb_e)
                return false;

            m3d::scalar best_d = -DBL_MAX;
            bool found = false;

            for (uint32_t i = na_s; i < na_e; ++i)
            {
                const uint32_t na = ha->vertex_to_neighbors_indices[i];
                const m3d::vec3 wa_n = tfa.transform_point(ha->vertices[na]);
                const m3d::vec3 ea = wsa - wa_n;
                for (uint32_t j = nb_s; j < nb_e; ++j)
                {
                    const uint32_t nb = hb->vertex_to_neighbors_indices[j];
                    const m3d::vec3 wb_n = tfb.transform_point(hb->vertices[nb]);
                    const m3d::vec3 eb = wsb - wb_n;

                    const m3d::vec3 c = m3d::cross(ea, eb);
                    const m3d::scalar c_len = m3d::length(c);
                    if (c_len < m3d::EPSILON) continue;
                    const m3d::vec3 cn = c / c_len;

                    const m3d::scalar d_pos = m3d::dot(cn, world_normal);
                    if (d_pos > best_d)
                    {
                        best_d = d_pos;
                        out.a0 = sa; out.a1 = na;
                        out.b0 = sb; out.b1 = nb;
                        out_normal = cn;
                        found = true;
                    }
                    const m3d::scalar d_neg = -d_pos;
                    if (d_neg > best_d)
                    {
                        best_d = d_neg;
                        out.a0 = sa; out.a1 = na;
                        out.b0 = sb; out.b1 = nb;
                        out_normal = -cn;
                        found = true;
                    }
                }
            }

            out_alignment = best_d;
            return found;
        }

        void emit_single_contact(ContactManifold &manifold,
                                 const m3d::vec3 &normal,
                                 const m3d::vec3 &point,
                                 m3d::scalar depth)
        {
            manifold.normal = normal;
            manifold.num_points = 1;
            manifold.points[0].position          = point;
            manifold.points[0].penetration_depth = depth;
        }

        void manifold_convex_convex(const m3d::vec3 &epa_normal,
                                     m3d::scalar epa_depth,
                                     const m3d::vec3 &epa_contact,
                                     const ConvexHullData *ha, const m3d::tf &tf_a,
                                     const ConvexHullData *hb, const m3d::tf &tf_b,
                                     ContactManifold &manifold)
        {
            manifold.normal = epa_normal;
            manifold.num_points = 0;

            const m3d::vec3 inv_normal = -epa_normal;

            // 1. Support vertices on each hull.
            const uint32_t sa = support_vertex_index(ha, tf_a, epa_normal);
            const uint32_t sb = support_vertex_index(hb, tf_b, inv_normal);

            // 2. Best polygon on each.
            m3d::scalar align_a, align_b;
            const uint32_t poly_a = pick_best_polygon(ha, tf_a, sa, epa_normal,  align_a);
            const uint32_t poly_b = pick_best_polygon(hb, tf_b, sb, inv_normal,  align_b);

            // 3. Best edge pair.
            EdgePair edges{};
            m3d::vec3 edge_n(0, 0, 0);
            m3d::scalar align_e = -DBL_MAX;
            const bool has_edge = pick_best_edge_pair(ha, tf_a, sa, hb, tf_b, sb,
                                                      epa_normal, edges, edge_n, align_e);

            constexpr m3d::scalar EPS = 1e-4;

            // 4. Edge-edge wins?
            if (has_edge && align_e > align_a + EPS && align_e > align_b + EPS)
            {
                m3d::vec3 l1, l2;
                const m3d::vec3 p1 = tf_a.transform_point(ha->vertices[edges.a0]);
                const m3d::vec3 d1 = tf_a.transform_point(ha->vertices[edges.a1]) - p1;
                const m3d::vec3 p2 = tf_b.transform_point(hb->vertices[edges.b0]);
                const m3d::vec3 d2 = tf_b.transform_point(hb->vertices[edges.b1]) - p2;

                if (clipping::skew_line_closest_points(p1, d1, p2, d2, l1, l2))
                {
                    emit_single_contact(manifold, epa_normal, (l1 + l2) * 0.5, epa_depth);
                    return;
                }
                emit_single_contact(manifold, epa_normal, epa_contact, epa_depth);
                return;
            }

            // 5. Face-face. Pick reference and incident.
            const bool ref_is_a = align_a > align_b;
            const ConvexHullData *href = ref_is_a ? ha : hb;
            const ConvexHullData *hinc = ref_is_a ? hb : ha;
            const m3d::tf &tf_ref      = ref_is_a ? tf_a : tf_b;
            const m3d::tf &tf_inc      = ref_is_a ? tf_b : tf_a;
            const uint32_t ref_poly    = ref_is_a ? poly_a : poly_b;
            const uint32_t inc_poly    = ref_is_a ? poly_b : poly_a;

            const m3d::vec3 ref_n_world = tf_ref.rotate_vector(href->poly_normals[ref_poly]);

            // Incident polygon vertices in world space.
            const uint32_t inc_s = hinc->poly_offsets[inc_poly];
            const uint32_t inc_e = hinc->poly_offsets[inc_poly + 1];
            const int inc_n = static_cast<int>(inc_e - inc_s);
            if (inc_n < 3)
            {
                emit_single_contact(manifold, epa_normal, epa_contact, epa_depth);
                return;
            }

            std::vector<m3d::vec3> inc_world(inc_n);
            for (int i = 0; i < inc_n; ++i)
                inc_world[i] = tf_inc.transform_point(
                    hinc->vertices[hinc->poly_indices[inc_s + i]]);

            // Reference polygon's first vertex in world (used as plane point).
            const uint32_t ref_first_v = href->poly_indices[href->poly_offsets[ref_poly]];
            const m3d::vec3 ref_pt_world = tf_ref.transform_point(href->vertices[ref_first_v]);

            // Boundary planes from neighbor polygons.
            std::vector<clipping::Plane> planes;
            const uint32_t n_s = href->poly_to_polys_offsets[ref_poly];
            const uint32_t n_e = href->poly_to_polys_offsets[ref_poly + 1];
            planes.reserve(n_e - n_s);
            for (uint32_t i = n_s; i < n_e; ++i)
            {
                const uint32_t neighbor_p = href->poly_to_polys_indices[i];
                clipping::Plane pl;
                const uint32_t first_v = href->poly_indices[href->poly_offsets[neighbor_p]];
                pl.point  = tf_ref.transform_point(href->vertices[first_v]);
                pl.normal = -tf_ref.rotate_vector(href->poly_normals[neighbor_p]);
                planes.push_back(pl);
            }

            // Sutherland-Hodgman clip.
            const int clip_cap = (inc_n + static_cast<int>(planes.size())) * 2 + 8;
            std::vector<m3d::vec3> clipped(static_cast<size_t>(clip_cap));
            int n_clipped = static_cast<int>(inc_world.size());
            if (!planes.empty())
            {
                n_clipped = clipping::sutherland_hodgman(
                    inc_world.data(), static_cast<int>(inc_world.size()),
                    planes.data(),     static_cast<int>(planes.size()),
                    clipped.data(),    clip_cap,
                    /*remove_only=*/false);
            }
            else
            {
                for (int i = 0; i < n_clipped; ++i) clipped[i] = inc_world[i];
            }

            // Final clip against the reference plane (remove points above it).
            clipping::Plane ref_plane;
            ref_plane.point  = ref_pt_world;
            ref_plane.normal = -ref_n_world;

            std::vector<m3d::vec3> final_pts(static_cast<size_t>(clip_cap + 4));
            const int n_final = clipping::sutherland_hodgman(
                clipped.data(), n_clipped,
                &ref_plane,     1,
                final_pts.data(), static_cast<int>(final_pts.size()),
                /*remove_only=*/true);

            // Per surviving point, compute penetration along the EPA normal.
            std::vector<m3d::vec3>   keep_pts;
            std::vector<m3d::scalar> keep_dep;
            keep_pts.reserve(static_cast<size_t>(n_final));
            keep_dep.reserve(static_cast<size_t>(n_final));

            for (int i = 0; i < n_final; ++i)
            {
                const m3d::vec3 p = final_pts[i];
                const m3d::scalar d_to_plane = m3d::dot(p - ref_pt_world, ref_n_world);
                const m3d::vec3 diff = ref_n_world * d_to_plane;
                m3d::scalar pen;
                if (ref_is_a)
                    pen =  m3d::dot(diff, epa_normal);
                else
                    pen = -m3d::dot(diff, epa_normal);

                if (pen < 0.0)
                {
                    keep_pts.push_back(p);
                    keep_dep.push_back(-pen);
                }
            }

            if (keep_pts.empty())
            {
                emit_single_contact(manifold, epa_normal, epa_contact, epa_depth);
                return;
            }

            m3d::vec3   red_pts[4];
            m3d::scalar red_dep[4];
            const int red_n = manifold_detail::reduce_to_4(
                keep_pts.data(), keep_dep.data(),
                static_cast<int>(keep_pts.size()),
                red_pts, red_dep);

            manifold.num_points = red_n;
            for (int i = 0; i < red_n; ++i)
            {
                manifold.points[i].position          = red_pts[i];
                manifold.points[i].penetration_depth = red_dep[i];
            }
        }
    } // namespace

    void generate_manifold(const m3d::vec3 &epa_normal,
                           m3d::scalar epa_depth,
                           const m3d::vec3 &epa_contact,
                           const Shape &shape_a,
                           const m3d::tf &tf_a,
                           const Shape &shape_b,
                           const m3d::tf &tf_b,
                           ContactManifold &manifold)
    {
        manifold.normal = epa_normal;
        manifold.num_points = 0;

        const ConvexHullData *ha = as_convex_hull(shape_a);
        const ConvexHullData *hb = as_convex_hull(shape_b);

        if (hull_has_adjacency(ha) && hull_has_adjacency(hb))
        {
            manifold_convex_convex(epa_normal, epa_depth, epa_contact,
                                    ha, tf_a, hb, tf_b, manifold);
            return;
        }

        // Non-ConvexHull pair (or hull missing adjacency / vertex-only):
        // emit the EPA witness as a single contact. Analytic dispatchers
        // cover the common non-ConvexHull pairs (sphere-sphere, sphere-box,
        // box-box, etc.); this fallback is what's left.
        manifold.num_points = 1;
        manifold.points[0].position          = epa_contact;
        manifold.points[0].penetration_depth = epa_depth;
    }

    bool gjk_epa_manifold(const Shape &sa,
                          const m3d::tf &tf_a,
                          const Shape &sb,
                          const m3d::tf &tf_b,
                          ContactManifold &manifold)
    {
        MinkowskiDiff md(&sa, &sb, tf_a, tf_b);

        m3d::vec3 guess = tf_b.pos - tf_a.pos;
        if (m3d::length_sq(guess) < m3d::EPSILON)
            guess = m3d::vec3(1.0, 0.0, 0.0);

        GJK gjk;
        if (gjk.evaluate(md, guess) != GJK::Inside)
            return false;

        EPA epa;
        if (epa.evaluate(gjk, md) != EPA::Valid)
            return false;

        generate_manifold(epa.normal, epa.depth, epa.contact_point,
                          sa, tf_a, sb, tf_b, manifold);
        return true;
    }

} // namespace rbc
