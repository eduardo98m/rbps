#include "rbps/CollisionPipeline.hpp"
#include "rbc/Dispatcher.hpp"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <numeric>

namespace rbps
{
    // =========================================================================
    //  Step 1 — Update broad phase AABBs from body transforms
    // =========================================================================

    void update_broad_phase_aabbs(rbps::ColliderCollection &cc,
                                  rbc::BroadPhaseState &bp,
                                  const BodyCollection &bc,
                                  m3d::scalar dt,
                                  bool use_velocity_expansion)
    {
        for (u_int32_t i = 0; i < cc.count(); ++i)
        {
            // ── STATIC BODY OPTIMISATION ──────────────────────────────────────
            // Static bodies never move. Their AABB was inserted at creation time
            // and never needs updating.
            if (cc.is_static[i])
                continue;

            // ── BROAD-PHASE BYPASS ────────────────────────────────────────────
            // Planes (and future heightmaps) have BP_INVALID_HANDLE — they never
            // enter the SAP endpoint list, so there is nothing to update.
            if (cc.bp_handle[i] == rbc::BP_INVALID_HANDLE)
                continue;

            const uint32_t bid = cc.body_id[i];
            const m3d::vec3 body_pos = bc.position[bid];
            const m3d::quat body_rot = bc.orientation[bid];

            const m3d::tf world_tf = rbps::collider_world_tf(cc, i, body_pos, body_rot);
            const rbc::AABB tight = rbc::compute_aabb(cc.shape[i], world_tf);

            if (use_velocity_expansion)
            {
                rbc::broad_phase_move_swept(bp, cc.bp_handle[i],
                                            tight,
                                            bc.linear_velocity[bid],
                                            dt);
            }
            else
            {
                rbc::broad_phase_move(bp, cc.bp_handle[i], tight);
            }
        }
    }

    // =========================================================================
    //  Helper: emit one contact into the ContactList
    // =========================================================================

    static void emit_contact(ContactList &out,
                             const rbc::ContactManifold &manifold,
                             uint32_t ba, uint32_t bb,
                             uint32_t ca, uint32_t cb,
                             const rbps::ColliderCollection &cc,
                             const BodyCollection &bc)
    {
        for (uint32_t i = 0; i < static_cast<uint32_t>(manifold.num_points); ++i)
        {
            const rbc::ContactPoint &c = manifold.points[i];
            const m3d::scalar half_depth = c.penetration_depth * 0.5;
            const m3d::vec3 p_on_a = c.position + manifold.normal * half_depth;
            const m3d::vec3 p_on_b = c.position - manifold.normal * half_depth;

            const m3d::vec3 r_a_wc = p_on_a - bc.position[ba];
            const m3d::vec3 r_b_wc = p_on_b - bc.position[bb];
            const m3d::vec3 r_a_local = m3d::rotate(m3d::conjugate(bc.orientation[ba]), r_a_wc);
            const m3d::vec3 r_b_local = m3d::rotate(m3d::conjugate(bc.orientation[bb]), r_b_wc);

            const m3d::scalar rest = m3d::min(cc.restitution[ca], cc.restitution[cb]);
            const m3d::scalar sf = (cc.static_friction[ca] + cc.static_friction[cb]) * 0.5;
            const m3d::scalar df = (cc.dynamic_friction[ca] + cc.dynamic_friction[cb]) * 0.5;

            out.body_a.push_back(ba);
            out.body_b.push_back(bb);
            out.collider_a.push_back(ca);
            out.collider_b.push_back(cb);
            out.r_a_local.push_back(r_a_local);
            out.r_b_local.push_back(r_b_local);
            out.normal.push_back(manifold.normal);
            out.restitution.push_back(rest);
            out.static_friction.push_back(sf);
            out.dynamic_friction.push_back(df);
            out.normal_lambda.push_back(0.0);
            out.tangent_lambda.push_back(0.0);
            out.point_on_a.push_back(p_on_a);
            out.point_on_b.push_back(p_on_b);
            out.penetration_depth.push_back(c.penetration_depth);
            out.collision.push_back(false);
            out.relative_velocity.push_back(0.0);
            out.normal_force.push_back(m3d::vec3(0));
            out.tangent_force.push_back(m3d::vec3(0));
            ++out.n_contacts;
        }
    }

    // =========================================================================
    //  Step 3 — Narrow phase: SAP pairs + plane loop
    // =========================================================================

    void run_narrow_phase(const rbc::BroadPhaseState &bp,
                          const rbps::ColliderCollection &cc,
                          const BodyCollection &bc,
                          ContactList &out)
    {
        // ── A. SAP candidate pairs ─────────────────────────────────────────────
        for (const rbc::BroadPhasePair &pair : bp.pairs)
        {
            const u_int32_t ca = pair.id_a;
            const u_int32_t cb = pair.id_b;

            if (ca >= cc.count() || cb >= cc.count())
                continue;

            const uint32_t ba = cc.body_id[ca];
            const uint32_t bb = cc.body_id[cb];

            // Static-static: will never penetrate further — skip.
            if (cc.is_static[ca] && cc.is_static[cb])
                continue;

            // Same body: composite shapes on one body must not self-collide.
            if (ba == bb)
                continue;

            const m3d::tf tf_a = rbps::collider_world_tf(cc, ca, bc.position[ba], bc.orientation[ba]);
            const m3d::tf tf_b = rbps::collider_world_tf(cc, cb, bc.position[bb], bc.orientation[bb]);

            rbc::ContactManifold manifold;
            if (rbc::dispatch(cc.shape[ca], tf_a, cc.shape[cb], tf_b, manifold))
                emit_contact(out, manifold, ba, bb,
                             static_cast<uint32_t>(ca),
                             static_cast<uint32_t>(cb), cc, bc);
        }

        // ── B. Plane (and future heightmap) loop ──────────────────────────────
        //
        //  Planes bypass the SAP entirely (BP_INVALID_HANDLE).  We test every
        //  plane collider against every non-plane collider.  This is O(P × N)
        //  where P = number of planes (almost always tiny: 0–4) and N = total
        //  colliders.  The cost is negligible for any practical scene.
        //
        //  Filters applied identically to the SAP path:
        //    • static-static pairs skipped
        //    • same-body pairs skipped
        //    • plane vs plane skipped (always returns false anyway)
        for (uint32_t pi = 0; pi < cc.count(); ++pi)
        {
            // Only process plane colliders in this loop.
            if (!cc.shape[pi].is<rbc::Plane>())
                continue;

            const uint32_t bp_body = cc.body_id[pi];
            const m3d::tf tf_plane = rbps::collider_world_tf(
                cc, pi, bc.position[bp_body], bc.orientation[bp_body]);

            for (uint32_t di = 0; di < cc.count(); ++di)
            {
                // Don't test the plane against itself or another plane.
                if (di == pi)
                    continue;
                if (cc.shape[di].is<rbc::Plane>())
                    continue;

                const uint32_t d_body = cc.body_id[di];

                // Static-static: nothing moves — skip.
                if (cc.is_static[pi] && cc.is_static[di])
                    continue;

                // Composite shapes on the same body must not self-collide.
                if (bp_body == d_body)
                    continue;

                const m3d::tf tf_dyn = rbps::collider_world_tf(
                    cc, di, bc.position[d_body], bc.orientation[d_body]);

                // Dispatch: the plane is always shape B so the normal convention
                // (A→B, i.e. shape toward plane) is consistent with the SAP path.
                rbc::ContactManifold manifold;
                if (rbc::dispatch(cc.shape[di], tf_dyn, cc.shape[pi], tf_plane, manifold))
                    emit_contact(out, manifold,
                                 d_body, bp_body,
                                 static_cast<uint32_t>(di),
                                 static_cast<uint32_t>(pi),
                                 cc, bc);
            }
        }
    }

    // =========================================================================
    //  Full pipeline
    // =========================================================================

    void run_collision_pipeline(rbps::ColliderCollection &cc,
                                rbc::BroadPhaseState &bp,
                                const BodyCollection &bc,
                                m3d::scalar dt,
                                ContactList &contacts_out,
                                const CollisionPipelineConfig &cfg)
    {
        contacts_out.clear();
        contacts_out.reserve(bp.pairs.size() * 2);

        update_broad_phase_aabbs(cc, bp, bc, dt, cfg.use_velocity_expansion);
        rbc::broad_phase_update(bp);
        run_narrow_phase(bp, cc, bc, contacts_out);
    }

    // =========================================================================
    //  Graph coloring — independent contact groups for parallel solving
    // =========================================================================

    std::vector<std::vector<u_int32_t>>
    get_collision_groups(const ContactList &contacts,
                         const BodyCollection &bc)
    {
        if (contacts.n_contacts == 0)
            return {};

        const u_int32_t n = contacts.n_contacts;

        std::unordered_map<uint32_t, std::vector<u_int32_t>> body_to_contacts;
        body_to_contacts.reserve(n * 2);

        for (u_int32_t i = 0; i < n; ++i)
        {
            if (bc.type[contacts.body_a[i]] == DYNAMIC)
                body_to_contacts[contacts.body_a[i]].push_back(i);
            if (bc.type[contacts.body_b[i]] == DYNAMIC)
                body_to_contacts[contacts.body_b[i]].push_back(i);
        }

        std::vector<u_int32_t> color(n, UINT32_MAX);

        std::vector<u_int32_t> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](u_int32_t a, u_int32_t b)
                  {
            auto deg = [&](u_int32_t ci) {
                u_int32_t d = 0;
                auto it_a = body_to_contacts.find(contacts.body_a[ci]);
                auto it_b = body_to_contacts.find(contacts.body_b[ci]);
                if (it_a != body_to_contacts.end()) d += it_a->second.size();
                if (it_b != body_to_contacts.end()) d += it_b->second.size();
                return d;
            };
            return deg(a) > deg(b); });

        for (u_int32_t ci : order)
        {
            std::unordered_set<u_int32_t> used_colors;
            for (uint32_t bid : {contacts.body_a[ci], contacts.body_b[ci]})
            {
                auto it = body_to_contacts.find(bid); // miss = static body, no edge
                if (it == body_to_contacts.end())
                    continue;
                for (u_int32_t neighbor : it->second)
                {
                    if (neighbor != ci && color[neighbor] != SIZE_MAX)
                        used_colors.insert(color[neighbor]);
                }
            }
            u_int32_t c = 0;
            while (used_colors.count(c))
                ++c;
            color[ci] = c;
        }

        std::unordered_map<u_int32_t, std::vector<u_int32_t>> color_groups;
        for (u_int32_t i = 0; i < n; ++i)
            color_groups[color[i]].push_back(i);

        std::vector<std::vector<u_int32_t>> groups;
        groups.reserve(color_groups.size());
        for (auto &[_, g] : color_groups)
            groups.push_back(std::move(g));

        std::sort(groups.begin(), groups.end(),
                  [](const auto &a, const auto &b)
                  { return a.size() > b.size(); });

        return groups;
    }

} // namespace rbps