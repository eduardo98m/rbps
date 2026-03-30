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
            // and never needs updating. Skip entirely — zero cost per static collider.
            if (cc.is_static[i])
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

        // ── PLANE / HEIGHTMAP HOOK ─────────────────────────────────────────
        // Planes and heightmaps cannot live in the SAP endpoint list because
        // their AABB is infinite (plane) or enormous (full heightmap).
        // The pattern is:
        //
        //   for (auto &plane : cc.static_planes)
        //       for (u_int32_t i = 0; i < cc.count(); ++i)
        //           if (!cc.is_static[i] && aabb_vs_plane(cc tight_aabb[i], plane))
        //               emit_pair_for_narrow_phase(i, plane);
        //
        // Heightmaps: keep a spatial hash of active cells; only register the cells
        // within range of dynamic bodies (each cell is a small AABB in SAP).
        // ──────────────────────────────────────────────────────────────────────
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

        for (uint32_t i = 0; i < manifold.num_points; ++i)
        {
            const rbc::ContactPoint &c = manifold.points[i];
            // World-space contact points split symmetrically along the normal.
            const m3d::scalar half_depth = c.penetration_depth * 0.5;
            const m3d::vec3 p_on_a = c.position + manifold.normal * half_depth;
            const m3d::vec3 p_on_b = c.position - manifold.normal * half_depth;

            // Convert world-space lever arms to body-LOCAL space (eq. 26).
            // conjugate(q) == inverse for unit quaternions.
            const m3d::vec3 r_a_wc = p_on_a - bc.position[ba];
            const m3d::vec3 r_b_wc = p_on_b - bc.position[bb];
            const m3d::vec3 r_a_local = m3d::rotate(m3d::conjugate(bc.orientation[ba]), r_a_wc);
            const m3d::vec3 r_b_local = m3d::rotate(m3d::conjugate(bc.orientation[bb]), r_b_wc);

            // Material mixing: average friction, minimum restitution.
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
            // Substep fields — sized but not filled; apply_constraint_position_level
            // reconstructs them each substep from r_local + current body transforms.
            out.point_on_a.push_back(p_on_a); // initial value, overwritten each substep
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
    //  Step 3 — Narrow phase filter + dispatch
    // =========================================================================

    void run_narrow_phase(const rbc::BroadPhaseState &bp,
                          const rbps::ColliderCollection &cc,
                          const BodyCollection &bc,
                          ContactList &out)
    {
        // Build a reverse map: user_id (ivc::ID.value) → collider slot index.
        // user_id was stored as ivc::ID.value in collider_add().
        // For simple sequential IDs, this is just the slot itself, but we
        // keep it explicit to be correct after remove/add cycles.
        //
        // OPTIMISATION NOTE: in a large scene you can cache this map and update
        // it only when colliders are added or removed.
        std::vector<u_int32_t> uid_to_slot(cc.count());
        for (u_int32_t i = 0; i < cc.count(); ++i)
            uid_to_slot[i] = i; // user_id == slot for now (see collider_add)

        for (const rbc::BroadPhasePair &pair : bp.pairs)
        {
            // Recover collider slot from user_id.
            // user_id was set to ivc::ID.value in collider_add(); since we
            // use ivc which compacts slots, slot == user_id here.
            const u_int32_t ca = pair.id_a;
            const u_int32_t cb = pair.id_b;

            if (ca >= cc.count() || cb >= cc.count())
                continue;

            const uint32_t ba = cc.body_id[ca];
            const uint32_t bb = cc.body_id[cb];

            // ── STATIC-STATIC FILTER ─────────────────────────────────────────
            // Two static bodies can never move, so even if their AABBs overlap
            // they will never penetrate further. Skip narrow phase entirely.
            // This is free — no GJK/EPA call, no contact emitted.
            if (cc.is_static[ca] && cc.is_static[cb])
                continue;

            // ── SAME-BODY FILTER ──────────────────────────────────────────────
            // Multiple colliders on the same body (composite shapes) should not
            // collide with each other.
            if (ba == bb)
                continue;

            // ── COLLISION GROUP FILTER ────────────────────────────────────────
            // TODO: add a collision_mask / collision_group per collider here
            // (e.g. robot links that should not self-collide).
            // if (!groups_should_collide(cc.group[ca], cc.group[cb])) continue;

            // Compute world transforms
            const m3d::tf tf_a = rbps::collider_world_tf(
                cc, ca, bc.position[ba], bc.orientation[ba]);
            const m3d::tf tf_b = rbps::collider_world_tf(
                cc, cb, bc.position[bb], bc.orientation[bb]);

            // ── NARROW PHASE DISPATCH ─────────────────────────────────────────
            // Routed through the function-pointer table in Dispatcher.hpp.
            // Analytic algorithms (SphereSphere, SphereBox, BoxBox) are tried
            // first via template specialisation; GJK/EPA is the fallback.
            rbc::ContactManifold manifold;
            if (rbc::dispatch(cc.shape[ca], tf_a,
                              cc.shape[cb], tf_b,
                              manifold))
            {
                emit_contact(out, manifold,
                             ba, bb,
                             static_cast<uint32_t>(ca),
                             static_cast<uint32_t>(cb),
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
        contacts_out.reserve(bp.pairs.size() * 2); // rough pre-alloc

        // 1. Push world-space AABBs into the broad phase
        update_broad_phase_aabbs(cc, bp, bc, dt, cfg.use_velocity_expansion);

        // 2. SAP sort + sweep → bp.pairs
        rbc::broad_phase_update(bp);

        // 3. Narrow phase: filter + dispatch → contacts_out
        run_narrow_phase(bp, cc, bc, contacts_out);
    }

    // =========================================================================
    //  Graph coloring — independent contact groups for parallel solving
    // =========================================================================
    //  Contacts are nodes in a graph; two contacts share an edge when they
    //  both involve the same DYNAMIC body (solving them together would cause
    //  a write race on that body's state).
    //
    //  KEY INSIGHT — static bodies do NOT create edges:
    //    The solver never writes back to a static body (inverse_mass == 0,
    //    no velocity update). Two contacts that share only a static body
    //    therefore have no actual data dependency and CAN be solved in parallel.
    //
    //    Without this rule, a ground plane shared by N independent tower stacks
    //    would merge all of them into a single sequential group even though no
    //    two towers interact with each other at all.
    //
    //  Returns groups sorted by size (largest first) so the parallel scheduler
    //  can use work-stealing effectively.
    // =========================================================================

    std::vector<std::vector<u_int32_t>>
    get_collision_groups(const ContactList &contacts,
                         const BodyCollection &bc)
    {
        if (contacts.n_contacts == 0)
            return {};

        const u_int32_t n = contacts.n_contacts;

        // Build adjacency: dynamic body_id → list of contact indices.
        // Static bodies are intentionally excluded: they create no edges.
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

        // Sort contacts: highest dynamic-body degree first (better greedy result).
        // FIX: original code had body_b[b] twice instead of body_a[b] + body_b[b].
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

        // Greedy coloring — only propagate edges through dynamic bodies.
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

        // Collect groups by color, sort descending by size.
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