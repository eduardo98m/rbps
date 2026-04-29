#pragma once
#include "visr/Snapshot.hpp"
#include "rbps/API/World.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

/**
 * @file SnapshotBuilder.hpp
 * @brief Convert a `rbps::World` into a `FrameSnapshot` POD copy.
 * @ingroup visr
 *
 * `rbc::Shape` is a thin `std::variant` wrapper; the builder uses
 * `std::visit` with a visitor struct that has one `operator()` per
 * concrete shape. Each arm stands on its own — no `decltype`, no
 * `if constexpr`, no `switch`.
 */

namespace visr
{
    namespace detail
    {
        /**
         * @brief Visitor that maps each `rbc::Shape` alternative to its `*Snap` POD.
         * @ingroup visr
         * @ingroup internals
         */
        struct ShapeToSnap
        {
            ShapeSnap operator()(const rbc::Sphere &s) const
            {
                return SpherSnap{s.radius};
            }
            ShapeSnap operator()(const rbc::Box &b) const
            {
                return BoxSnap{b.half_extents};
            }
            ShapeSnap operator()(const rbc::Capsule &c) const
            {
                return CapsuleSnap{c.radius, c.half_height};
            }
            ShapeSnap operator()(const rbc::Plane &p) const
            {
                // PlaneSnap::normal   → local-space unit normal
                // PlaneSnap::distance → the d offset (plane equation: n·x = d)
                return PlaneSnap{p.normal, p.d};
            }
            ShapeSnap operator()(const rbc::Cone &c) const
            {
                return ConeSnap{c.base_radius, c.half_height};
            }
            ShapeSnap operator()(const rbc::Ellipsoid &e) const
            {
                return EllipsoidSnap{e.half_extents};
            }
            ShapeSnap operator()(const rbc::Heightmap &h) const
            {
                if (h.data)
                    return HeightmapSnap{
                        h.data->cols,
                        h.data->rows,
                        static_cast<m3d::scalar>(h.data->scale.x)};
                return HeightmapSnap{0u, 0u, 1.0};
            }
            ShapeSnap operator()(const rbc::Mesh &m) const
            {
                return MeshSnap{
                    m.data ? m.data->vert_count : 0u,
                    m.data ? m.data->face_count : 0u};
            }
        };

        /**
         * @brief Variant-level helper: dispatch a shape into its `ShapeSnap`.
         * @ingroup internals
         */
        inline ShapeSnap to_shape_snap(const rbc::Shape &s)
        {
            return std::visit(ShapeToSnap{}, s.v);
        }

    } // namespace detail

    /**
     * @brief Build a `FrameSnapshot` from the current state of `world`.
     *
     * Iterates every body / collider / contact / joint / constraint row
     * and copies the data into POD `*Snap` structs. Stable IDs (not
     * packed slot indices) are written, so the visualizer can keep its
     * selection state consistent across frames where a body has been
     * swap-popped.
     *
     * @param world       The physics world to snapshot.
     * @param frame_index Caller-managed frame counter (for the
     *                    `FrameSnapshot::frame_index` field).
     * @param sim_time    Caller-managed simulation time in seconds.
     *
     * @ingroup visr
     */
    inline FrameSnapshot build_snapshot(const rbps::World &world,
                                        uint64_t frame_index,
                                        double sim_time)
    {
        FrameSnapshot snap;
        snap.frame_index = frame_index;
        snap.sim_time = sim_time;
        snap.timestep = world.timestep;
        snap.substeps = world.substeps;
        snap.broad_phase_pairs = static_cast<uint32_t>(
            world.broad_phase_state.pairs.size());

        const auto &bc = world.bodies;
        const auto &cc = world.colliders;
        const auto &cl = world.contacts;
        const auto &jc = world.joints;
        const auto &co = world.constraints;

        // ── Bodies ───────────────────────────────────────────────────────────
        snap.bodies.reserve(bc.count());
        for (uint32_t i = 0; i < bc.count(); ++i)
        {
            BodySnap b;
            b.id = bc._ids[i]; // _ids[data_index] = stable ID
            b.is_static = (bc.type[i] == rbps::BodyType::STATIC);
            b.position = bc.position[i];
            b.orientation = bc.orientation[i];
            b.linear_velocity = bc.linear_velocity[i];
            b.angular_velocity = bc.angular_velocity[i];
            b.force = bc.force[i];
            b.torque = bc.torque[i];
            b.mass = bc.mass[i];
            b.inverse_mass = bc.inverse_mass[i];
            snap.bodies.push_back(b);
        }

        // ── Colliders ────────────────────────────────────────────────────────
        snap.colliders.reserve(cc.count());
        for (uint32_t i = 0; i < cc.count(); ++i)
        {
            const uint32_t body_slot = bc.index_of(cc.body_id[i]);
            const m3d::vec3 bpos = bc.position[body_slot];
            const m3d::quat brot = bc.orientation[body_slot];
            const m3d::tf wtf = rbps::collider_world_tf(cc, i, bpos, brot);

            ColliderSnap c;
            c.id = cc._ids[i];
            c.body_id = cc.body_id[i];
            c.is_static = cc.is_static[i];
            c.world_pos = wtf.pos;
            c.world_rot = wtf.rot;
            c.restitution = cc.restitution[i];
            c.static_friction = cc.static_friction[i];
            c.dynamic_friction = cc.dynamic_friction[i];
            c.shape = detail::to_shape_snap(cc.shape[i]);
            snap.colliders.push_back(c);
        }

        // ── Contacts ─────────────────────────────────────────────────────────
        snap.contacts.reserve(cl.n_contacts);
        for (uint32_t i = 0; i < cl.n_contacts; ++i)
        {
            ContactSnap ct;
            ct.body_a = bc._ids[cl.body_a[i]];
            ct.body_b = bc._ids[cl.body_b[i]];
            ct.collider_a = cc._ids[cl.collider_a[i]];
            ct.collider_b = cc._ids[cl.collider_b[i]];
            ct.point_on_a = cl.point_on_a[i];
            ct.point_on_b = cl.point_on_b[i];
            ct.normal = cl.normal[i];
            ct.penetration_depth = cl.penetration_depth[i];
            ct.normal_lambda = cl.normal_lambda[i];
            ct.tangent_lambda = cl.tangent_lambda[i];
            ct.normal_force = cl.normal_force[i];
            ct.tangent_force = cl.tangent_force[i];
            ct.relative_velocity = cl.relative_velocity[i];
            ct.active = cl.collision[i];
            snap.contacts.push_back(ct);
        }

        // ── Joints ───────────────────────────────────────────────────────────
        snap.joints.reserve(jc.count());
        for (uint32_t i = 0; i < jc.count(); ++i)
        {
            const uint32_t b1_slot = bc.index_of(jc.body_1[i]);
            const uint32_t b2_slot = bc.index_of(jc.body_2[i]);

            JointSnap j;
            j.id = jc._ids[i];
            j.body_a = jc.body_1[i];
            j.body_b = jc.body_2[i];
            j.main_axis = jc.main_axis[i];
            j.current_position = jc.current_position[i];
            j.lower_limit = jc.lower_limit[i];
            j.upper_limit = jc.upper_limit[i];
            j.limited = jc.limited[i];
            j.damping = jc.damping[i];

            j.anchor_a = bc.position[b1_slot] + m3d::rotate(bc.orientation[b1_slot], jc.r_1[i]);
            j.anchor_b = bc.position[b2_slot] + m3d::rotate(bc.orientation[b2_slot], jc.r_2[i]);

            switch (jc.type[i])
            {
            case rbps::JointType::PRISMATIC:
                j.type = JointTypeSnap::Prismatic;
                break;
            case rbps::JointType::REVOLUTE:
                j.type = JointTypeSnap::Revolute;
                break;
            case rbps::JointType::FIXED:
                j.type = JointTypeSnap::Fixed;
                break;
            }
            snap.joints.push_back(j);
        }

        // ── Constraints ──────────────────────────────────────────────────────
        snap.constraints.reserve(co.count());
        for (uint32_t i = 0; i < co.count(); ++i)
        {
            ConstraintSnap cs;
            cs.id = co._ids[i];
            cs.body_a = static_cast<uint32_t>(co.body_1[i]);
            cs.body_b = static_cast<uint32_t>(co.body_2[i]);
            cs.direction = co.direction[i];
            cs.magnitude = co.magnitude[i];
            cs.lambda = co.lambda[i];
            cs.compliance = co.compliance[i];
            cs.type = (co.type[i] == rbps::ConstraintType::POSITIONAL)
                          ? ConstraintTypeSnap::Positional
                          : ConstraintTypeSnap::Rotational;
            snap.constraints.push_back(cs);
        }

        return snap;
    }

} // namespace visr