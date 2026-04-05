#pragma once
#include "visr/Snapshot.hpp"
#include "rbps/API/World.hpp"
#include "rbc/shapes/ShapeTypes.hpp"

// ============================================================================
//  visr/SnapshotBuilder.hpp  (C++17 compatible)
//
//  rbc::Shape fix
//  ──────────────
//  rbc::Shape is a tagged struct, not a std::variant — std::visit won't work.
//  Switch on shape.tag and pull fields from the union.
//  Adapt the case labels and member names below to match ShapeTypes.hpp.
//
//  SoA id fix
//  ──────────
//  DynSoABase has no public id_of() method.
//  The stable ID for packed data index i is simply _ids[i] (public member).
// ============================================================================

namespace visr
{
    namespace detail
    {
        // ── Shape → ShapeSnap ─────────────────────────────────────────────────
        //  ADAPT: change ShapeTag::* and member field names to match your rbc::Shape.
        inline ShapeSnap to_shape_snap(const rbc::Shape &s)
        {
            switch (s.type)
            {
            case rbc::ShapeType::Sphere:
                return SpherSnap{s.sphere.radius};

            case rbc::ShapeType::Box:
                return BoxSnap{s.box.half_extents};

            case rbc::ShapeType::Capsule:
                return CapsuleSnap{s.capsule.radius, s.capsule.half_height};

            case rbc::ShapeType::Plane:
                // PlaneSnap::normal  → local-space unit normal
                // PlaneSnap::distance → the d offset (plane equation: n·x = d)
                // The visualizer rotates the normal by world_rot at draw time.
                return PlaneSnap{s.plane.normal, s.plane.d};

            case rbc::ShapeType::Cone:
                return ConeSnap{s.cone.base_radius, s.cone.half_height};

            case rbc::ShapeType::Ellipsoid:
                return EllipsoidSnap{s.ellipsoid.half_extents};

            case rbc::ShapeType::Heightmap:
                if (s.heightmap.data)
                    return HeightmapSnap{
                        s.heightmap.data->cols,
                        s.heightmap.data->rows,
                        static_cast<m3d::scalar>(s.heightmap.data->scale.x)};
                return HeightmapSnap{0u, 0u, 1.0};

                // case rbc::ShapeType::Mesh:
                //     return MeshSnap{
                //         s.mesh.data ? s.mesh.data->vert_count : 0u,
                //         s.mesh.data ? s.mesh.data->face_count : 0u
                //     };

            default:
                return SpherSnap{0.1};
            }
        }

    } // namespace detail

    // -------------------------------------------------------------------------
    //  build_snapshot
    // -------------------------------------------------------------------------
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