#pragma once
#include <cstdint>
#include <vector>
#include <variant>
#include <math3d/math3d.hpp>

/**
 * @file Snapshot.hpp
 * @brief Plain data types that flow from the physics engine to the visualizer.
 * @ingroup visr
 *
 * No `rbps` or `rbc` types leak through this layer — the visualizer only
 * ever sees `*Snap` PODs. Shape parameters are captured as a
 * `std::variant<>` of plain structs; the builder (in
 * [SnapshotBuilder.hpp](SnapshotBuilder.hpp)) converts `rbc::Shape →
 * ShapeSnap` via `std::visit`, so adding a new `rbc` shape only requires
 * a new arm there.
 *
 * IDs in every snapshot are the stable user-facing IDs from the engine
 * (the same values returned by `create_body` / `create_collider`), NOT
 * packed slot indices. The builder converts at snapshot time.
 */

namespace visr
{
    /** @brief POD: sphere shape parameters. @ingroup visr */
    struct SpherSnap    { m3d::scalar radius; };
    /** @brief POD: oriented-box shape parameters. @ingroup visr */
    struct BoxSnap      { m3d::vec3   half_extents; };
    /** @brief POD: capsule shape parameters. @ingroup visr */
    struct CapsuleSnap  { m3d::scalar radius; m3d::scalar half_height; };
    /** @brief POD: infinite plane (`normal · x = distance`). @ingroup visr */
    struct PlaneSnap    { m3d::vec3   normal; m3d::scalar distance; };
    /** @brief POD: cone shape parameters. @ingroup visr */
    struct ConeSnap     { m3d::scalar radius; m3d::scalar height; };
    /** @brief POD: ellipsoid shape parameters. @ingroup visr */
    struct EllipsoidSnap{ m3d::vec3   semi_axes; };
    /** @brief POD: heightmap dimensions (height data is NOT snapshotted). @ingroup visr */
    struct HeightmapSnap{ uint32_t    cols; uint32_t rows;
                          m3d::scalar cell_size; };
    /** @brief POD: triangle mesh dimensions (vertex/face data is NOT snapshotted). @ingroup visr */
    struct MeshSnap     { uint32_t    vertex_count; uint32_t face_count; };

    /**
     * @brief Variant over all `*Snap` shape PODs.
     *
     * Mirror of `rbc::Shape` for the visualizer. To add a new shape:
     * extend this variant **and** add an arm in
     * [SnapshotBuilder.hpp](SnapshotBuilder.hpp)'s `ShapeToSnap`.
     *
     * @ingroup visr
     */
    using ShapeSnap = std::variant<
        SpherSnap,
        BoxSnap,
        CapsuleSnap,
        PlaneSnap,
        ConeSnap,
        EllipsoidSnap,
        HeightmapSnap,
        MeshSnap
    >;

    /**
     * @brief Per-body snapshot (POD copy of `BodyCollection` row data).
     * @ingroup visr
     */
    struct BodySnap
    {
        uint32_t      id;
        bool          is_static;

        m3d::vec3     position;
        m3d::quat     orientation;
        m3d::vec3     linear_velocity;
        m3d::vec3     angular_velocity;
        m3d::vec3     force;
        m3d::vec3     torque;
        m3d::scalar   mass;
        m3d::scalar   inverse_mass;
    };

    /**
     * @brief Per-collider snapshot. `world_pos` / `world_rot` are pre-composed
     *        world-space transforms (body pose × local offset), ready for the renderer.
     * @ingroup visr
     */
    struct ColliderSnap
    {
        uint32_t      id;
        uint32_t      body_id;
        bool          is_static;

        m3d::vec3     world_pos;
        m3d::quat     world_rot;

        m3d::scalar   restitution;
        m3d::scalar   static_friction;
        m3d::scalar   dynamic_friction;

        ShapeSnap     shape;
    };

    /**
     * @brief Per-contact snapshot, captured after the last substep.
     *
     * `normal_lambda` / `tangent_lambda` are the accumulated Lagrange
     * multipliers — useful for plotting contact forces or colour-coding
     * contact points by load.
     *
     * @ingroup visr
     */
    struct ContactSnap
    {
        uint32_t      collider_a;
        uint32_t      collider_b;
        uint32_t      body_a;
        uint32_t      body_b;

        m3d::vec3     point_on_a;       // world space
        m3d::vec3     point_on_b;       // world space
        m3d::vec3     normal;           // world space, points a→b
        m3d::scalar   penetration_depth;

        m3d::scalar   normal_lambda;    // accumulated normal impulse magnitude
        m3d::scalar   tangent_lambda;   // accumulated friction impulse magnitude
        m3d::vec3     normal_force;
        m3d::vec3     tangent_force;
        m3d::scalar   relative_velocity;

        bool          active;           // false → bodies separated this frame
    };

    /** @brief Joint kind enum mirrored from `rbps::JointType`. @ingroup visr */
    enum class JointTypeSnap : uint8_t { Prismatic, Revolute, Fixed };

    /**
     * @brief Per-joint snapshot (POD copy of `JointCollection` row data).
     *
     * `current_position` is the slide distance for `Prismatic`, the angle
     * for `Revolute`, unused for `Fixed`.
     *
     * @ingroup visr
     */
    struct JointSnap
    {
        uint32_t        id;
        JointTypeSnap   type;
        uint32_t        body_a;
        uint32_t        body_b;

        m3d::vec3       anchor_a;       // world-space attachment point on body a
        m3d::vec3       anchor_b;       // world-space attachment point on body b
        m3d::vec3       main_axis;

        m3d::scalar     current_position;  // slide distance or angle
        m3d::scalar     lower_limit;
        m3d::scalar     upper_limit;
        bool            limited;

        m3d::scalar     damping;
    };

    /** @brief Constraint kind enum mirrored from `rbps::ConstraintType`. @ingroup visr */
    enum class ConstraintTypeSnap : uint8_t { Positional, Rotational };

    /**
     * @brief Per-constraint snapshot (one entry per XPBD constraint row).
     * @ingroup visr
     */
    struct ConstraintSnap
    {
        uint32_t            id;
        ConstraintTypeSnap  type;
        uint32_t            body_a;
        uint32_t            body_b;
        m3d::vec3           direction;
        m3d::scalar         magnitude;
        m3d::scalar         lambda;
        m3d::scalar         compliance;
    };

    /**
     * @brief Top-level frame — everything the visualizer needs for one tick.
     *
     * Built once per physics step by `build_snapshot` (see
     * [SnapshotBuilder.hpp](SnapshotBuilder.hpp)) and shipped through the
     * transport. Pure value type — safe to deep-copy across threads.
     *
     * @ingroup visr
     */
    struct FrameSnapshot
    {
        uint64_t    frame_index  = 0;
        double      sim_time     = 0.0;
        double      timestep     = 0.0;
        int         substeps     = 0;

        std::vector<BodySnap>       bodies;
        std::vector<ColliderSnap>   colliders;
        std::vector<ContactSnap>    contacts;
        std::vector<JointSnap>      joints;
        std::vector<ConstraintSnap> constraints;

        // Broad-phase pair count — useful for perf graphs
        uint32_t    broad_phase_pairs = 0;
    };

} // namespace visr