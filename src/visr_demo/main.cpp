#include "visr/VisrApp.hpp"
#include "rbps/API/World.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/JointAPI.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include "imgui.h"

// ── Helpers ───────────────────────────────────────────────────────────────────

// Create a body whose inertia tensor is derived from its collider shape + mass.
// This is the canonical way to avoid having to compute it by hand.
static uint32_t make_sphere_body(rbps::World &w,
                                 m3d::vec3 pos,
                                 m3d::scalar mass,
                                 m3d::scalar radius,
                                 rbps::BodyType type = rbps::BodyType::DYNAMIC,
                                 m3d::scalar restitution = 0.5,
                                 m3d::scalar static_friction = 0.4,
                                 m3d::scalar dynamic_friction = 0.3)
{
    rbc::Sphere sphere{radius};

    rbps::BodyParams bp{};
    bp.type = type;
    bp.position = pos;
    bp.mass = mass;
    if (type == rbps::BodyType::DYNAMIC)
    {
        // I_shape is computed at unit density; scale to actual mass / volume.
        const m3d::scalar vol = rbc::compute_volume(sphere);
        bp.inertia_tensor = rbc::compute_inertia_tensor(sphere) * (mass / vol);
    }
    const uint32_t id = w.create_body(bp);

    rbps::ColliderParams cp{};
    cp.body_id = id;
    cp.local_pos = m3d::vec3{0, 0, 0};
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape = rbc::Shape(sphere);
    cp.restitution = restitution;
    cp.static_friction = static_friction;
    cp.dynamic_friction = dynamic_friction;
    w.create_collider(cp);

    return id;
}

static void build_fixed_joint_demo(rbps::World &w)
{
    // Static anchor (no collider needed — it's just a reference point)
    rbps::BodyParams ap{};
    ap.type = rbps::BodyType::STATIC;
    ap.position = m3d::vec3{-4.0, 4.0, 0.0};
    ap.mass = 0.0;
    const uint32_t anchor = w.create_body(ap);

    // Dynamic bob attached to anchor
    const uint32_t bob = make_sphere_body(w, {-4.0, 2.0, 0.0}, 1.0, 0.3);

    // Fixed joint: r_1 = {0,-2,0} so the bob hangs 2 m below the anchor.
    rbps::FixedJointParams fjp{};
    fjp.body_1 = anchor;
    fjp.body_2 = bob;
    fjp.r_1 = m3d::vec3{0, -2.0, 0};
    fjp.r_2 = m3d::vec3{0, 0.0, 0};
    w.create_fixed_joint(fjp);
}

// Revolute joint: classic pendulum.
// Axis = Z.  Bob starts displaced sideways so it actually swings.
static void build_revolute_joint_demo(rbps::World &w)
{
    // Static pivot
    rbps::BodyParams pp{};
    pp.type = rbps::BodyType::STATIC;
    pp.position = m3d::vec3{0.0, 4.0, 0.0};
    pp.mass = 0.0;
    const uint32_t pivot = w.create_body(pp);

    // Bob: displaced 1.8 m on X from the pivot so it has swing amplitude.
    const uint32_t bob = make_sphere_body(w, {1.8, 4.0, 0.0}, 20.0, 0.3);

    rbps::RevoluteJointParams rjp{};
    rjp.body_1 = pivot;
    rjp.body_2 = bob;
    rjp.aligned_axis = m3d::vec3{0, 0, 1}; // rotation axis
    rjp.limit_axis = m3d::vec3{1, 0, 0};   // reference for angle measurement
    rjp.r_1 = m3d::vec3{0, 0, 0};
    rjp.r_2 = m3d::vec3{-1.8, 0, 0}; // bob offset from its own COM
    rjp.limited = false;
    rjp.damping = 0.5;
    rjp.actuation_type = rbps::JointActuationType::FREE;
    w.create_revolute_joint(rjp);
}

// Prismatic joint: one static rail body + one bob that slides along Y.
// Give the bob an initial velocity so it actually moves.
static void build_prismatic_joint_demo(rbps::World &w)
{
    // Static rail anchor
    rbps::BodyParams rp{};
    rp.type = rbps::BodyType::STATIC;
    rp.position = m3d::vec3{4.0, 4.0, 0.0};
    rp.mass = 0.0;
    const uint32_t rail = w.create_body(rp);

    // Sliding bob — starts at the anchor position, will slide along Y.
    const uint32_t bob = make_sphere_body(w, {4.0, 4.0, 0.0}, 1.0, 0.3);

    // Give it an initial downward velocity so it slides visibly from frame 1.
    const uint32_t slot = w.bodies.index_of(bob);
    w.bodies.linear_velocity[slot] = m3d::vec3{0, -2.0, 0};

    rbps::PrismaticJointParams pjp{};
    pjp.body_1 = rail;
    pjp.body_2 = bob;
    pjp.moving_axis = m3d::vec3{0, 1, 0}; // slide along Y
    pjp.r_1 = m3d::vec3{0, 0, 0};
    pjp.r_2 = m3d::vec3{0, 0, 0};
    pjp.limited = true;
    pjp.lower_limit = -2.5;
    pjp.upper_limit = 2.5;
    pjp.damping = 0.1;
    w.create_prismatic_joint(pjp);
}

// Spawn a tetrahedron-shaped `ConvexHull` body so the new collider is
// visible in the visualizer. Vertex / face arrays have static-storage
// duration so they outlive the world (`ConvexHullData` is non-owning).
static void build_convex_hull_demo(rbps::World &w)
{
    static const m3d::vec3 tet_verts[4] = {
        m3d::vec3( 0.6,  0.6,  0.6),
        m3d::vec3(-0.6, -0.6,  0.6),
        m3d::vec3(-0.6,  0.6, -0.6),
        m3d::vec3( 0.6, -0.6, -0.6),
    };
    static const uint32_t tet_faces[4 * 3] = {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    static rbc::ConvexHullData *tet_hull =
        rbc::convex_hull_data_create(tet_verts, 4, tet_faces, 4);

    rbc::ConvexHull hull(tet_hull);

    rbps::BodyParams bp{};
    bp.type     = rbps::BodyType::DYNAMIC;
    bp.position = m3d::vec3{0.5, 5.0, 0.0};
    bp.mass     = 5.0;
    // I_shape is computed at unit density; scale to actual mass / volume.
    const m3d::scalar vol = rbc::compute_volume(hull);
    bp.inertia_tensor = rbc::compute_inertia_tensor(hull) * (bp.mass / vol);
    const uint32_t id = w.create_body(bp);

    rbps::ColliderParams cp{};
    cp.body_id          = id;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{1, 0, 0, 0};
    cp.shape            = rbc::Shape(hull);
    cp.restitution      = 0.2;
    cp.static_friction  = 0.5;
    cp.dynamic_friction = 0.4;
    w.create_collider(cp);
}

static void build_box_tower(rbps::World &w)
{
    double y_pos = 0.00;
    int num_boxes = 40;
    double k = 60.0;
    for (int i = 0; i < num_boxes; ++i)
    {
        rbc::Box box{{0.5, 0.5, 0.5}};
        rbps::BodyParams box_body_p{};
        box_body_p.type = rbps::BodyType::DYNAMIC;
        box_body_p.position = m3d::vec3{2.0, y_pos, 4.0};
        box_body_p.mass = 1000.0/ k ; // make each box lighter than the last so they fall at different rates
        // I_shape is computed at unit density; scale to actual mass / volume.
        const m3d::scalar vol_box = rbc::compute_volume(box);
        box_body_p.inertia_tensor = rbc::compute_inertia_tensor(box) * (box_body_p.mass / vol_box);
        const uint32_t box_id = w.create_body(box_body_p);
        rbps::ColliderParams box_cp{};
        box_cp.body_id = box_id;
        box_cp.local_pos = m3d::vec3{0, 0, 0};
        box_cp.local_rot = m3d::quat{1, 0, 0, 0};
        box_cp.shape = rbc::Shape(box);
        box_cp.restitution = 0.1;
        box_cp.static_friction = 0.99;
        box_cp.dynamic_friction = 0.99;
        w.create_collider(box_cp);
        y_pos += 1.0;
        k *= 2.0;
    }
}

// Static ground plane so free-falling spheres have something to land on.
static void build_ground(rbps::World &w)
{
    rbps::BodyParams gp{};
    gp.type = rbps::BodyType::STATIC;
    gp.position = m3d::vec3{0.0, -0.5, 0.0};
    const m3d::scalar tilt_deg = 0.0;
    const m3d::scalar tilt_rad = tilt_deg * M_PI / 180.0;
    gp.orientation = m3d::quat::from_axis_angle({0, 1, 1}, tilt_rad); // rotate box to lie flat
    gp.mass = 0.0;

    const uint32_t ground = w.create_body(gp);

    rbps::ColliderParams cp{};
    cp.body_id = ground;
    cp.local_pos = m3d::vec3{0, 0, 0};
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    // rbc::Shape::Plane();
    cp.shape = rbc::Shape(rbc::Plane({0.0, 1.0, 0.0}, 0.0)); // rbc::Shape(rbc::Box{{100.0, 0.5, 100.0}});

    cp.restitution = 0.3;
    cp.static_friction = 0.4;
    cp.dynamic_friction = 0.3;
    w.create_collider(cp);

    rbc::Sphere sphere{0.5};

    rbps::BodyParams bp{};
    bp.type = rbps::BodyType::DYNAMIC;
    bp.position = m3d::vec3{-2.0, 4.0, -4.01};
    bp.mass = 50.0;

    // I_shape is computed at unit density; scale to actual mass / volume.
    const m3d::scalar vol = rbc::compute_volume(sphere);
    bp.inertia_tensor = rbc::compute_inertia_tensor(sphere) * (bp.mass / vol);

    const uint32_t id = w.create_body(bp);

    rbps::ColliderParams ball_cp{};
    ball_cp.body_id = id;
    ball_cp.local_pos = m3d::vec3{0.0, 0.0, 0.0};
    ball_cp.local_rot = m3d::quat{1, 0, 0, 0};
    ball_cp.shape = rbc::Shape(sphere);
    ball_cp.restitution = 0.1;
    ball_cp.static_friction = 0.6;
    ball_cp.dynamic_friction = 0.4;
    w.create_collider(ball_cp);

    rbc::Capsule capsule{0.5, 0.25};
    rbps::BodyParams capsule_body_p{};
    capsule_body_p.type = rbps::BodyType::DYNAMIC;
    capsule_body_p.position = m3d::vec3{-2.0, 3.0, -4.0};
    capsule_body_p.mass = 10.0; // make each box lighter than the last so they fall at different rates
    // I_shape is computed at unit density; scale to actual mass / volume.
    const m3d::scalar vol_capsule = rbc::compute_volume(capsule);
    capsule_body_p.inertia_tensor = rbc::compute_inertia_tensor(capsule) * (capsule_body_p.mass / vol_capsule);
    const uint32_t capsule_id = w.create_body(capsule_body_p);
    rbps::ColliderParams capsule_cp{};
    capsule_cp.body_id = capsule_id;
    capsule_cp.local_pos = m3d::vec3{0, 0, 0};
    capsule_cp.local_rot = m3d::quat{1, 0, 0, 0};
    capsule_cp.shape = rbc::Shape(capsule);
    capsule_cp.restitution = 0.4;
    capsule_cp.static_friction = 0.2;
    capsule_cp.dynamic_friction = 0.99;
    w.create_collider(capsule_cp);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    visr::VisrApp app;
    app.title = "rbps — joint showcase";
    app.screen_w = 1440;
    app.screen_h = 900;

    app.world.timestep = 1.0 / 48.0;
    app.world.substeps = 25;

    build_ground(app.world);
    build_fixed_joint_demo(app.world);
    build_revolute_joint_demo(app.world);
    build_prismatic_joint_demo(app.world);
    build_box_tower(app.world);
    build_convex_hull_demo(app.world);

    // ── Extra demo panel ──────────────────────────────────────────────────
    app.extra_guis.push_back([&]()
                             {
        ImGui::Begin("Demo");
        ImGui::SeparatorText("Joint showcase");
        ImGui::TextDisabled("Left:   Fixed    (two spheres welded)");
        ImGui::TextDisabled("Centre: Revolute (pendulum, swings on Z)");
        ImGui::TextDisabled("Right:  Prismatic (slides on Y, limited +/-2.5)");
        ImGui::Separator();

        const visr::FrameSnapshot *snap = app.channel.transport.latest_snapshot();
        if (snap)
        {
            ImGui::Text("Active contacts : %zu", snap->contacts.size());
            size_t live = 0;
            for (auto &c : snap->contacts) if (c.active) ++live;
            ImGui::Text("Live contacts   : %zu", live);
        }

        ImGui::SeparatorText("Chaos");
        if (ImGui::Button("Kick all dynamic bodies"))
        {
            const visr::FrameSnapshot *s = app.channel.transport.latest_snapshot();
            if (s)
            {
                for (const auto &b : s->bodies)
                {
                    if (b.is_static) continue;
                    app.channel.transport.push_command(visr::CmdApplyImpulse{
                        b.id,
                        m3d::vec3{(b.id % 3 == 0) ? 5.0 : -5.0, 3.0, 0.0},
                        b.position
                    });
                }
            }
        }
        ImGui::End(); });

    app.run();
    return 0;
}