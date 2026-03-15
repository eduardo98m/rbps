// ============================================================================
//  src/visr_demo/main.cpp
//
//  Showcases all three joint types using spheres (easiest to see + collide).
//  Inertia tensors are computed automatically from shape + mass.
//
//  Scene layout:
//
//    x = -4            x = 0             x = +4
//
//    [A]──fixed──[B]   [C]──revolute──[D]  [E]──prismatic──[F]
//    static+bob         pendulum pivot+bob  sliding rail+bob
//
//  Each group is separated so joints don't interfere with each other.
//  A dynamic box ground catches falling spheres at y = 0.
// ============================================================================

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
static uint32_t make_sphere_body(rbps::World    &w,
                                  m3d::vec3       pos,
                                  m3d::scalar     mass,
                                  m3d::scalar     radius,
                                  rbps::BodyType  type     = rbps::BodyType::DYNAMIC,
                                  m3d::scalar     restitution      = 0.5,
                                  m3d::scalar     static_friction  = 0.4,
                                  m3d::scalar     dynamic_friction = 0.3)
{
    rbc::Sphere sphere{radius};

    rbps::BodyParams bp{};
    bp.type     = type;
    bp.position = pos;
    bp.mass     = mass;
    if (type == rbps::BodyType::DYNAMIC)
    {
        // I_shape is computed at unit density; scale to actual mass / volume.
        const m3d::scalar vol = rbc::compute_volume(sphere);
        bp.inertia_tensor = rbc::compute_inertia_tensor(sphere) * (mass / vol);
    }
    const uint32_t id = w.create_body(bp);

    rbps::ColliderParams cp{};
    cp.body_id          = id;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{1, 0, 0, 0};
    cp.shape            = rbc::Shape(sphere);
    cp.restitution      = restitution;
    cp.static_friction  = static_friction;
    cp.dynamic_friction = dynamic_friction;
    w.create_collider(cp);

    return id;
}

// ── Scene sections ────────────────────────────────────────────────────────────

// Fixed joint: two spheres welded together — the bob inherits the static
// anchor's stillness while the joint prevents ALL relative motion.
// Useful to test: joint error should be near zero every frame.
static void build_fixed_joint_demo(rbps::World &w)
{
    // Static anchor (no collider needed — it's just a reference point)
    rbps::BodyParams ap{};
    ap.type     = rbps::BodyType::STATIC;
    ap.position = m3d::vec3{-4.0, 4.0, 0.0};
    ap.mass     = 0.0;
    const uint32_t anchor = w.create_body(ap);

    // Dynamic bob attached to anchor
    const uint32_t bob = make_sphere_body(w, {-4.0, 2.0, 0.0}, 1.0, 0.3);

    // Fixed joint: r_1 = {0,-2,0} so the bob hangs 2 m below the anchor.
    rbps::FixedJointParams fjp{};
    fjp.body_1 = anchor;
    fjp.body_2 = bob;
    fjp.r_1    = m3d::vec3{0, -2.0,  0};
    fjp.r_2    = m3d::vec3{0,  0.0,  0};
    w.create_fixed_joint(fjp);
}

// Revolute joint: classic pendulum.
// Axis = Z.  Bob starts displaced sideways so it actually swings.
static void build_revolute_joint_demo(rbps::World &w)
{
    // Static pivot
    rbps::BodyParams pp{};
    pp.type     = rbps::BodyType::STATIC;
    pp.position = m3d::vec3{0.0, 4.0, 0.0};
    pp.mass     = 0.0;
    const uint32_t pivot = w.create_body(pp);

    // Bob: displaced 1.8 m on X from the pivot so it has swing amplitude.
    const uint32_t bob = make_sphere_body(w, {1.8, 4.0, 0.0}, 1.0, 0.3);

    rbps::RevoluteJointParams rjp{};
    rjp.body_1       = pivot;
    rjp.body_2       = bob;
    rjp.aligned_axis = m3d::vec3{0, 0, 1};   // rotation axis
    rjp.limit_axis   = m3d::vec3{1, 0, 0};   // reference for angle measurement
    rjp.r_1          = m3d::vec3{0,    0, 0};
    rjp.r_2          = m3d::vec3{-1.8, 0, 0}; // bob offset from its own COM
    rjp.limited      = false;
    rjp.damping      = 0.05;
    w.create_revolute_joint(rjp);
}

// Prismatic joint: one static rail body + one bob that slides along Y.
// Give the bob an initial velocity so it actually moves.
static void build_prismatic_joint_demo(rbps::World &w)
{
    // Static rail anchor
    rbps::BodyParams rp{};
    rp.type     = rbps::BodyType::STATIC;
    rp.position = m3d::vec3{4.0, 4.0, 0.0};
    rp.mass     = 0.0;
    const uint32_t rail = w.create_body(rp);

    // Sliding bob — starts at the anchor position, will slide along Y.
    const uint32_t bob = make_sphere_body(w, {4.0, 4.0, 0.0}, 1.0, 0.3);

    // Give it an initial downward velocity so it slides visibly from frame 1.
    const uint32_t slot = w.bodies.index_of(bob);
    w.bodies.linear_velocity[slot] = m3d::vec3{0, -2.0, 0};

    rbps::PrismaticJointParams pjp{};
    pjp.body_1       = rail;
    pjp.body_2       = bob;
    pjp.moving_axis  = m3d::vec3{0, 1, 0};  // slide along Y
    pjp.r_1          = m3d::vec3{0, 0, 0};
    pjp.r_2          = m3d::vec3{0, 0, 0};
    pjp.limited      = true;
    pjp.lower_limit  = -2.5;
    pjp.upper_limit  =  2.5;
    pjp.damping      = 0.1;
    w.create_prismatic_joint(pjp);
}

// Static ground plane so free-falling spheres have something to land on.
static void build_ground(rbps::World &w)
{
    rbps::BodyParams gp{};
    gp.type     = rbps::BodyType::STATIC;
    gp.position = m3d::vec3{0.0, -0.3, 0.0};
    gp.mass     = 0.0;
    const uint32_t ground = w.create_body(gp);

    rbps::ColliderParams cp{};
    cp.body_id          = ground;
    cp.local_pos        = m3d::vec3{0, 0, 0};
    cp.local_rot        = m3d::quat{1, 0, 0, 0};
    cp.shape            = rbc::Shape(rbc::Box{{12.0, 0.3, 6.0}});
    cp.restitution      = 0.4;
    cp.static_friction  = 0.6;
    cp.dynamic_friction = 0.4;
    w.create_collider(cp);

    rbc::Sphere sphere{0.3};

    rbps::BodyParams bp{};
    bp.type     = rbps::BodyType::DYNAMIC;
    bp.position = m3d::vec3{0.0, 1.0, 0.0};
    bp.mass     = 1.0;

    // I_shape is computed at unit density; scale to actual mass / volume.
    const m3d::scalar vol = rbc::compute_volume(sphere);
    bp.inertia_tensor = rbc::compute_inertia_tensor(sphere) * (bp.mass / vol);
    
    const uint32_t id = w.create_body(bp);

    rbps::ColliderParams ball_cp{};
    ball_cp.body_id          = id;
    ball_cp.local_pos        = m3d::vec3{0, 0, 0};
    ball_cp.local_rot        = m3d::quat{1, 0, 0, 0};
    ball_cp.shape            = rbc::Shape(sphere);
    ball_cp.restitution      = 0.3;
    ball_cp.static_friction  = 0.6;
    ball_cp.dynamic_friction = 0.4;
    w.create_collider(ball_cp);

}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    visr::VisrApp app;
    app.title    = "rbps — joint showcase";
    app.screen_w = 1440;
    app.screen_h = 900;

    app.world.timestep = 1.0 / 60.0;
    app.world.substeps = 20;

    build_ground         (app.world);
    build_fixed_joint_demo   (app.world);
    build_revolute_joint_demo(app.world);
    build_prismatic_joint_demo(app.world);


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
        ImGui::End();
    });

    app.run();
    return 0;
}