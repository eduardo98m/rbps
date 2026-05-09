#include "visr/VisrApp.hpp"
#include "rbps/API/World.hpp"
#include "rbps/API/BodyAPI.hpp"
#include "rbps/API/ColliderAPI.hpp"
#include "rbps/API/JointAPI.hpp"
#include "rbc/shapes/ShapeTypes.hpp"
#include "imgui.h"
#include <cmath>

// ── Convex Hull Data Providers ────────────────────────────────────────────────

inline rbc::ConvexHullData *get_tetrahedron_data()
    {
        static const m3d::vec3 verts[4] = {
            m3d::vec3( 0.6,  0.6,  0.6),
            m3d::vec3(-0.6, -0.6,  0.6),
            m3d::vec3(-0.6,  0.6, -0.6),
            m3d::vec3( 0.6, -0.6, -0.6),
        };
        // CCW-from-outside winding so cross(B-A, C-A) yields the outward normal.
        // (Original 0,1,2 / 0,3,1 / 0,2,3 / 1,3,2 was inward — see plan.)
        static const uint32_t faces[4 * 3] = {
            0, 2, 1,
            0, 1, 3,
            0, 3, 2,
            1, 2, 3,
        };
        static rbc::ConvexHullData *data =
            rbc::convex_hull_data_create(verts, 4, faces, 4);
        return data;
    }

static rbc::ConvexHullData *get_octahedron_data()
{
    static const m3d::vec3 verts[6] = {
        m3d::vec3(1.0, 0.0, 0.0), m3d::vec3(-1.0, 0.0, 0.0),
        m3d::vec3(0.0, 1.0, 0.0), m3d::vec3(0.0, -1.0, 0.0),
        m3d::vec3(0.0, 0.0, 1.0), m3d::vec3(0.0, 0.0, -1.0)};
    static const uint32_t faces[8 * 3] = {
        0, 2, 4, 0, 4, 3, 0, 3, 5, 0, 5, 2,
        1, 4, 2, 1, 3, 4, 1, 5, 3, 1, 2, 5};
    static rbc::ConvexHullData *data = rbc::convex_hull_data_create(verts, 6, faces, 8);
    return data;
}

static rbc::ConvexHullData *get_prism_data()
{
    static const m3d::vec3 verts[6] = {
        m3d::vec3(0.0, 1.0, 1.0), m3d::vec3(-0.866, -0.5, 1.0), m3d::vec3(0.866, -0.5, 1.0),
        m3d::vec3(0.0, 1.0, -1.0), m3d::vec3(-0.866, -0.5, -1.0), m3d::vec3(0.866, -0.5, -1.0)};
    static const uint32_t faces[5 * 6] = {
        0, 1, 2, 3, 5, 4,
        0, 3, 4, 0, 4, 1,
        1, 4, 5, 1, 5, 2,
        2, 5, 3, 2, 3, 0};
    static rbc::ConvexHullData *data = rbc::convex_hull_data_create(verts, 6, faces, 8);
    return data;
}

inline rbc::ConvexHullData *get_hex_prism_data()
    {
        static m3d::vec3 verts[12];
        static uint32_t  faces[20 * 3];
        static bool initialized = false;

        if (!initialized)
        {
            for (int i = 0; i < 6; ++i)
            {
                const float angle = i * (2 * m3d::PI / 6.0f);
                const float x = std::cos(angle) * 0.8f;
                const float z = std::sin(angle) * 0.8f;
                verts[i]     = m3d::vec3(x,  0.5f, z);
                verts[i + 6] = m3d::vec3(x, -0.5f, z);
            }

            int f = 0;
            // Vertices on top/bottom hexagons go CCW around +Y when viewed
            // from above (+Y → -Y), because (cos θ, sin θ) with θ increasing
            // and looking down -Y traces the X-Z plane CW... but cross(B-A,
            // C-A) of three CCW-from-+Y points lands in -Y. So for the
            // OUTWARD top-cap normal we need (0, i+1, i), and for the
            // outward bottom-cap normal we need (6, i+6, i+7). Sides also
            // need their winding flipped from the naive choice.

            // Top cap (tri-fan around vertex 0) — outward = +Y
            for (int i = 1; i < 5; ++i)
            {
                faces[f++] = 0; faces[f++] = i + 1; faces[f++] = i;
            }
            // Bottom cap (tri-fan around vertex 6) — outward = -Y
            for (int i = 1; i < 5; ++i)
            {
                faces[f++] = 6; faces[f++] = i + 6; faces[f++] = i + 7;
            }
            // Sides: 6 quads = 12 triangles, outward radial
            for (int i = 0; i < 6; ++i)
            {
                const int next = (i + 1) % 6;
                faces[f++] = i;    faces[f++] = next;       faces[f++] = i + 6;
                faces[f++] = next; faces[f++] = next + 6;   faces[f++] = i + 6;
            }
            initialized = true;
        }

        static rbc::ConvexHullData *data =
            rbc::convex_hull_data_create(verts, 12, faces, 20);
        return data;
    }
// ── Corrected Helpers ─────────────────────────────────────────────────────────

// This function now takes volume and unit_inertia explicitly to avoid the Shape overloading error
static uint32_t spawn_dynamic_body(rbps::World &w,
                                   const rbc::Shape &shape,
                                   m3d::scalar volume,
                                   const m3d::smat3 &unit_inertia,
                                   const m3d::vec3 &pos,
                                   const m3d::quat &rot,
                                   m3d::scalar mass)
{
    rbps::BodyParams bp{};
    bp.type = rbps::BodyType::DYNAMIC;
    bp.position = pos;
    bp.orientation = rot;
    bp.mass = mass;

    // Correctly scale the inertia tensor based on mass/volume
    bp.inertia_tensor = unit_inertia * (mass / volume);

    const uint32_t id = w.create_body(bp);

    rbps::ColliderParams cp{};
    cp.body_id = id;
    cp.local_pos = m3d::vec3{0, 0, 0};
    cp.local_rot = m3d::quat{1, 0, 0, 0};
    cp.shape = shape;
    cp.restitution = 0.2;
    cp.static_friction = 0.5;
    cp.dynamic_friction = 0.4;
    w.create_collider(cp);

    return id;
}

// Fixed-type helper for internal use (e.g. joints)
static uint32_t make_sphere_body(rbps::World &w, m3d::vec3 pos, m3d::scalar mass, m3d::scalar radius)
{
    rbc::Sphere sphere{radius};
    return spawn_dynamic_body(w, rbc::Shape(sphere), rbc::compute_volume(sphere), rbc::compute_inertia_tensor(sphere), pos, m3d::quat{1, 0, 0, 0}, mass);
}

// ── Demos ─────────────────────────────────────────────────────────────────────

static void build_fixed_joint_demo(rbps::World &w)
{
    rbps::BodyParams ap{};
    ap.type = rbps::BodyType::STATIC;
    ap.position = m3d::vec3{-4.0, 4.0, 0.0};
    const uint32_t anchor = w.create_body(ap);
    const uint32_t bob = make_sphere_body(w, {-4.0, 2.0, 0.0}, 1.0, 0.3);

    rbps::FixedJointParams fjp{};
    fjp.body_1 = anchor;
    fjp.body_2 = bob;
    fjp.r_1 = {0, -2.0, 0};
    fjp.r_2 = {0, 0, 0};
    w.create_fixed_joint(fjp);
}

static void build_revolute_joint_demo(rbps::World &w)
{
    rbps::BodyParams pp{};
    pp.type = rbps::BodyType::STATIC;
    pp.position = m3d::vec3{0.0, 4.0, 0.0};
    const uint32_t pivot = w.create_body(pp);
    const uint32_t bob = make_sphere_body(w, {1.8, 4.0, 0.0}, 20.0, 0.3);

    rbps::RevoluteJointParams rjp{};
    rjp.body_1 = pivot;
    rjp.body_2 = bob;
    rjp.aligned_axis = {0, 0, 1};
    rjp.limit_axis = {1, 0, 0};
    rjp.r_1 = {0, 0, 0};
    rjp.r_2 = {-1.8, 0, 0};
    w.create_revolute_joint(rjp);
}

static void build_prismatic_joint_demo(rbps::World &w)
{
    rbps::BodyParams rp{};
    rp.type = rbps::BodyType::STATIC;
    rp.position = m3d::vec3{4.0, 4.0, 0.0};
    const uint32_t rail = w.create_body(rp);
    const uint32_t bob = make_sphere_body(w, {4.0, 4.0, 0.0}, 1.0, 0.3);

    const uint32_t slot = w.bodies.index_of(bob);
    w.bodies.linear_velocity[slot] = m3d::vec3{0, -2.0, 0};

    rbps::PrismaticJointParams pjp{};
    pjp.body_1 = rail;
    pjp.body_2 = bob;
    pjp.moving_axis = {0, 1, 0};
    pjp.limited = true;
    pjp.lower_limit = -2.5;
    pjp.upper_limit = 2.5;
    w.create_prismatic_joint(pjp);
}

static void build_ground(rbps::World &w)
{
    rbps::BodyParams gp{};
    gp.type = rbps::BodyType::STATIC;
    gp.position = m3d::vec3{0.0, -0.5, 0.0};
    const uint32_t ground = w.create_body(gp);

    rbps::ColliderParams cp{};
    cp.body_id = ground;
    cp.shape = rbc::Shape(rbc::Plane({0.0, 1.0, 0.0}, 0.0));
    w.create_collider(cp);

    // Initial scatter
    rbc::Box box{{0.5, 0.5, 0.5}};
    spawn_dynamic_body(w, rbc::Shape(box), rbc::compute_volume(box), rbc::compute_inertia_tensor(box), {2.0, 1.0, 4.0}, m3d::quat{1, 0, 0, 0}, 5.0);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main()
{
    visr::VisrApp app;
    app.title = "RBPS - Thread-Safe Stacking Demo";

    // 1. Build Ground
    rbps::BodyParams gp{};
    gp.type = rbps::BodyType::STATIC;
    uint32_t ground = app.world.create_body(gp);
    rbps::ColliderParams cp{};
    cp.body_id = ground;
    cp.shape = rbc::Shape(rbc::Plane({0, 1, 0}, 0));
    app.world.create_collider(cp);

    // 2. Interactive UI
    app.extra_guis.push_back([&]()
                             {
        ImGui::Begin("Safe Spawner");
        
        static int shape_idx = 0;
        const char* names[] = { "Sphere", "Box", "Cylinder", "Hex Prism (Stackable)", "Tetrahedron" };
        ImGui::Combo("Shape", &shape_idx, names, 5);

        static float pos[3] = { 0, 5, 0 };
        ImGui::DragFloat3("Pos", pos, 0.1f);

        if (ImGui::Button("Spawn (Command Queue)", ImVec2(-1, 40))) {
            visr::CmdSpawnBody cmd;
            cmd.position = { pos[0], pos[1], pos[2] };
            cmd.orientation = m3d::quat{1,0,0,0};
            cmd.mass = 5.0f;

            if (shape_idx == 0) {
                rbc::Sphere s{0.5};
                cmd.shape = rbc::Shape(s); cmd.volume = rbc::compute_volume(s); cmd.unit_inertia = rbc::compute_inertia_tensor(s);
            } else if (shape_idx == 1) {
                rbc::Box s{{0.5, 0.5, 0.5}};
                cmd.shape = rbc::Shape(s); cmd.volume = rbc::compute_volume(s); cmd.unit_inertia = rbc::compute_inertia_tensor(s);
            } 
            else if (shape_idx == 2) {
                rbc::Cylinder s = rbc::Cylinder(0.5, 0.6);
                cmd.shape = rbc::Shape(s); cmd.volume = rbc::compute_volume(s); cmd.unit_inertia = rbc::compute_inertia_tensor(s);
            }
            else if (shape_idx == 3) {
                rbc::ConvexHull s(get_hex_prism_data());
                cmd.shape = rbc::Shape(s); cmd.volume = rbc::compute_volume(s); cmd.unit_inertia = rbc::compute_inertia_tensor(s);
            } 
            else {
                rbc::ConvexHull s(get_tetrahedron_data());
                cmd.shape = rbc::Shape(s); cmd.volume = rbc::compute_volume(s); cmd.unit_inertia = rbc::compute_inertia_tensor(s);
            }

            // PUSH TO COMMAND QUEUE - Thread Safe!
            app.channel.transport.push_command(cmd);
        }

        if (ImGui::Button("Clear All Dynamic")) {
            // Future exercise: add a CmdDeleteBody
        }
        
        ImGui::End(); });

    app.run();
    return 0;
}