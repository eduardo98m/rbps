// ============================================================================
//  src/visr_demo/main.cpp
//
//  Minimal scene that exercises every major system:
//    • Static ground box
//    • Stack of dynamic boxes (gravity, contacts, friction)
//    • A pendulum via revolute joint
//    • An impulse button in the ImGui panel
//    • Live graphs for body velocity + contact lambda
//
//  ADAPT comments mark every API call whose field names you may need to
//  adjust to match your actual BodyParams / ColliderParams / JointParams
//  structs in BodyAPI.hpp, ColliderAPI.hpp, JointAPI.hpp.
//
//  Build:
//    cmake -DRBPS_BUILD_VISR=ON -DCMAKE_BUILD_TYPE=Release ..
//    make visr_demo
// ============================================================================

#include "visr/VisrApp.hpp"      // brings in World, DebugChannel, Raylib, ImGui

// ── Scene constants ──────────────────────────────────────────────────────────

static constexpr float GRAVITY       = -9.81f;
static constexpr float GROUND_Y      = -2.0f;
static constexpr float BOX_HALF      = 0.5f;
static constexpr int   STACK_COUNT   = 4;

// ── Scene setup ──────────────────────────────────────────────────────────────

static void build_scene(rbps::World &world)
{
    // ── Ground (static flat box) ──────────────────────────────────────────

    // ADAPT: replace field names to match your BodyParams struct
    rbps::BodyParams ground_body{};
    ground_body.type     = rbps::BodyType::STATIC;
    ground_body.position = m3d::vec3{0.0f, GROUND_Y, 0.0f};
    ground_body.mass     = 0.0f;   // ignored for STATIC but set anyway
    const uint32_t ground_id = world.create_body(ground_body);

    // ADAPT: replace field names to match your ColliderParams struct
    rbps::ColliderParams ground_col{};
    ground_col.shape            = rbc::Box({5.0f, 0.3f, 5.0f});
    ground_col.body_id          = ground_id;
    ground_col.local_pos        = m3d::vec3{0, 0, 0};
    ground_col.local_rot        = m3d::quat{0, 0, 0, 1};
    ground_col.restitution      = 0.3f;
    ground_col.static_friction  = 0.6f;
    ground_col.dynamic_friction = 0.4f;
    // ADAPT: set your shape — this assumes rbc::Shape has a make_box helper
    // or a tag+union you fill manually:
    //   ground_col.shape = rbc::Shape::make_box({5.0f, 0.3f, 5.0f});
    world.create_collider(ground_col);

    // ── Stack of dynamic boxes ────────────────────────────────────────────

    uint32_t stack_ids[STACK_COUNT];
    for (int i = 0; i < STACK_COUNT; ++i)
    {
        const float y = GROUND_Y + 0.3f + BOX_HALF + i * (BOX_HALF * 2.0f + 0.02f);

        rbps::BodyParams bp{};
        bp.type     = rbps::BodyType::DYNAMIC;
        bp.position = m3d::vec3{0.0f, y, 0.0f};
        bp.mass     = 1.0f;
        // ADAPT: inertia tensor for a uniform box:
        //   I = mass/12 * (h²+d², w²+d², w²+h²)  with w=h=d=2*BOX_HALF
        const float side = BOX_HALF * 2.0f;
        const float I    = (1.0f / 6.0f) * bp.mass * side * side;
        bp.inertia_tensor = m3d::smat3(I, I, I, 0, 0, 0);
        stack_ids[i] = world.create_body(bp);

        rbps::ColliderParams cp{};
        cp.shape            = rbc::Box({BOX_HALF, BOX_HALF, BOX_HALF});
        cp.body_id          = stack_ids[i];
        cp.local_pos        = m3d::vec3{0, 0, 0};
        cp.local_rot        = m3d::quat{0, 0, 0, 1};
        cp.restitution      = 0.2f;
        cp.static_friction  = 0.5f;
        cp.dynamic_friction = 0.3f;
        world.create_collider(cp);
    }

    // ── Pendulum (revolute joint) ─────────────────────────────────────────
    //  pivot_body is static, bob hangs from it via a revolute joint.

    rbps::BodyParams pivot_bp{};
    pivot_bp.type     = rbps::BodyType::STATIC;
    pivot_bp.position = m3d::vec3{3.0f, 2.0f, 0.0f};
    pivot_bp.mass     = 0.0f;
    const uint32_t pivot_id = world.create_body(pivot_bp);

    rbps::BodyParams bob_bp{};
    bob_bp.type     = rbps::BodyType::DYNAMIC;
    bob_bp.position = m3d::vec3{3.0f + 1.5f, 2.0f, 0.0f};  // offset 1.5m
    bob_bp.mass     = 0.5f;
    const float Ib  = (2.0f / 5.0f) * bob_bp.mass * 0.2f * 0.2f;
    bob_bp.inertia_tensor = m3d::smat3(Ib, Ib, Ib, 0, 0, 0);
    const uint32_t bob_id = world.create_body(bob_bp);

    // Sphere collider on the bob
    rbps::ColliderParams bob_col{};
    bob_col.body_id          = bob_id;
    bob_col.local_pos        = m3d::vec3{0, 0, 0};
    bob_col.local_rot        = m3d::quat{0, 0, 0, 1};
    bob_col.restitution      = 0.5f;
    bob_col.static_friction  = 0.3f;
    bob_col.dynamic_friction = 0.2f;
    bob_col.shape = rbc::Sphere(0.2);
    world.create_collider(bob_col);

    // ADAPT: replace field names to match your RevoluteJointParams struct
    rbps::RevoluteJointParams rjp{};
    rjp.body_1    = pivot_id;
    rjp.body_2    = bob_id;
    // r_1: attachment on pivot in pivot's local space (at its COM)
    rjp.r_1       = m3d::vec3{0, 0, 0};
    // r_2: attachment on bob — offset so the bob swings 1.5m from pivot
    rjp.r_2       = m3d::vec3{-1.5f, 0, 0};
    rjp.aligned_axis = m3d::vec3{0, 0, 1};   // rotate about Z
    rjp.limited   = false;
    rjp.damping   = 0.05f;
    world.create_revolute_joint(rjp);
}

// ── Extra ImGui overlay ───────────────────────────────────────────────────────
//
//  Extends the standard panels with demo-specific controls.
//  Call this inside the rlImGuiBegin/End block, after draw_all().

static void draw_demo_overlay(visr::InProcessTransport &transport,
                               const visr::FrameSnapshot &snap)
{
    ImGui::Begin("Demo Controls");

    ImGui::SeparatorText("Chaos button");
    if (ImGui::Button("Kick all dynamic bodies"))
    {
        for (const auto &b : snap.bodies)
        {
            if (b.is_static) continue;
            visr::CmdApplyImpulse kick{};
            kick.body_id    = b.id;
            kick.world_point= b.position;
            // Random-ish lateral kick
            kick.impulse    = m3d::vec3{
                (b.id % 3 == 0) ?  5.0f : -5.0f,
                3.0f,
                (b.id % 2 == 0) ?  2.0f : -2.0f
            };
            transport.push_command(kick);
        }
    }

    ImGui::SeparatorText("Gravity");
    static float grav = -9.81f;
    if (ImGui::SliderFloat("Y gravity", &grav, -20.0f, 0.0f))
        transport.push_command(visr::CmdSetGravity{m3d::vec3{0, grav, 0}});

    ImGui::SeparatorText("Quick-track");
    ImGui::TextDisabled("Click a body in the Bodies panel,");
    ImGui::TextDisabled("then use the Graphs panel to track it.");

    ImGui::SeparatorText("Stats");
    ImGui::Text("Active contacts : %zu", snap.contacts.size());
    size_t active = 0;
    for (auto &c : snap.contacts) if (c.active) ++active;
    ImGui::Text("  of which live : %zu", active);

    ImGui::End();
}

// ── main ─────────────────────────────────────────────────────────────────────

int main()
{
    visr::VisrApp app("rbps visualizer — demo", 1440, 900);

    // Configure the world
    app.world.timestep = 1.0 / 60.0;
    app.world.substeps = 20;

    // Populate the scene
    build_scene(app.world);

    // Inject the extra overlay into the render loop by overriding run().
    // We do this inline rather than subclassing, by copying VisrApp::run()
    // and adding our extra panel call.  (Or just add it to VisrApp::run()
    // directly if you own that file.)
    {
        std::atomic<bool> running{true};
        auto &world   = app.world;
        auto &channel = app.channel;

        // Physics thread
        std::thread phys([&]()
        {
            while (running.load(std::memory_order_relaxed))
            {
                channel.poll(world);
                if (channel.should_step())
                    world.step();
                channel.push(world);
            }
        });

        // Render thread (must be the main thread on macOS/Windows)
        InitWindow(1440, 900, "rbps visr — demo");
        SetTargetFPS(60);
        rlImGuiSetup(true);

        Camera3D cam{};
        cam.position   = {6.0f, 6.0f, 12.0f};
        cam.target     = {0.0f, 0.0f,  0.0f};
        cam.up         = {0.0f, 1.0f,  0.0f};
        cam.fovy       = 45.0f;
        cam.projection = CAMERA_PERSPECTIVE;

        visr::ui::SelectionState sel{};
        visr::draw::DrawFlags      flags{};
        flags.velocities = true;   // show velocity arrows

        while (!WindowShouldClose())
        {
            // Orbit camera with mouse
            UpdateCamera(&cam, CAMERA_ORBITAL);

            const visr::FrameSnapshot *snap =
                channel.transport.latest_snapshot();

            BeginDrawing();
            ClearBackground({30, 30, 35, 255});

            BeginMode3D(cam);
            DrawGrid(20, 1.0f);
            if (snap)
                visr::draw::draw_scene(*snap, flags, sel.body_id);
            EndMode3D();

            rlImGuiBegin();
            if (snap)
            {
                visr::ui::draw_all(channel, channel.transport, *snap, sel);
                draw_demo_overlay(channel.transport, *snap);
            }
            rlImGuiEnd();

            DrawFPS(10, 10);
            EndDrawing();
        }

        running.store(false, std::memory_order_relaxed);
        rlImGuiShutdown();
        CloseWindow();
        phys.join();
    }

    return 0;
}