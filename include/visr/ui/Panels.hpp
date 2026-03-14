#pragma once
#include <imgui.h>
#include <string>
#include <cmath>
#include "visr/Snapshot.hpp"
#include "visr/Command.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

// ============================================================================
//  visr/ui/Panels.hpp
//
//  Self-contained ImGui panels.  Each draw_*_panel() function:
//    - Takes a const FrameSnapshot& (read-only physics state)
//    - Takes an InProcessTransport& to push commands back
//    - Has no state of its own beyond ImGui's retained-mode widgets
//
//  Call these from your render loop after ImGui::NewFrame() / BeginDockspace.
//
//  Selection state (selected_body_id / selected_collider_id) is kept here
//  as simple uint32_t so panels can cross-reference each other (e.g. the
//  contact panel highlights contacts involving the selected body).
// ============================================================================

namespace visr::ui
{
    // -------------------------------------------------------------------------
    //  Shared selection state — lives in the render thread
    // -------------------------------------------------------------------------
    struct SelectionState
    {
        uint32_t body_id     = UINT32_MAX;   // UINT32_MAX = nothing selected
        uint32_t collider_id = UINT32_MAX;
        uint32_t joint_id    = UINT32_MAX;
    };

    // -------------------------------------------------------------------------
    //  Helper: vec3 label
    // -------------------------------------------------------------------------
    static void show_vec3(const char *label, const m3d::vec3 &v)
    {
        ImGui::Text("%-20s  %.3f  %.3f  %.3f", label, v.x, v.y, v.z);
    }

    // =========================================================================
    //  Simulation control panel
    // =========================================================================
    inline void draw_sim_control_panel(const FrameSnapshot &snap,
                                       InProcessTransport  &transport,
                                       bool                &paused,
                                       bool                &step_once)
    {
        ImGui::Begin("Simulation");

        ImGui::Text("Frame : %llu", (unsigned long long)snap.frame_index);
        ImGui::Text("Time  : %.4f s", snap.sim_time);
        ImGui::Separator();

        if (ImGui::Button(paused ? "Resume" : "Pause"))
            paused = !paused;

        ImGui::SameLine();
        if (paused && ImGui::Button("Step Once"))
            step_once = true;

        ImGui::Separator();

        // Editable timestep / substeps
        static double ts      = snap.timestep;
        static int    substeps = snap.substeps;
        bool changed = false;
        changed |= ImGui::InputDouble("Timestep",  &ts,      0.0, 0.0, "%.6f");
        changed |= ImGui::InputInt   ("Substeps",  &substeps);
        if (changed && ImGui::Button("Apply##ts"))
            transport.push_command(CmdSetTimestep{ts, substeps});

        ImGui::Separator();
        ImGui::Text("BP pairs : %u",   snap.broad_phase_pairs);
        ImGui::Text("Bodies   : %zu",  snap.bodies.size());
        ImGui::Text("Contacts : %zu",  snap.contacts.size());
        ImGui::Text("Joints   : %zu",  snap.joints.size());

        ImGui::End();
    }

    // =========================================================================
    //  Body list + inspector panel
    // =========================================================================
    inline void draw_body_panel(const FrameSnapshot &snap,
                                InProcessTransport  &transport,
                                SelectionState      &sel)
    {
        ImGui::Begin("Bodies");

        ImGui::Text("%zu bodies", snap.bodies.size());
        ImGui::Separator();

        for (const auto &b : snap.bodies)
        {
            char label[64];
            std::snprintf(label, sizeof(label), "[%u] %s",
                          b.id, b.is_static ? "STATIC" : "DYNAMIC");

            bool selected = (sel.body_id == b.id);
            if (ImGui::Selectable(label, selected))
            {
                sel.body_id = b.id;
                transport.push_command(CmdSelectBody{b.id});
            }
        }

        ImGui::Separator();

        // ── Inspector for selected body ──────────────────────────────────
        if (sel.body_id != UINT32_MAX)
        {
            const BodySnap *found = nullptr;
            for (auto &b : snap.bodies)
                if (b.id == sel.body_id) { found = &b; break; }

            if (found)
            {
                ImGui::Text("Body #%u", found->id);
                ImGui::Separator();

                show_vec3("position",      found->position);
                show_vec3("lin_velocity",  found->linear_velocity);
                show_vec3("ang_velocity",  found->angular_velocity);
                show_vec3("force",         found->force);
                show_vec3("torque",        found->torque);
                ImGui::Text("%-20s  %.4f", "mass",         found->mass);
                ImGui::Text("%-20s  %.4f", "inv_mass",     found->inverse_mass);
                ImGui::Separator();

                // ── Apply impulse widget ─────────────────────────────────
                // static m3d::vec3 imp_dir{0, 1, 0};
                // static float     imp_mag = 10.0f;
                // ImGui::InputFloat3("Impulse dir",  dynamic_cast<float*>(&imp_dir.x));
                // ImGui::SliderFloat("Magnitude",    &imp_mag, 0.0f, 1000.0f);
                // if (ImGui::Button("Apply Impulse"))
                //     transport.push_command(CmdApplyImpulse{
                //         found->id,
                //         imp_dir * imp_mag,
                //         found->position  // at COM
                //     });

                ImGui::SameLine();
                if (ImGui::Button("Zero Velocity"))
                    transport.push_command(CmdZeroVelocity{found->id});

                if (ImGui::Button("Teleport to origin"))
                    transport.push_command(CmdTeleportBody{
                        found->id, {}, {}
                    });
            }
        }

        ImGui::End();
    }

    // =========================================================================
    //  Collider inspector panel
    // =========================================================================

    static const char *shape_name(const ShapeSnap &s)
    {
        return std::visit([](auto &&v) -> const char* {
            using T = std::decay_t<decltype(v)>;
            if      constexpr (std::is_same_v<T, SpherSnap>)     return "Sphere";
            else if constexpr (std::is_same_v<T, BoxSnap>)       return "Box";
            else if constexpr (std::is_same_v<T, CapsuleSnap>)   return "Capsule";
            else if constexpr (std::is_same_v<T, PlaneSnap>)     return "Plane";
            else if constexpr (std::is_same_v<T, ConeSnap>)      return "Cone";
            else if constexpr (std::is_same_v<T, EllipsoidSnap>) return "Ellipsoid";
            else if constexpr (std::is_same_v<T, HeightmapSnap>) return "Heightmap";
            else if constexpr (std::is_same_v<T, MeshSnap>)      return "Mesh";
            else return "Unknown";
        }, s);
    }

    inline void draw_collider_panel(const FrameSnapshot &snap,
                                    SelectionState      &sel)
    {
        ImGui::Begin("Colliders");

        for (const auto &c : snap.colliders)
        {
            char label[80];
            std::snprintf(label, sizeof(label), "[%u] %s (body %u)",
                          c.id, shape_name(c.shape), c.body_id);
            bool selected = (sel.collider_id == c.id);
            if (ImGui::Selectable(label, selected))
                sel.collider_id = c.id;
        }

        ImGui::Separator();

        if (sel.collider_id != UINT32_MAX)
        {
            const ColliderSnap *found = nullptr;
            for (auto &c : snap.colliders)
                if (c.id == sel.collider_id) { found = &c; break; }

            if (found)
            {
                ImGui::Text("Collider #%u  —  %s", found->id, shape_name(found->shape));
                ImGui::Text("Body #%u  |  %s",
                            found->body_id, found->is_static ? "STATIC" : "DYNAMIC");
                ImGui::Separator();
                show_vec3("world_pos",       found->world_pos);
                ImGui::Text("%-20s  %.4f", "restitution",    found->restitution);
                ImGui::Text("%-20s  %.4f", "static_friction",found->static_friction);
                ImGui::Text("%-20s  %.4f", "dyn_friction",   found->dynamic_friction);

                // Shape-specific params
                ImGui::Separator();
                std::visit([](auto &&sh)
                {
                    using T = std::decay_t<decltype(sh)>;
                    if      constexpr (std::is_same_v<T, SpherSnap>)
                        ImGui::Text("radius       = %.4f", sh.radius);
                    else if constexpr (std::is_same_v<T, BoxSnap>)
                        ImGui::Text("half_extents = %.3f  %.3f  %.3f",
                                    sh.half_extents.x, sh.half_extents.y, sh.half_extents.z);
                    else if constexpr (std::is_same_v<T, CapsuleSnap>)
                        ImGui::Text("radius=%.4f  half_height=%.4f",
                                    sh.radius, sh.half_height);
                    else if constexpr (std::is_same_v<T, PlaneSnap>)
                        ImGui::Text("normal=(%.3f %.3f %.3f)  d=%.4f",
                                    sh.normal.x, sh.normal.y, sh.normal.z, sh.distance);
                    else if constexpr (std::is_same_v<T, ConeSnap>)
                        ImGui::Text("radius=%.4f  height=%.4f", sh.radius, sh.height);
                    else if constexpr (std::is_same_v<T, EllipsoidSnap>)
                        ImGui::Text("semi_axes = %.3f  %.3f  %.3f",
                                    sh.semi_axes.x, sh.semi_axes.y, sh.semi_axes.z);
                    else if constexpr (std::is_same_v<T, HeightmapSnap>)
                        ImGui::Text("grid %u × %u  cell=%.4f",
                                    sh.cols, sh.rows, sh.cell_size);
                    else if constexpr (std::is_same_v<T, MeshSnap>)
                        ImGui::Text("verts=%u  faces=%u",
                                    sh.vertex_count, sh.face_count);
                }, found->shape);
            }
        }

        ImGui::End();
    }

    // =========================================================================
    //  Contact list panel
    // =========================================================================
    inline void draw_contact_panel(const FrameSnapshot &snap,
                                   const SelectionState &sel)
    {
        ImGui::Begin("Contacts");

        ImGui::Text("%zu contacts", snap.contacts.size());
        ImGui::Separator();

        if (ImGui::BeginTable("contacts", 6,
            ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY |
            ImGuiTableFlags_RowBg   | ImGuiTableFlags_SizingFixedFit,
            ImVec2(0, 300)))
        {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("ColA");
            ImGui::TableSetupColumn("ColB");
            ImGui::TableSetupColumn("Depth");
            ImGui::TableSetupColumn("λ_n");
            ImGui::TableSetupColumn("λ_t");
            ImGui::TableSetupColumn("Active");
            ImGui::TableHeadersRow();

            for (const auto &ct : snap.contacts)
            {
                // Highlight contacts involving the selected body
                bool highlight = (sel.body_id != UINT32_MAX &&
                                  (ct.body_a == sel.body_id ||
                                   ct.body_b == sel.body_id));
                if (highlight)
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                          IM_COL32(80, 40, 0, 100));

                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::Text("%u", ct.collider_a);
                ImGui::TableNextColumn(); ImGui::Text("%u", ct.collider_b);
                ImGui::TableNextColumn(); ImGui::Text("%.4f", ct.penetration_depth);
                ImGui::TableNextColumn(); ImGui::Text("%.4f", ct.normal_lambda);
                ImGui::TableNextColumn(); ImGui::Text("%.4f", ct.tangent_lambda);
                ImGui::TableNextColumn(); ImGui::Text("%s", ct.active ? "Y" : "N");
            }
            ImGui::EndTable();
        }

        ImGui::End();
    }

    // =========================================================================
    //  Joint panel
    // =========================================================================
    inline void draw_joint_panel(const FrameSnapshot &snap,
                                 InProcessTransport  &transport,
                                 SelectionState      &sel)
    {
        ImGui::Begin("Joints");

        for (const auto &j : snap.joints)
        {
            const char *type_str =
                j.type == JointTypeSnap::Prismatic ? "Prismatic" :
                j.type == JointTypeSnap::Revolute  ? "Revolute"  : "Fixed";
            char label[80];
            std::snprintf(label, sizeof(label), "[%u] %s  (b%u↔b%u)",
                          j.id, type_str, j.body_a, j.body_b);
            bool selected = (sel.joint_id == j.id);
            if (ImGui::Selectable(label, selected))
                sel.joint_id = j.id;
        }

        ImGui::Separator();

        if (sel.joint_id != UINT32_MAX)
        {
            const JointSnap *found = nullptr;
            for (auto &j : snap.joints)
                if (j.id == sel.joint_id) { found = &j; break; }

            if (found)
            {
                ImGui::Text("pos=%.4f  [%.3f, %.3f]",
                            found->current_position,
                            found->lower_limit, found->upper_limit);
                ImGui::Text("damping = %.4f", found->damping);

                static float target = 0.0f;
                ImGui::SliderFloat("Target", &target,
                                   found->lower_limit, found->upper_limit);
                if (ImGui::Button("Set Target"))
                    transport.push_command(CmdSetJointTarget{found->id, target});

                static float damping = found->damping;
                ImGui::SliderFloat("Damping", &damping, 0.0f, 100.0f);
                if (ImGui::Button("Set Damping"))
                    transport.push_command(CmdSetJointDamping{found->id, damping});
            }
        }

        ImGui::End();
    }

    // =========================================================================
    //  Live graph panel (tracked quantities)
    // =========================================================================
    template<typename T>
    inline void draw_graph_panel(DebugChannel<T>     &channel,
                                 InProcessTransport  &transport,
                                 const FrameSnapshot &snap)
    {
        ImGui::Begin("Graphs");

        // Quick-add: track linear speed of selected body
        static uint32_t track_id = 0;
        ImGui::InputScalar("Track body ID", ImGuiDataType_U32, &track_id);

        if (ImGui::Button("Track Lin Speed"))
            transport.push_command(CmdTrackQuantity{
                TrackTarget::BodyLinearSpeed, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Track Pos Y"))
            transport.push_command(CmdTrackQuantity{
                TrackTarget::BodyPositionY, track_id, true});

        ImGui::Separator();

        for (const auto &series : channel.tracked_series())
        {
            if (series.samples.empty()) continue;

            // Build float array for ImGui::PlotLines
            std::vector<float> vals;
            vals.reserve(series.samples.size());
            float vmin =  1e30f, vmax = -1e30f;
            for (auto &s : series.samples)
            {
                vals.push_back(s.value);
                vmin = std::min(vmin, s.value);
                vmax = std::max(vmax, s.value);
            }

            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "%.3f", vals.back());

            char title[80];
            std::snprintf(title, sizeof(title), "id=%u  min=%.2f  max=%.2f",
                          series.id, vmin, vmax);

            ImGui::PlotLines(title,
                             vals.data(), (int)vals.size(),
                             0, overlay, vmin, vmax,
                             ImVec2(0, 80));
        }

        ImGui::End();
    }

    // =========================================================================
    //  Master draw — call once per render frame
    // =========================================================================
    template<typename T>
    inline void draw_all(DebugChannel<T>    &channel,
                         InProcessTransport &transport,
                         const FrameSnapshot &snap,
                         SelectionState      &sel)
    {
        bool paused     = channel.is_paused();
        bool step_once  = false;

        draw_sim_control_panel(snap, transport, paused, step_once);

        if (paused != channel.is_paused())
            paused ? channel.pause() : channel.resume();
        if (step_once)
            channel.request_step_once();

        draw_body_panel    (snap, transport, sel);
        draw_collider_panel(snap, sel);
        draw_contact_panel (snap, sel);
        draw_joint_panel   (snap, transport, sel);
        draw_graph_panel   (channel, transport, snap);
    }

} // namespace visr::ui