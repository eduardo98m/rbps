#pragma once
#include <imgui.h>
#include <string>
#include <cmath>
#include <algorithm>
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
//  Changes from v1:
//    - SelectionState: added contact_idx (was missing → contacts unselectable)
//    - draw_contact_panel: rows are now clickable, colour-coded by depth + active
//      state, body IDs shown alongside collider IDs.
//    - draw_contact_detail_panel: new — shows every field of the selected contact
//      with normal-direction bar chart and unit-length validation.
//    - draw_body_panel: uncommented + fixed impulse widget; contacts-involving-body
//      list; derived speed readouts.
//    - draw_joint_panel: fixed static-float-initialised-from-pointer bug; added
//      live limit bar coloured green/red.
//    - draw_sim_control_panel: now owns pause/resume directly on the channel
//      (no stale local bool comparison).
//    - draw_all: updated signature to match new panel signatures.
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
        uint32_t contact_idx = UINT32_MAX;   // index into snap.contacts (NEW)
    };

    // -------------------------------------------------------------------------
    //  Colour helpers
    // -------------------------------------------------------------------------
    // Depth: green (near 0) → yellow → red (at max_depth and beyond)
    static inline ImVec4 depth_color(float depth, float max_depth = 0.15f)
    {
        const float t = std::min(depth / max_depth, 1.0f);
        if (t < 0.5f) return ImVec4(t * 2.0f, 1.0f, 0.0f, 1.0f);
        return             ImVec4(1.0f, 1.0f - (t - 0.5f) * 2.0f, 0.0f, 1.0f);
    }

    // Lambda magnitude: white (zero) → blue (compression) / pink (tension)
    static inline ImVec4 lambda_color(float lambda)
    {
        const float t = std::min(std::fabs(lambda) / 5.0f, 1.0f);
        return lambda < 0.0f
            ? ImVec4(1.0f - t * 0.5f, 1.0f - t * 0.5f, 1.0f, 1.0f)
            : ImVec4(1.0f, 1.0f - t * 0.5f, 1.0f - t * 0.5f, 1.0f);
    }

    // -------------------------------------------------------------------------
    //  Helper: inline vec3 row
    // -------------------------------------------------------------------------
    static void show_vec3(const char *label, const m3d::vec3 &v)
    {
        ImGui::Text("%-22s  %8.4f  %8.4f  %8.4f", label, v.x, v.y, v.z);
    }

    // =========================================================================
    //  Simulation control panel
    //  v2: pause/resume directly on channel — no stale local bool.
    // =========================================================================
    template<typename T>
    inline void draw_sim_control_panel(const FrameSnapshot &snap,
                                       InProcessTransport  &transport,
                                       DebugChannel<T>     &channel)
    {
        ImGui::Begin("Simulation");

        ImGui::Text("Frame : %llu", (unsigned long long)snap.frame_index);
        ImGui::Text("Time  : %.4f s", snap.sim_time);
        ImGui::Separator();

        const bool paused = channel.is_paused();

        // Toggle directly on the channel — avoids the v1 bug where a local
        // bool was compared to itself after flipping.
        if (ImGui::Button(paused ? "Resume" : "Pause "))
            paused ? channel.resume() : channel.pause();

        ImGui::SameLine();
        if (paused && ImGui::Button("Step Once"))
            channel.request_step_once();

        if (paused)
        {
            ImGui::SameLine(0, 12);
            ImGui::TextColored({1.0f, 0.6f, 0.0f, 1.0f}, "PAUSED");
        }

        ImGui::Separator();

        static double ts       = snap.timestep;
        static int    substeps = snap.substeps;
        bool changed = false;
        changed |= ImGui::InputDouble("Timestep",  &ts,      0.001, 0.0, "%.6f");
        changed |= ImGui::InputInt   ("Substeps",  &substeps);
        if (substeps < 1) substeps = 1;
        if (changed && ImGui::Button("Apply##ts"))
            transport.push_command(CmdSetTimestep{ts, substeps});

        ImGui::Separator();

        const size_t active_n = std::count_if(snap.contacts.begin(), snap.contacts.end(),
                                              [](const ContactSnap &c){ return c.active; });
        ImGui::Text("BP pairs   : %u",    snap.broad_phase_pairs);
        ImGui::Text("Bodies     : %zu",   snap.bodies.size());
        ImGui::Text("Contacts   : %zu  (%zu active)", snap.contacts.size(), active_n);
        ImGui::Text("Joints     : %zu",   snap.joints.size());
        ImGui::Text("Constraints: %zu",   snap.constraints.size());

        // Quick worst-case contact summary
        float max_depth = 0.0f;
        for (const auto &c : snap.contacts)
            max_depth = std::max(max_depth, (float)c.penetration_depth);
        if (max_depth > 0.0f)
            ImGui::TextColored(depth_color(max_depth),
                               "Max depth: %.5f m", max_depth);

        ImGui::End();
    }

    // =========================================================================
    //  Body list + inspector panel
    //  v2: impulse widget uncommented + fixed; contacts list; speed readouts.
    // =========================================================================
    inline void draw_body_panel(const FrameSnapshot &snap,
                                InProcessTransport  &transport,
                                SelectionState      &sel)
    {
        ImGui::Begin("Bodies");

        ImGui::Text("%zu bodies", snap.bodies.size());
        ImGui::Separator();

        if (ImGui::BeginChild("body_list", ImVec2(0, 130), true))
        {
            for (const auto &b : snap.bodies)
            {
                char label[64];
                std::snprintf(label, sizeof(label), "[%u] %s",
                              b.id, b.is_static ? "STATIC" : "DYNAMIC");
                const bool selected = (sel.body_id == b.id);
                if (ImGui::Selectable(label, selected))
                {
                    sel.body_id = selected ? UINT32_MAX : b.id;
                    transport.push_command(selected
                        ? Command{CmdClearSelection{}}
                        : Command{CmdSelectBody{b.id}});
                }
            }
        }
        ImGui::EndChild();

        ImGui::Separator();

        if (sel.body_id == UINT32_MAX)
        {
            ImGui::TextDisabled("(click a body to inspect)");
            ImGui::End();
            return;
        }

        const BodySnap *found = nullptr;
        for (const auto &b : snap.bodies)
            if (b.id == sel.body_id) { found = &b; break; }
        if (!found) { ImGui::End(); return; }

        const BodySnap &b = *found;

        ImGui::TextColored(b.is_static ? ImVec4(0.6f,0.6f,0.6f,1) : ImVec4(0.4f,0.8f,1,1),
                           b.is_static ? "STATIC" : "DYNAMIC");
        ImGui::SameLine(); ImGui::Text("  Body #%u", b.id);
        ImGui::Separator();

        show_vec3("position",     b.position);
        show_vec3("lin_velocity", b.linear_velocity);
        show_vec3("ang_velocity", b.angular_velocity);
        show_vec3("force",        b.force);
        show_vec3("torque",       b.torque);

        // Derived speeds — much easier to read at a glance than raw vectors
        const float lspeed = (float)std::sqrt(
            b.linear_velocity.x  * b.linear_velocity.x  +
            b.linear_velocity.y  * b.linear_velocity.y  +
            b.linear_velocity.z  * b.linear_velocity.z);
        const float aspeed = (float)std::sqrt(
            b.angular_velocity.x * b.angular_velocity.x +
            b.angular_velocity.y * b.angular_velocity.y +
            b.angular_velocity.z * b.angular_velocity.z);
        ImGui::Text("%-22s  %8.4f  m/s",   "|lin_velocity|", lspeed);
        ImGui::Text("%-22s  %8.4f  rad/s", "|ang_velocity|", aspeed);
        ImGui::Text("%-22s  %8.4f", "mass",     b.mass);
        ImGui::Text("%-22s  %8.4f", "inv_mass", b.inverse_mass);

        // ── Contacts involving this body ──────────────────────────────────
        ImGui::Separator();
        ImGui::TextDisabled("Contacts involving this body:");
        int ncont = 0;
        for (uint32_t i = 0; i < (uint32_t)snap.contacts.size(); ++i)
        {
            const ContactSnap &c = snap.contacts[i];
            if (c.body_a != b.id && c.body_b != b.id) continue;
            ++ncont;
            const bool is_sel = (sel.contact_idx == i);
            char clabel[80];
            std::snprintf(clabel, sizeof(clabel),
                          "  ct#%u  depth=%.4f  λn=%.3f  %s",
                          i, c.penetration_depth, c.normal_lambda,
                          c.active ? "" : "[inactive]");
            if (ImGui::Selectable(clabel, is_sel))
                sel.contact_idx = is_sel ? UINT32_MAX : i;
        }
        if (ncont == 0) ImGui::TextDisabled("  (none)");

        ImGui::Separator();

        // ── Actions ───────────────────────────────────────────────────────
        if (!b.is_static)
        {
            if (ImGui::Button("Zero Velocity"))
                transport.push_command(CmdZeroVelocity{b.id});
            ImGui::SameLine();
            if (ImGui::Button("Teleport to origin"))
                transport.push_command(CmdTeleportBody{b.id, {}, {}});

            // Impulse widget — was fully commented out in v1 due to a bad
            // float* cast. Fixed: InputFloat3 takes a plain float[3].
            ImGui::Separator();
            ImGui::TextDisabled("Apply impulse:");
            static float imp[3]  = {0.0f, 5.0f, 0.0f};
            static float imp_mag = 1.0f;
            ImGui::InputFloat3("Direction##imp", imp);
            ImGui::SliderFloat("Magnitude##imp", &imp_mag, 0.0f, 200.0f);
            if (ImGui::Button("Apply##imp"))
            {
                const float len = std::sqrt(imp[0]*imp[0] + imp[1]*imp[1] + imp[2]*imp[2]);
                if (len > 1e-6f)
                {
                    const float s = imp_mag / len;
                    transport.push_command(CmdApplyImpulse{
                        b.id,
                        m3d::vec3{imp[0]*s, imp[1]*s, imp[2]*s},
                        b.position   // at COM
                    });
                }
            }
        }

        ImGui::End();
    }

    // =========================================================================
    //  Collider inspector panel  (logic unchanged from v1, layout tidied)
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

        if (ImGui::BeginChild("coll_list", ImVec2(0, 120), true))
        {
            for (const auto &c : snap.colliders)
            {
                char label[80];
                std::snprintf(label, sizeof(label), "[%u] %s (body %u)",
                              c.id, shape_name(c.shape), c.body_id);
                const bool selected = (sel.collider_id == c.id);
                if (ImGui::Selectable(label, selected))
                    sel.collider_id = selected ? UINT32_MAX : c.id;
            }
        }
        ImGui::EndChild();

        ImGui::Separator();

        if (sel.collider_id == UINT32_MAX)
        {
            ImGui::TextDisabled("(click a collider to inspect)");
            ImGui::End();
            return;
        }

        const ColliderSnap *found = nullptr;
        for (const auto &c : snap.colliders)
            if (c.id == sel.collider_id) { found = &c; break; }
        if (!found) { ImGui::End(); return; }

        ImGui::Text("Collider #%u  —  %s", found->id, shape_name(found->shape));
        ImGui::Text("Body #%u  |  %s",
                    found->body_id, found->is_static ? "STATIC" : "DYNAMIC");
        ImGui::Separator();
        show_vec3("world_pos",      found->world_pos);
        ImGui::Text("%-22s  %.4f", "restitution",    found->restitution);
        ImGui::Text("%-22s  %.4f", "static_friction", found->static_friction);
        ImGui::Text("%-22s  %.4f", "dyn_friction",   found->dynamic_friction);

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
                ImGui::Text("radius=%.4f  half_height=%.4f", sh.radius, sh.half_height);
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
                ImGui::Text("verts=%u  faces=%u", sh.vertex_count, sh.face_count);
        }, found->shape);

        ImGui::End();
    }

    // =========================================================================
    //  Contact list panel
    //  v2: clickable rows, colour-coded depth + lambda, body IDs, inactive filter
    // =========================================================================
    inline void draw_contact_panel(const FrameSnapshot &snap,
                                   SelectionState      &sel)
    {
        ImGui::Begin("Contacts");

        const size_t active_n = std::count_if(snap.contacts.begin(), snap.contacts.end(),
                                              [](const ContactSnap &c){ return c.active; });
        ImGui::Text("%zu contacts  (%zu active)", snap.contacts.size(), active_n);
        ImGui::SameLine(0, 16);
        static bool show_inactive = false;
        ImGui::Checkbox("Show inactive##ct", &show_inactive);

        ImGui::Separator();

        // Columns: # | BdyA | BdyB | Depth | λ_n | λ_t | V_rel | OK
        if (ImGui::BeginTable("contacts", 8,
            ImGuiTableFlags_Borders      |
            ImGuiTableFlags_ScrollY      |
            ImGuiTableFlags_RowBg        |
            ImGuiTableFlags_SizingFixedFit,
            ImVec2(0, 260)))
        {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("#",     ImGuiTableColumnFlags_WidthFixed, 28);
            ImGui::TableSetupColumn("BdyA",  ImGuiTableColumnFlags_WidthFixed, 44);
            ImGui::TableSetupColumn("BdyB",  ImGuiTableColumnFlags_WidthFixed, 44);
            ImGui::TableSetupColumn("Depth", ImGuiTableColumnFlags_WidthFixed, 72);
            ImGui::TableSetupColumn("λ_n",   ImGuiTableColumnFlags_WidthFixed, 68);
            ImGui::TableSetupColumn("λ_t",   ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableSetupColumn("V_rel", ImGuiTableColumnFlags_WidthFixed, 60);
            ImGui::TableSetupColumn("OK",    ImGuiTableColumnFlags_WidthFixed, 24);
            ImGui::TableHeadersRow();

            for (uint32_t i = 0; i < (uint32_t)snap.contacts.size(); ++i)
            {
                const ContactSnap &ct = snap.contacts[i];
                if (!ct.active && !show_inactive) continue;

                ImGui::TableNextRow();

                // Highlight rows for contacts involving the selected body
                const bool body_match = (sel.body_id != UINT32_MAX &&
                                         (ct.body_a == sel.body_id ||
                                          ct.body_b == sel.body_id));
                if (body_match)
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                          IM_COL32(80, 50, 0, 130));

                // Selectable in column 0 with SpanAllColumns so the whole row is clickable
                ImGui::TableSetColumnIndex(0);
                char row_id[16];
                std::snprintf(row_id, sizeof(row_id), "%u", i);
                const bool row_sel = (sel.contact_idx == i);
                if (ImGui::Selectable(row_id, row_sel,
                    ImGuiSelectableFlags_SpanAllColumns, ImVec2(0, 0)))
                {
                    sel.contact_idx = row_sel ? UINT32_MAX : i;
                }

                ImGui::TableSetColumnIndex(1); ImGui::Text("%u", ct.body_a);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%u", ct.body_b);

                // Depth — cell background colour-coded
                ImGui::TableSetColumnIndex(3);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                    ImGui::ColorConvertFloat4ToU32(
                        depth_color((float)ct.penetration_depth)));
                ImGui::Text("%.5f", ct.penetration_depth);

                // λ_n — cell background colour-coded
                ImGui::TableSetColumnIndex(4);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                    ImGui::ColorConvertFloat4ToU32(
                        lambda_color((float)ct.normal_lambda)));
                ImGui::Text("%.4f", ct.normal_lambda);

                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.4f", ct.tangent_lambda);

                ImGui::TableSetColumnIndex(6);
                {
                    const float vr = (float)ct.relative_velocity;
                    ImGui::TextColored(
                        vr < 0 ? ImVec4(1,0.4f,0.4f,1) : ImVec4(0.7f,1,0.7f,1),
                        "%.3f", vr);
                }

                ImGui::TableSetColumnIndex(7);
                ImGui::TextColored(ct.active ? ImVec4(0,1,0.4f,1) : ImVec4(0.5f,0.5f,0.5f,1),
                                   ct.active ? "Y" : "N");
            }
            ImGui::EndTable();
        }

        ImGui::End();
    }

    // =========================================================================
    //  Contact detail panel  (NEW in v2)
    //  Only opens when sel.contact_idx is valid.
    // =========================================================================
    inline void draw_contact_detail_panel(const FrameSnapshot  &snap,
                                          const SelectionState &sel)
    {
        if (sel.contact_idx == UINT32_MAX ||
            sel.contact_idx >= (uint32_t)snap.contacts.size())
            return;

        ImGui::Begin("Contact Detail");

        const ContactSnap &c = snap.contacts[sel.contact_idx];

        ImGui::TextColored(c.active ? ImVec4(0,1,0.4f,1) : ImVec4(1,0.4f,0.4f,1),
                           c.active ? "ACTIVE" : "INACTIVE");
        ImGui::SameLine(0, 12);
        ImGui::Text("Contact #%u", sel.contact_idx);
        ImGui::Separator();

        // Two-column table for cleaner layout
        if (ImGui::BeginTable("cd_tbl", 2, ImGuiTableFlags_BordersInnerV))
        {
            ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed, 140);
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);

            // Helper lambda to add one text row
            auto row = [](const char *label, const char *fmt, auto ...args)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                char buf[256];
                std::snprintf(buf, sizeof(buf), fmt, args...);
                ImGui::TextUnformatted(buf);
            };

            row("Body A (slot)",  "%u", c.body_a);
            row("Body B (slot)",  "%u", c.body_b);
            row("Collider A",     "%u", c.collider_a);
            row("Collider B",     "%u", c.collider_b);

            // Depth with colour
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Penetration");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(depth_color((float)c.penetration_depth),
                               "%.7f m", c.penetration_depth);

            row("Normal",
                "(%.5f,  %.5f,  %.5f)", c.normal.x, c.normal.y, c.normal.z);
            row("Point on A",
                "(%.4f,  %.4f,  %.4f)", c.point_on_a.x, c.point_on_a.y, c.point_on_a.z);
            row("Point on B",
                "(%.4f,  %.4f,  %.4f)", c.point_on_b.x, c.point_on_b.y, c.point_on_b.z);

            // λ_n with colour
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("λ normal");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(lambda_color((float)c.normal_lambda),
                               "%.7f", c.normal_lambda);

            row("λ tangential",  "%.7f",       c.tangent_lambda);
            row("Relative vel",  "%.5f m/s",   c.relative_velocity);
            row("Normal force",
                "(%.4f,  %.4f,  %.4f)", c.normal_force.x, c.normal_force.y, c.normal_force.z);
            row("Tangent force",
                "(%.4f,  %.4f,  %.4f)", c.tangent_force.x, c.tangent_force.y, c.tangent_force.z);

            ImGui::EndTable();
        }

        // ── Normal direction mini bar chart ───────────────────────────────
        // A progress bar centered on 0.5 makes it easy to see direction at a glance.
        ImGui::Separator();
        ImGui::TextDisabled("Normal components  (range [-1, 1]):");

        const float nx = (float)c.normal.x;
        const float ny = (float)c.normal.y;
        const float nz = (float)c.normal.z;
        const float bar_w = ImGui::GetContentRegionAvail().x - 46.0f;

        auto bar = [&](const char *lbl, float val)
        {
            const float frac = (val + 1.0f) * 0.5f;   // [-1,1] → [0,1]
            char overlay[24];
            std::snprintf(overlay, sizeof(overlay), "%.3f", val);
            ImGui::ProgressBar(frac, ImVec2(bar_w, 12), overlay);
            ImGui::SameLine(0, 6); ImGui::TextUnformatted(lbl);
        };
        bar("X", nx);
        bar("Y", ny);
        bar("Z", nz);

        // Sanity-check: normal should be unit length
        const float mag = std::sqrt(nx*nx + ny*ny + nz*nz);
        ImGui::Separator();
        if (std::fabs(mag - 1.0f) > 0.01f)
            ImGui::TextColored({1, 0.3f, 0.3f, 1},
                               "WARNING  |normal| = %.5f  (expected 1.0)", mag);
        else
            ImGui::TextColored({0.4f, 1, 0.4f, 1}, "|normal| = %.5f  OK", mag);

        ImGui::End();
    }

    // =========================================================================
    //  Joint panel
    //  v2: fixed static-float-from-pointer bug; live limit bar; deselect toggle.
    // =========================================================================
    inline void draw_joint_panel(const FrameSnapshot &snap,
                                 InProcessTransport  &transport,
                                 SelectionState      &sel)
    {
        ImGui::Begin("Joints");

        if (ImGui::BeginChild("joint_list", ImVec2(0, 120), true))
        {
            for (const auto &j : snap.joints)
            {
                const char *type_str =
                    j.type == JointTypeSnap::Prismatic ? "Prismatic" :
                    j.type == JointTypeSnap::Revolute  ? "Revolute"  : "Fixed";
                char label[80];
                std::snprintf(label, sizeof(label), "[%u] %s  (b%u↔b%u)",
                              j.id, type_str, j.body_a, j.body_b);
                const bool selected = (sel.joint_id == j.id);
                if (ImGui::Selectable(label, selected))
                    sel.joint_id = selected ? UINT32_MAX : j.id;
            }
        }
        ImGui::EndChild();

        ImGui::Separator();

        if (sel.joint_id == UINT32_MAX)
        {
            ImGui::TextDisabled("(click a joint to inspect)");
            ImGui::End();
            return;
        }

        const JointSnap *found = nullptr;
        for (const auto &j : snap.joints)
            if (j.id == sel.joint_id) { found = &j; break; }
        if (!found) { ImGui::End(); return; }

        const JointSnap &j = *found;

        ImGui::Text("Joint #%u  |  b%u ↔ b%u", j.id, j.body_a, j.body_b);
        ImGui::Text("damping = %.4f", j.damping);
        ImGui::Separator();

        // ── Live limit bar ────────────────────────────────────────────────
        if (j.limited)
        {
            const float pos   = (float)j.current_position;
            const float lo    = (float)j.lower_limit;
            const float hi    = (float)j.upper_limit;
            const float range = hi - lo;
            const bool  out   = (pos < lo || pos > hi);

            ImGui::Text("pos = %.4f   [%.3f, %.3f]", pos, lo, hi);

            // ProgressBar fraction clamped to [0,1]
            const float frac = range > 1e-6f
                ? std::max(0.0f, std::min((pos - lo) / range, 1.0f))
                : 0.5f;

            char overlay[32];
            std::snprintf(overlay, sizeof(overlay), "%.3f", pos);

            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                out ? ImVec4(1, 0.3f, 0.3f, 1) : ImVec4(0.3f, 0.9f, 0.3f, 1));
            ImGui::ProgressBar(frac, ImVec2(-1, 14), overlay);
            ImGui::PopStyleColor();

            if (out)
                ImGui::TextColored({1, 0.3f, 0.3f, 1}, "  OUTSIDE LIMITS");
        }
        else
        {
            ImGui::Text("pos = %.4f   (unlimited)", j.current_position);
        }

        ImGui::Separator();

        // ── Target / damping controls ─────────────────────────────────────
        // Do NOT use static here — statics are initialised only once and would
        // show the first joint's values even after switching to a different joint.
        // Use plain local floats (ImGui SliderFloat works on a frame-local copy).
        const float lo_f = j.limited ? (float)j.lower_limit : -10.0f;
        const float hi_f = j.limited ? (float)j.upper_limit :  10.0f;

        float target  = (float)j.current_position;
        float damping = (float)j.damping;

        if (ImGui::SliderFloat("Target##jt",  &target,  lo_f,   hi_f))  { /* editing */ }
        if (ImGui::Button("Set Target##jt"))
            transport.push_command(CmdSetJointTarget{j.id, (m3d::scalar)target});

        if (ImGui::SliderFloat("Damping##jd", &damping, 0.0f, 100.0f))  { /* editing */ }
        if (ImGui::Button("Set Damping##jd"))
            transport.push_command(CmdSetJointDamping{j.id, (m3d::scalar)damping});

        ImGui::End();
    }

    // =========================================================================
    //  Live graph panel (tracked quantities)  — logic unchanged from v1
    // =========================================================================
    template<typename T>
    inline void draw_graph_panel(DebugChannel<T>     &channel,
                                 InProcessTransport  &transport,
                                 const FrameSnapshot &snap)
    {
        ImGui::Begin("Graphs");

        static uint32_t track_id = 0;
        ImGui::InputScalar("Track body ID", ImGuiDataType_U32, &track_id);

        if (ImGui::Button("Lin Speed"))
            transport.push_command(CmdTrackQuantity{
                TrackTarget::BodyLinearSpeed, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Ang Speed"))
            transport.push_command(CmdTrackQuantity{
                TrackTarget::BodyAngularSpeed, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Pos Y"))
            transport.push_command(CmdTrackQuantity{
                TrackTarget::BodyPositionY, track_id, true});

        ImGui::Separator();

        for (const auto &series : channel.tracked_series())
        {
            if (series.samples.empty()) continue;

            std::vector<float> vals;
            vals.reserve(series.samples.size());
            float vmin =  1e30f, vmax = -1e30f;
            for (const auto &s : series.samples)
            {
                vals.push_back(s.value);
                vmin = std::min(vmin, s.value);
                vmax = std::max(vmax, s.value);
            }
            if (vmax - vmin < 1e-6f) { vmin -= 0.1f; vmax += 0.1f; }

            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "%.3f", vals.back());
            char title[80];
            std::snprintf(title, sizeof(title), "id=%u  [%.2f, %.2f]",
                          series.id, vmin, vmax);

            ImGui::PlotLines(title, vals.data(), (int)vals.size(),
                             0, overlay, vmin, vmax, ImVec2(0, 70));
        }

        ImGui::End();
    }

    // =========================================================================
    //  Master draw — call once per render frame
    // =========================================================================
    template<typename T>
    inline void draw_all(DebugChannel<T>     &channel,
                         InProcessTransport  &transport,
                         const FrameSnapshot &snap,
                         SelectionState      &sel)
    {
        draw_sim_control_panel   (snap, transport, channel);   // v2: owns pause directly
        draw_body_panel          (snap, transport, sel);
        draw_collider_panel      (snap, sel);
        draw_contact_panel       (snap, sel);
        draw_contact_detail_panel(snap, sel);                  // only shows if contact selected
        draw_joint_panel         (snap, transport, sel);
        draw_graph_panel         (channel, transport, snap);
    }

} // namespace visr::ui