#pragma once
#include <imgui.h>
#include <implot.h>
#include <string>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "visr/Snapshot.hpp"
#include "visr/Command.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

// ============================================================================
//  visr/ui/Panels.hpp
//
//  Self-contained ImGui/ImPlot panels for the visr debugger.
//
//  Global flags (inline variables, C++17):
//    g_screenshot_requested  set by graph panel, read+cleared by RenderSystem
//    g_export_status         last export result string, shown as toast
//    g_export_status_time    GetTime() at last export, for toast fade-out
//
//  Changelog v4:
//    - draw_graph_panel: replaced ImGui::PlotLines with ImPlot, added
//      "Copy CSV" clipboard export and "Save PNG" screenshot request.
//    - Root bug fix is in DebugChannel::poll() (see DebugChannel.hpp).
// ============================================================================

namespace visr::ui
{
    // ── Global flags shared with RenderSystem ─────────────────────────────────
    // These are inline so they have exactly one definition across TUs.
    inline bool        g_screenshot_requested = false;
    inline std::string g_export_status;
    inline double      g_export_status_time   = -999.0;

    // -------------------------------------------------------------------------
    //  Shared selection state
    // -------------------------------------------------------------------------
    struct SelectionState
    {
        uint32_t body_id     = UINT32_MAX;
        uint32_t collider_id = UINT32_MAX;
        uint32_t joint_id    = UINT32_MAX;
        uint32_t contact_idx = UINT32_MAX;
    };

    // -------------------------------------------------------------------------
    //  Colour helpers
    // -------------------------------------------------------------------------
    static inline ImVec4 depth_color(float depth, float max_depth = 0.15f)
    {
        const float t = std::min(depth / max_depth, 1.0f);
        if (t < 0.5f) return ImVec4(t * 2.0f, 1.0f, 0.0f, 1.0f);
        return             ImVec4(1.0f, 1.0f - (t - 0.5f) * 2.0f, 0.0f, 1.0f);
    }

    static inline ImVec4 lambda_color(float lambda)
    {
        const float t = std::min(std::fabs(lambda) / 5.0f, 1.0f);
        return lambda < 0.0f
            ? ImVec4(1.0f - t * 0.5f, 1.0f - t * 0.5f, 1.0f, 1.0f)
            : ImVec4(1.0f, 1.0f - t * 0.5f, 1.0f - t * 0.5f, 1.0f);
    }

    static void show_vec3(const char *label, const m3d::vec3 &v)
    {
        ImGui::Text("%-22s  %8.4f  %8.4f  %8.4f", label, v.x, v.y, v.z);
    }

    // =========================================================================
    //  Help panel
    // =========================================================================
    inline void draw_help_panel()
    {
        ImGui::SetNextWindowSize({340, 320}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({10,  840}, ImGuiCond_FirstUseEver);

        if (!ImGui::Begin("Help / Controls")) { ImGui::End(); return; }

        ImGui::SeparatorText("Camera");
        ImGui::TextDisabled("  W/S               forward / back");
        ImGui::TextDisabled("  A/D               strafe left / right");
        ImGui::TextDisabled("  Q / LShift        up");
        ImGui::TextDisabled("  E / LCtrl         down");
        ImGui::TextDisabled("  RMB drag          look (yaw + pitch)");
        ImGui::TextDisabled("  Scroll wheel      zoom (FOV)");
        ImGui::TextDisabled("  Camera panel      adjust speed & sensitivity");

        ImGui::SeparatorText("Selection");
        ImGui::TextDisabled("  LMB click         select body");
        ImGui::TextDisabled("  LMB same body     deselect");
        ImGui::TextDisabled("  LMB empty space   deselect all");
        ImGui::TextDisabled("  ESC               deselect all");
        ImGui::TextDisabled("  Contact/Joint row click to select, click again to deselect");

        ImGui::SeparatorText("Simulation");
        ImGui::TextDisabled("  Space             pause / resume");
        ImGui::TextDisabled("  Step Once button  advance 1 frame while paused");

        ImGui::SeparatorText("Graphs");
        ImGui::TextDisabled("  Copy CSV          all series → clipboard");
        ImGui::TextDisabled("  Save PNG          full-window screenshot → PNG file");
        ImGui::TextDisabled("  Stop button       remove a tracked series");

        ImGui::SeparatorText("Colour coding");
        ImGui::TextDisabled("  Depth cell        green=shallow  red=deep");
        ImGui::TextDisabled("  λ cell            blue=compression  pink=tension");
        ImGui::TextDisabled("  |normal| check    red warning if not unit length");

        ImGui::End();
    }

    // =========================================================================
    //  Simulation control panel
    // =========================================================================
    template<typename T>
    inline void draw_sim_control_panel(const FrameSnapshot &snap,
                                       InProcessTransport  &transport,
                                       DebugChannel<T>     &channel,
                                       SelectionState      &sel)
    {
        ImGui::Begin("Simulation");

        ImGui::Text("Frame : %llu", (unsigned long long)snap.frame_index);
        ImGui::Text("Time  : %.4f s", snap.sim_time);
        ImGui::Separator();

        const bool paused = channel.is_paused();
        if (ImGui::Button(paused ? "Resume [Space]" : "Pause  [Space]"))
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

        if (ImGui::Button("Deselect All [ESC]"))
        {
            sel.body_id = sel.collider_id = sel.joint_id = sel.contact_idx = UINT32_MAX;
            transport.push_command(CmdClearSelection{});
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
        { ImGui::TextDisabled("(click a body or LMB in 3D to inspect)"); ImGui::End(); return; }

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

        const float lspeed = (float)std::sqrt(
            b.linear_velocity.x * b.linear_velocity.x +
            b.linear_velocity.y * b.linear_velocity.y +
            b.linear_velocity.z * b.linear_velocity.z);
        const float aspeed = (float)std::sqrt(
            b.angular_velocity.x * b.angular_velocity.x +
            b.angular_velocity.y * b.angular_velocity.y +
            b.angular_velocity.z * b.angular_velocity.z);
        ImGui::Text("%-22s  %8.4f  m/s",   "|lin_velocity|", lspeed);
        ImGui::Text("%-22s  %8.4f  rad/s", "|ang_velocity|", aspeed);
        ImGui::Text("%-22s  %8.4f", "mass",     b.mass);
        ImGui::Text("%-22s  %8.4f", "inv_mass", b.inverse_mass);

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
            std::snprintf(clabel, sizeof(clabel), "  ct#%u  depth=%.4f  λn=%.3f  %s",
                          i, c.penetration_depth, c.normal_lambda,
                          c.active ? "" : "[inactive]");
            if (ImGui::Selectable(clabel, is_sel))
                sel.contact_idx = is_sel ? UINT32_MAX : i;
        }
        if (ncont == 0) ImGui::TextDisabled("  (none)");

        ImGui::Separator();
        if (!b.is_static)
        {
            if (ImGui::Button("Zero Velocity"))
                transport.push_command(CmdZeroVelocity{b.id});
            ImGui::SameLine();
            if (ImGui::Button("Teleport to origin"))
                transport.push_command(CmdTeleportBody{b.id, {}, {}});

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
                        b.id, m3d::vec3{imp[0]*s, imp[1]*s, imp[2]*s}, b.position});
                }
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

    inline void draw_collider_panel(const FrameSnapshot &snap, SelectionState &sel)
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
        { ImGui::TextDisabled("(click a collider to inspect)"); ImGui::End(); return; }

        const ColliderSnap *found = nullptr;
        for (const auto &c : snap.colliders)
            if (c.id == sel.collider_id) { found = &c; break; }
        if (!found) { ImGui::End(); return; }

        ImGui::Text("Collider #%u  —  %s", found->id, shape_name(found->shape));
        ImGui::Text("Body #%u  |  %s", found->body_id,
                    found->is_static ? "STATIC" : "DYNAMIC");
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
                ImGui::Text("grid %u × %u  cell=%.4f", sh.cols, sh.rows, sh.cell_size);
            else if constexpr (std::is_same_v<T, MeshSnap>)
                ImGui::Text("verts=%u  faces=%u", sh.vertex_count, sh.face_count);
        }, found->shape);
        ImGui::End();
    }

    // =========================================================================
    //  Contact list panel
    // =========================================================================
    inline void draw_contact_panel(const FrameSnapshot &snap, SelectionState &sel)
    {
        ImGui::Begin("Contacts");
        const size_t active_n = std::count_if(snap.contacts.begin(), snap.contacts.end(),
                                              [](const ContactSnap &c){ return c.active; });
        ImGui::Text("%zu contacts  (%zu active)", snap.contacts.size(), active_n);
        ImGui::SameLine(0, 16);
        static bool show_inactive = false;
        ImGui::Checkbox("Show inactive##ct", &show_inactive);
        ImGui::Separator();

        if (ImGui::BeginTable("contacts", 8,
            ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY |
            ImGuiTableFlags_RowBg   | ImGuiTableFlags_SizingFixedFit,
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
                const bool body_match = (sel.body_id != UINT32_MAX &&
                                         (ct.body_a == sel.body_id ||
                                          ct.body_b == sel.body_id));
                if (body_match)
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                          IM_COL32(80, 50, 0, 130));

                ImGui::TableSetColumnIndex(0);
                char row_id[16]; std::snprintf(row_id, sizeof(row_id), "%u", i);
                const bool row_sel = (sel.contact_idx == i);
                if (ImGui::Selectable(row_id, row_sel,
                    ImGuiSelectableFlags_SpanAllColumns, ImVec2(0, 0)))
                    sel.contact_idx = row_sel ? UINT32_MAX : i;

                ImGui::TableSetColumnIndex(1); ImGui::Text("%u", ct.body_a);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%u", ct.body_b);
                ImGui::TableSetColumnIndex(3);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                    ImGui::ColorConvertFloat4ToU32(depth_color((float)ct.penetration_depth)));
                ImGui::Text("%.5f", ct.penetration_depth);
                ImGui::TableSetColumnIndex(4);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                    ImGui::ColorConvertFloat4ToU32(lambda_color((float)ct.normal_lambda)));
                ImGui::Text("%.4f", ct.normal_lambda);
                ImGui::TableSetColumnIndex(5); ImGui::Text("%.4f", ct.tangent_lambda);
                ImGui::TableSetColumnIndex(6);
                {
                    const float vr = (float)ct.relative_velocity;
                    ImGui::TextColored(vr < 0 ? ImVec4(1,0.4f,0.4f,1) : ImVec4(0.7f,1,0.7f,1),
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
    //  Contact detail panel
    // =========================================================================
    inline void draw_contact_detail_panel(const FrameSnapshot &snap,
                                          const SelectionState &sel)
    {
        if (sel.contact_idx == UINT32_MAX ||
            sel.contact_idx >= (uint32_t)snap.contacts.size()) return;

        ImGui::Begin("Contact Detail");
        const ContactSnap &c = snap.contacts[sel.contact_idx];

        ImGui::TextColored(c.active ? ImVec4(0,1,0.4f,1) : ImVec4(1,0.4f,0.4f,1),
                           c.active ? "ACTIVE" : "INACTIVE");
        ImGui::SameLine(0, 12); ImGui::Text("Contact #%u", sel.contact_idx);
        ImGui::Separator();

        if (ImGui::BeginTable("cd_tbl", 2, ImGuiTableFlags_BordersInnerV))
        {
            ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed, 140);
            ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
            auto row = [](const char *label, const char *fmt, auto ...args)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                char buf[256]; std::snprintf(buf, sizeof(buf), fmt, args...);
                ImGui::TextUnformatted(buf);
            };
            row("Body A",        "%u", c.body_a);
            row("Body B",        "%u", c.body_b);
            row("Collider A",    "%u", c.collider_a);
            row("Collider B",    "%u", c.collider_b);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Penetration");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(depth_color((float)c.penetration_depth),
                               "%.7f m", c.penetration_depth);
            row("Normal",     "(%.5f,  %.5f,  %.5f)", c.normal.x, c.normal.y, c.normal.z);
            row("Point on A", "(%.4f,  %.4f,  %.4f)",
                c.point_on_a.x, c.point_on_a.y, c.point_on_a.z);
            row("Point on B", "(%.4f,  %.4f,  %.4f)",
                c.point_on_b.x, c.point_on_b.y, c.point_on_b.z);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("λ normal");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(lambda_color((float)c.normal_lambda),
                               "%.7f", c.normal_lambda);
            row("λ tangential", "%.7f",       c.tangent_lambda);
            row("Relative vel", "%.5f m/s",   c.relative_velocity);
            row("Normal force", "(%.4f,  %.4f,  %.4f)",
                c.normal_force.x, c.normal_force.y, c.normal_force.z);
            row("Tangent force","(%.4f,  %.4f,  %.4f)",
                c.tangent_force.x, c.tangent_force.y, c.tangent_force.z);
            ImGui::EndTable();
        }

        ImGui::Separator();
        ImGui::TextDisabled("Normal components  [-1 … 1]:");
        const float nx = (float)c.normal.x, ny = (float)c.normal.y, nz = (float)c.normal.z;
        const float bar_w = ImGui::GetContentRegionAvail().x - 46.0f;
        auto bar = [&](const char *lbl, float val)
        {
            char ov[24]; std::snprintf(ov, sizeof(ov), "%.3f", val);
            ImGui::ProgressBar((val + 1.0f) * 0.5f, ImVec2(bar_w, 12), ov);
            ImGui::SameLine(0, 6); ImGui::TextUnformatted(lbl);
        };
        bar("X", nx); bar("Y", ny); bar("Z", nz);
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
        { ImGui::TextDisabled("(click a joint to inspect)"); ImGui::End(); return; }

        const JointSnap *found = nullptr;
        for (const auto &j : snap.joints)
            if (j.id == sel.joint_id) { found = &j; break; }
        if (!found) { ImGui::End(); return; }

        const JointSnap &j = *found;
        ImGui::Text("Joint #%u  |  b%u ↔ b%u", j.id, j.body_a, j.body_b);
        ImGui::Text("damping = %.4f", j.damping);
        ImGui::Separator();

        if (j.limited)
        {
            const float pos   = (float)j.current_position;
            const float lo    = (float)j.lower_limit;
            const float hi    = (float)j.upper_limit;
            const bool  out   = (pos < lo || pos > hi);
            ImGui::Text("pos = %.4f   [%.3f, %.3f]", pos, lo, hi);
            const float range = hi - lo;
            const float frac  = range > 1e-6f
                ? std::max(0.0f, std::min((pos - lo) / range, 1.0f)) : 0.5f;
            char overlay[32]; std::snprintf(overlay, sizeof(overlay), "%.3f", pos);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                out ? ImVec4(1, 0.3f, 0.3f, 1) : ImVec4(0.3f, 0.9f, 0.3f, 1));
            ImGui::ProgressBar(frac, ImVec2(-1, 14), overlay);
            ImGui::PopStyleColor();
            if (out) ImGui::TextColored({1, 0.3f, 0.3f, 1}, "  OUTSIDE LIMITS");
        }
        else
            ImGui::Text("pos = %.4f   (unlimited)", j.current_position);

        ImGui::Separator();
        const float lo_f = j.limited ? (float)j.lower_limit : -10.0f;
        const float hi_f = j.limited ? (float)j.upper_limit :  10.0f;
        float target  = (float)j.current_position;
        float damping = (float)j.damping;
        ImGui::SliderFloat("Target##jt",  &target,  lo_f,   hi_f);
        if (ImGui::Button("Set Target##jt"))
            transport.push_command(CmdSetJointTarget{j.id, (m3d::scalar)target});
        ImGui::SliderFloat("Damping##jd", &damping, 0.0f, 100.0f);
        if (ImGui::Button("Set Damping##jd"))
            transport.push_command(CmdSetJointDamping{j.id, (m3d::scalar)damping});
        ImGui::End();
    }

    // =========================================================================
    //  Graph panel — ImPlot line charts with CSV clipboard + PNG screenshot
    // =========================================================================

    // Human-readable label for each TrackTarget enum value.
    static inline const char *track_target_name(TrackTarget t)
    {
        switch (t)
        {
        case TrackTarget::BodyLinearSpeed:       return "LinSpeed";
        case TrackTarget::BodyAngularSpeed:      return "AngSpeed";
        case TrackTarget::BodyPositionX:         return "PosX";
        case TrackTarget::BodyPositionY:         return "PosY";
        case TrackTarget::BodyPositionZ:         return "PosZ";
        case TrackTarget::ContactNormalLambda:   return "λ_normal";
        case TrackTarget::ContactTangentLambda:  return "λ_tangent";
        case TrackTarget::ContactPenetrationDepth: return "Depth";
        case TrackTarget::JointPosition:         return "JointPos";
        case TrackTarget::JointError:            return "JointErr";
        case TrackTarget::ConstraintLambda:      return "ConstraintλG";
        default:                                 return "Value";
        }
    }

    // Linearise a ring buffer into chronological order.
    // Returns x (frame index) and y (value) arrays ready for ImPlot.
    static inline void linearise_series(const TrackedSeries &series,
                                        std::vector<double> &xs,
                                        std::vector<double> &ys)
    {
        const size_t n = series.samples.size();
        xs.clear(); ys.clear();
        if (n == 0) return;
        xs.reserve(n); ys.reserve(n);

        // When the ring is full, head_ points to the oldest slot.
        // When not full, samples are in append order starting at index 0.
        const bool is_full = (n >= TRACK_HISTORY_DEPTH);
        const size_t start = is_full ? (series.head_ % n) : 0;

        for (size_t k = 0; k < n; ++k)
        {
            const auto &s = series.samples[(start + k) % n];
            xs.push_back(static_cast<double>(s.frame));
            ys.push_back(static_cast<double>(s.value));
        }
    }

    template<typename T>
    inline void draw_graph_panel(DebugChannel<T>     &channel,
                                 InProcessTransport  &transport)
    {
        ImGui::SetNextWindowSize({520, 420}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({900, 480}, ImGuiCond_FirstUseEver);
        ImGui::Begin("Graphs");

        // ── Track-quantity selector ────────────────────────────────────────
        static uint32_t track_id = 0;
        ImGui::InputScalar("Body ID##tid", ImGuiDataType_U32, &track_id);

        if (ImGui::Button("Lin Speed"))
            transport.push_command(CmdTrackQuantity{TrackTarget::BodyLinearSpeed, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Ang Speed"))
            transport.push_command(CmdTrackQuantity{TrackTarget::BodyAngularSpeed, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Pos X"))
            transport.push_command(CmdTrackQuantity{TrackTarget::BodyPositionX, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Pos Y"))
            transport.push_command(CmdTrackQuantity{TrackTarget::BodyPositionY, track_id, true});
        ImGui::SameLine();
        if (ImGui::Button("Pos Z"))
            transport.push_command(CmdTrackQuantity{TrackTarget::BodyPositionZ, track_id, true});

        ImGui::Separator();

        const auto &all_series = channel.tracked_series();

        // ── Export buttons ─────────────────────────────────────────────────
        if (!all_series.empty())
        {
            // ── Copy CSV to clipboard ──────────────────────────────────────
            // One header row + one data row per sample.
            // Columns: frame, then one column per series.
            if (ImGui::Button("Copy CSV"))
            {
                // Build header
                std::string csv = "frame";
                for (const auto &s : all_series)
                {
                    csv += ',';
                    csv += track_target_name(s.target);
                    csv += '_';
                    csv += std::to_string(s.id);
                }
                csv += '\n';

                // Find the max number of samples across all series
                size_t max_n = 0;
                for (const auto &s : all_series) max_n = std::max(max_n, s.samples.size());

                // Linearise each series once
                std::vector<std::vector<double>> all_xs(all_series.size());
                std::vector<std::vector<double>> all_ys(all_series.size());
                for (size_t si = 0; si < all_series.size(); ++si)
                    linearise_series(all_series[si], all_xs[si], all_ys[si]);

                // Data rows — align on frame index of the longest series
                for (size_t k = 0; k < max_n; ++k)
                {
                    // Use the frame column from the first series that has index k
                    double frame_val = (k < all_xs[0].size()) ? all_xs[0][k]
                                     : static_cast<double>(k);
                    char row[64]; std::snprintf(row, sizeof(row), "%.0f", frame_val);
                    csv += row;

                    for (size_t si = 0; si < all_series.size(); ++si)
                    {
                        csv += ',';
                        if (k < all_ys[si].size())
                        {
                            char cell[32];
                            std::snprintf(cell, sizeof(cell), "%.6f", all_ys[si][k]);
                            csv += cell;
                        }
                        // else: empty cell (series has fewer samples)
                    }
                    csv += '\n';
                }

                ImGui::SetClipboardText(csv.c_str());
            }

            ImGui::SameLine();

            // ── Save PNG (full-window screenshot) ──────────────────────────
            // Sets g_screenshot_requested; RenderSystem takes the screenshot
            // after EndDrawing() of this frame.
            if (ImGui::Button("Save PNG"))
                g_screenshot_requested = true;

            // Toast: show export status for ~3 seconds after the last export
            if (!g_export_status.empty())
            {
                ImGui::SameLine(0, 16);
                // Fade out after 3 seconds
                ImGui::TextColored({0.4f, 1.0f, 0.4f, 1.0f}, "%s", g_export_status.c_str());
            }

            ImGui::Separator();
        }
        else
        {
            ImGui::TextDisabled("No series tracked yet.  Add one above.");
            ImGui::Separator();
        }

        // ── Active series list with stop buttons ───────────────────────────
        uint32_t stop_id       = UINT32_MAX;
        TrackTarget stop_target = TrackTarget::BodyLinearSpeed;
        {
            char label[80];
            for (const auto &s : all_series)
            {
                std::snprintf(label, sizeof(label), "Stop  %s id=%u",
                              track_target_name(s.target), s.id);
                if (ImGui::SmallButton(label))
                {
                    stop_id     = s.id;
                    stop_target = s.target;
                }
                ImGui::SameLine(0, 12);
                ImGui::TextDisabled("%s  id=%u  (%zu samples)",
                                    track_target_name(s.target), s.id,
                                    s.samples.size());
            }
        }
        if (stop_id != UINT32_MAX)
            transport.push_command(CmdTrackQuantity{stop_target, stop_id, false});

        if (!all_series.empty())
            ImGui::Separator();

        // ── ImPlot: one line per series, all in the same plot ──────────────
        if (!all_series.empty())
        {
            // Compute overall x range across all series for axis setup
            double x_min =  1e18, x_max = -1e18;
            double y_min =  1e18, y_max = -1e18;

            std::vector<std::vector<double>> xs_cache(all_series.size());
            std::vector<std::vector<double>> ys_cache(all_series.size());

            for (size_t si = 0; si < all_series.size(); ++si)
            {
                linearise_series(all_series[si], xs_cache[si], ys_cache[si]);
                for (double v : xs_cache[si]) { x_min = std::min(x_min, v); x_max = std::max(x_max, v); }
                for (double v : ys_cache[si]) { y_min = std::min(y_min, v); y_max = std::max(y_max, v); }
            }

            // Add a small margin around y range so lines aren't flush with axes
            const double y_margin = (y_max - y_min) * 0.08 + 1e-6;
            y_min -= y_margin; y_max += y_margin;

            ImPlot::SetNextAxesLimits(x_min, x_max, y_min, y_max, ImGuiCond_Always);

            if (ImPlot::BeginPlot("##tracked", ImVec2(-1, 220),
                                  ImPlotFlags_NoTitle))
            {
                ImPlot::SetupAxes("frame", "value",
                                  ImPlotAxisFlags_AutoFit,
                                  ImPlotAxisFlags_AutoFit);

                for (size_t si = 0; si < all_series.size(); ++si)
                {
                    const auto &xs = xs_cache[si];
                    const auto &ys = ys_cache[si];
                    if (xs.empty()) continue;

                    char series_label[64];
                    std::snprintf(series_label, sizeof(series_label),
                                  "%s[%u]",
                                  track_target_name(all_series[si].target),
                                  all_series[si].id);

                    ImPlot::PlotLine(series_label,
                                     xs.data(), ys.data(),
                                     static_cast<int>(xs.size()));
                }

                ImPlot::EndPlot();
            }

            // ── Last-value readout below the plot ──────────────────────────
            for (const auto &s : all_series)
            {
                if (s.samples.empty()) continue;
                // Most recent value is always at the tail of the ring
                const float last = s.samples.empty() ? 0.0f
                    : s.samples[(s.head_ > 0 ? (s.head_ - 1) % s.samples.size()
                                              : s.samples.size() - 1)].value;
                ImGui::Text("  %s[%u] = %.5f",
                            track_target_name(s.target), s.id, last);
            }
        }

        ImGui::End();
    }

    // =========================================================================
    //  Master draw — call once per render frame between rlImGuiBegin/End
    // =========================================================================
    template<typename T>
    inline void draw_all(DebugChannel<T>     &channel,
                         InProcessTransport  &transport,
                         const FrameSnapshot &snap,
                         SelectionState      &sel)
    {
        draw_sim_control_panel   (snap, transport, channel, sel);
        draw_body_panel          (snap, transport, sel);
        draw_collider_panel      (snap, sel);
        draw_contact_panel       (snap, sel);
        draw_contact_detail_panel(snap, sel);
        draw_joint_panel         (snap, transport, sel);
        draw_graph_panel         (channel, transport);   // no snap needed
        draw_help_panel          ();
    }

} // namespace visr::ui