#pragma once
#include <imgui.h>
#include <implot.h>
#include <string>
#include <cmath>
#include <cstring>
#include <ctime>
#include <algorithm>
#include "visr/Snapshot.hpp"
#include "visr/Command.hpp"
#include "visr/InProcessTransport.hpp"
#include "visr/DebugChannel.hpp"

// ============================================================================
//  visr/ui/Panels.hpp
//
//  Graph panel v4 changes:
//    - x-axis uses sim_time (seconds) instead of raw frame index.
//    - No samples are recorded while paused (DebugChannel fix).
//    - Export button opens a modal with filename input, upscale selector,
//      and output-size preview.  On confirm, RenderSystem takes a post-frame
//      screenshot, crops to the plot rect, and writes the PNG.
//    - CSV clipboard copies frame, sim_time, and all series values.
//
//  Global state (inline, C++17 — one definition across all TUs):
//    g_plot_export_request  — written by the modal; read + cleared by RenderSystem
//    g_export_status        — result string shown as a toast
//    g_export_status_time   — GetTime() of last export (for 4-second toast)
// ============================================================================

namespace visr::ui
{
    // ── Export request: written by modal, consumed by RenderSystem ────────────
    struct PlotExportRequest
    {
        bool    pending  = false;
        ImVec2  pos      = {0, 0};     // plot screen position (ImGui coords)
        ImVec2  size     = {0, 0};     // plot screen size     (ImGui coords)
        int     upscale  = 1;          // 1, 2, or 4
        char    filename[128] = "visr_plot";
    };

    inline PlotExportRequest g_plot_export_request;
    inline std::string       g_export_status;
    inline double            g_export_status_time = -999.0;

    // ── Full decorated plot widget rect (set BEFORE BeginPlot each frame) ──────
    // Covers the ENTIRE ImPlot widget: inner canvas + tick labels + axis names.
    // GetPlotPos/Size (inner canvas only) must NOT be used for the PNG crop —
    // that was the original bug where axis labels were missing from exports.
    inline ImVec2 g_last_plot_pos  = {0, 0};
    inline ImVec2 g_last_plot_size = {0, 0};

    // ── Modal open flag (set by Export button, cleared by modal) ─────────────
    inline bool g_export_modal_open = false;

    // ── Auto-fit toggle: re-fit view to data every frame when ON. ────────────
    // When OFF the user can freely pan/zoom; hit "Fit" to snap back to data.
    inline bool g_auto_fit = true;

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

    // ── Colour helpers ────────────────────────────────────────────────────────
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
        ImGui::SetNextWindowSize({340, 330}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({10,  840}, ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Help / Controls")) { ImGui::End(); return; }

        ImGui::SeparatorText("Camera");
        ImGui::TextDisabled("  W/S               forward / back");
        ImGui::TextDisabled("  A/D               strafe left / right");
        ImGui::TextDisabled("  Q / LShift        up");
        ImGui::TextDisabled("  E / LCtrl         down");
        ImGui::TextDisabled("  RMB drag          look (yaw + pitch)");
        ImGui::TextDisabled("  Scroll wheel      zoom (FOV)");
        ImGui::TextDisabled("  Camera panel      speed & sensitivity sliders");

        ImGui::SeparatorText("Selection");
        ImGui::TextDisabled("  LMB click         select body");
        ImGui::TextDisabled("  LMB same body     deselect");
        ImGui::TextDisabled("  LMB empty / ESC   deselect all");
        ImGui::TextDisabled("  Contact/Joint row click = select, re-click = deselect");

        ImGui::SeparatorText("Simulation");
        ImGui::TextDisabled("  Space             pause / resume");
        ImGui::TextDisabled("  Step Once         advance 1 frame while paused");

        ImGui::SeparatorText("Graphs");
        ImGui::TextDisabled("  x-axis              simulation time (seconds)");
        ImGui::TextDisabled("  Paused              graph flatlines (no new points)");
        ImGui::TextDisabled("  Auto-fit checkbox   ON = re-fit to data every frame");
        ImGui::TextDisabled("                      OFF = free pan/zoom");
        ImGui::TextDisabled("  Fit button          snap view to all data once");
        ImGui::TextDisabled("  Scroll wheel        zoom in/out on hovered axis");
        ImGui::TextDisabled("  Click + drag        pan the view");
        ImGui::TextDisabled("  Double-click        reset zoom to default");
        ImGui::TextDisabled("  Copy CSV            all series to clipboard");
        ImGui::TextDisabled("  Export Plot PNG     opens modal (full widget incl. axes)");
        ImGui::TextDisabled("    Rename axes in the modal before exporting");
        ImGui::TextDisabled("  Stop button         remove a tracked series");

        ImGui::SeparatorText("Colour coding");
        ImGui::TextDisabled("  Depth cell        green=shallow  red=deep");
        ImGui::TextDisabled("  lambda cell       blue=compression  pink=tension");
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
        changed |= ImGui::InputDouble("Timestep", &ts, 0.001, 0.0, "%.6f");
        changed |= ImGui::InputInt("Substeps", &substeps);
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
            ImGui::TextColored(depth_color(max_depth), "Max depth: %.5f m", max_depth);

        ImGui::End();
    }

    // =========================================================================
    //  Body panel
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
                    transport.push_command(selected ? Command{CmdClearSelection{}}
                                                    : Command{CmdSelectBody{b.id}});
                }
            }
        }
        ImGui::EndChild();
        ImGui::Separator();

        if (sel.body_id == UINT32_MAX)
        { ImGui::TextDisabled("(click a body or LMB in 3D)"); ImGui::End(); return; }

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
        show_vec3("orientation",    m3d::to_rpy(b.orientation));
        show_vec3("lin_velocity", b.linear_velocity);
        show_vec3("ang_velocity", b.angular_velocity);
        show_vec3("force",        b.force);
        show_vec3("torque",       b.torque);

        const float lspeed = std::sqrt((float)(b.linear_velocity.x * b.linear_velocity.x +
                                               b.linear_velocity.y * b.linear_velocity.y +
                                               b.linear_velocity.z * b.linear_velocity.z));
        const float aspeed = std::sqrt((float)(b.angular_velocity.x * b.angular_velocity.x +
                                               b.angular_velocity.y * b.angular_velocity.y +
                                               b.angular_velocity.z * b.angular_velocity.z));
        ImGui::Text("%-22s  %8.4f  m/s",   "|lin_velocity|", lspeed);
        ImGui::Text("%-22s  %8.4f  rad/s", "|ang_velocity|", aspeed);
        ImGui::Text("%-22s  %8.4f", "mass", b.mass);

        ImGui::Separator();
        ImGui::TextDisabled("Contacts involving this body:");
        int ncont = 0;
        for (uint32_t i = 0; i < (uint32_t)snap.contacts.size(); ++i)
        {
            const ContactSnap &c = snap.contacts[i];
            if (c.body_a != b.id && c.body_b != b.id) continue;
            ++ncont;
            const bool is_sel = (sel.contact_idx == i);
            char cl[80];
            std::snprintf(cl, sizeof(cl), "  ct#%u  depth=%.4f  λn=%.3f  %s",
                          i, c.penetration_depth, c.normal_lambda,
                          c.active ? "" : "[inactive]");
            if (ImGui::Selectable(cl, is_sel))
                sel.contact_idx = is_sel ? UINT32_MAX : i;
        }
        if (ncont == 0) ImGui::TextDisabled("  (none)");

        ImGui::Separator();
        if (!b.is_static)
        {
            if (ImGui::Button("Zero Velocity")) transport.push_command(CmdZeroVelocity{b.id});
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
                const float len = std::sqrt(imp[0]*imp[0]+imp[1]*imp[1]+imp[2]*imp[2]);
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
    //  Collider panel
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
        show_vec3("world_pos", found->world_pos);
        ImGui::Text("%-22s  %.4f", "restitution",    found->restitution);
        ImGui::Text("%-22s  %.4f", "static_friction", found->static_friction);
        ImGui::Text("%-22s  %.4f", "dyn_friction",   found->dynamic_friction);
        ImGui::Separator();
        std::visit([](auto &&sh)
        {
            using T = std::decay_t<decltype(sh)>;
            if      constexpr (std::is_same_v<T, SpherSnap>)
                ImGui::Text("radius = %.4f", sh.radius);
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
                ImGui::Text("semi_axes=%.3f  %.3f  %.3f",
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
            row("Body A",    "%u", c.body_a);
            row("Body B",    "%u", c.body_b);
            row("Collider A","%u", c.collider_a);
            row("Collider B","%u", c.collider_b);
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
            ImGui::TextColored(lambda_color((float)c.normal_lambda), "%.7f", c.normal_lambda);
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
        const float bw = ImGui::GetContentRegionAvail().x - 46.0f;
        auto bar = [&](const char *lbl, float val)
        {
            char ov[24]; std::snprintf(ov, sizeof(ov), "%.3f", val);
            ImGui::ProgressBar((val + 1.0f) * 0.5f, ImVec2(bw, 12), ov);
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
            const float pos = (float)j.current_position;
            const float lo  = (float)j.lower_limit, hi = (float)j.upper_limit;
            const bool  out = (pos < lo || pos > hi);
            ImGui::Text("pos = %.4f   [%.3f, %.3f]", pos, lo, hi);
            const float range = hi - lo;
            const float frac  = range > 1e-6f
                ? std::max(0.0f, std::min((pos - lo) / range, 1.0f)) : 0.5f;
            char ov[32]; std::snprintf(ov, sizeof(ov), "%.3f", pos);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                out ? ImVec4(1, 0.3f, 0.3f, 1) : ImVec4(0.3f, 0.9f, 0.3f, 1));
            ImGui::ProgressBar(frac, ImVec2(-1, 14), ov);
            ImGui::PopStyleColor();
            if (out) ImGui::TextColored({1, 0.3f, 0.3f, 1}, "  OUTSIDE LIMITS");
        }
        else
            ImGui::Text("pos = %.4f   (unlimited)", j.current_position);

        ImGui::Separator();
        const float lo_f = j.limited ? (float)j.lower_limit : -10.0f;
        const float hi_f = j.limited ? (float)j.upper_limit :  10.0f;
        float target = (float)j.current_position, damping = (float)j.damping;
        ImGui::SliderFloat("Target##jt",  &target,  lo_f,   hi_f);
        if (ImGui::Button("Set Target##jt"))
            transport.push_command(CmdSetJointTarget{j.id, (m3d::scalar)target});
        ImGui::SliderFloat("Damping##jd", &damping, 0.0f, 100.0f);
        if (ImGui::Button("Set Damping##jd"))
            transport.push_command(CmdSetJointDamping{j.id, (m3d::scalar)damping});
        ImGui::End();
    }

    // =========================================================================
    //  Graph panel — ImPlot, sim_time x-axis, CSV + plot-image export
    // =========================================================================

    static inline const char *track_target_name(TrackTarget t)
    {
        switch (t)
        {
        case TrackTarget::BodyLinearSpeed:          return "LinSpeed";
        case TrackTarget::BodyAngularSpeed:         return "AngSpeed";
        case TrackTarget::BodyPositionX:            return "PosX";
        case TrackTarget::BodyPositionY:            return "PosY";
        case TrackTarget::BodyPositionZ:            return "PosZ";
        case TrackTarget::ContactNormalLambda:      return "lam_n";
        case TrackTarget::ContactTangentLambda:     return "lam_t";
        case TrackTarget::ContactPenetrationDepth:  return "Depth";
        case TrackTarget::JointPosition:            return "JointPos";
        case TrackTarget::JointError:               return "JointErr";
        case TrackTarget::ConstraintLambda:         return "ConstraintLam";
        default:                                    return "Value";
        }
    }

    // Unroll the ring buffer into chronological (sim_time, value) arrays.
    static inline void linearise(const TrackedSeries     &s,
                                 std::vector<double>     &xs,
                                 std::vector<double>     &ys)
    {
        const size_t n = s.samples.size();
        xs.clear(); ys.clear();
        if (n == 0) return;
        xs.reserve(n); ys.reserve(n);

        const bool   is_full = (n >= TRACK_HISTORY_DEPTH);
        const size_t start   = is_full ? (s.head_ % n) : 0;
        for (size_t k = 0; k < n; ++k)
        {
            const auto &samp = s.samples[(start + k) % n];
            xs.push_back(samp.sim_time);   // ← sim seconds, not frame number
            ys.push_back(static_cast<double>(samp.value));
        }
    }

    // ── Axis labels — edited in the modal, applied live every frame ──────────
    // Stored as char arrays so InputText can edit them directly.
    inline char g_x_label[64] = "sim time (s)";
    inline char g_y_label[64] = "value";

    // Flag set by "Fit" button; consumed inside draw_graph_panel one frame later.
    inline bool g_fit_plot = false;

    // ── Plot export modal ─────────────────────────────────────────────────────
    // Called every frame; only visible when g_export_modal_open is true.
    inline void draw_export_modal()
    {
        if (g_export_modal_open)
            ImGui::OpenPopup("Export Plot Image");

        // AlwaysAutoResize keeps the modal tight around its content.
        if (!ImGui::BeginPopupModal("Export Plot Image", nullptr,
                                    ImGuiWindowFlags_AlwaysAutoResize))
            return;

        g_export_modal_open = false;   // consumed — popup manages its own state

        // ── Filename ──────────────────────────────────────────────────────
        ImGui::Text("Filename  (.png appended if missing):");
        ImGui::SetNextItemWidth(300);
        ImGui::InputText("##fname", g_plot_export_request.filename,
                         sizeof(g_plot_export_request.filename));

        ImGui::Separator();

        // ── Axis labels ───────────────────────────────────────────────────
        // Editing these updates the live plot immediately (same globals used
        // in SetupAxes every frame), so the screenshot captures the new labels
        // without any special handling.
        ImGui::Text("Axis labels:");
        ImGui::SetNextItemWidth(240);
        ImGui::InputText("X label##xl", g_x_label, sizeof(g_x_label));
        ImGui::SetNextItemWidth(240);
        ImGui::InputText("Y label##yl", g_y_label, sizeof(g_y_label));
        ImGui::TextDisabled("  (labels update the live plot immediately)");

        ImGui::Separator();

        // ── Resolution ────────────────────────────────────────────────────
        const float pw = g_last_plot_size.x;
        const float ph = g_last_plot_size.y;
        ImGui::Text("Plot region on screen: %.0f × %.0f px", pw, ph);

        static int scale_idx = 0;
        const char *scale_labels[] = { "1×  (screen res)", "2×  (upscaled)", "4×  (upscaled)" };
        const int   scale_values[] = { 1, 2, 4 };
        ImGui::SetNextItemWidth(200);
        ImGui::Combo("Output scale", &scale_idx, scale_labels, 3);
        const int upscale = scale_values[scale_idx];

        ImGui::TextColored({0.6f, 0.9f, 0.6f, 1},
                           "Output size: %.0f × %.0f px",
                           pw * upscale, ph * upscale);

        ImGui::Separator();
        ImGui::TextDisabled("The plot area (axes + lines, including axis labels)");
        ImGui::TextDisabled("will be cropped from the current frame and saved.");

        ImGui::Separator();

        // ── Buttons ───────────────────────────────────────────────────────
        if (ImGui::Button("Export", {120, 0}))
        {
            g_plot_export_request.pending  = true;
            g_plot_export_request.pos      = g_last_plot_pos;
            g_plot_export_request.size     = g_last_plot_size;
            g_plot_export_request.upscale  = upscale;
            // filename already written into g_plot_export_request.filename
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", {120, 0}))
            ImGui::CloseCurrentPopup();

        ImGui::EndPopup();
    }

    template<typename T>
    inline void draw_graph_panel(DebugChannel<T>    &channel,
                                 InProcessTransport &transport)
    {
        ImGui::SetNextWindowSize({520, 460}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({900, 480}, ImGuiCond_FirstUseEver);
        ImGui::Begin("Graphs");

        // ── Track-quantity selector ────────────────────────────────────────
        static uint32_t track_id = 0;
        ImGui::SetNextItemWidth(80);
        ImGui::InputScalar("Body ID##tid", ImGuiDataType_U32, &track_id);
        ImGui::SameLine();
        ImGui::TextDisabled("Add:");
        ImGui::SameLine();

        if (ImGui::SmallButton("LinSpd")) transport.push_command(CmdTrackQuantity{TrackTarget::BodyLinearSpeed,  track_id, true});
        ImGui::SameLine();
        if (ImGui::SmallButton("AngSpd")) transport.push_command(CmdTrackQuantity{TrackTarget::BodyAngularSpeed, track_id, true});
        ImGui::SameLine();
        if (ImGui::SmallButton("PosX"))   transport.push_command(CmdTrackQuantity{TrackTarget::BodyPositionX, track_id, true});
        ImGui::SameLine();
        if (ImGui::SmallButton("PosY"))   transport.push_command(CmdTrackQuantity{TrackTarget::BodyPositionY, track_id, true});
        ImGui::SameLine();
        if (ImGui::SmallButton("PosZ"))   transport.push_command(CmdTrackQuantity{TrackTarget::BodyPositionZ, track_id, true});

        ImGui::Separator();

        // Take a VALUE COPY of the series vector, not a reference.
        // tracked_series() now returns by value precisely to prevent a
        // dangling-reference segfault if the vector reallocates mid-frame.
        const auto all_series = channel.tracked_series();

        // ── Export toolbar ─────────────────────────────────────────────────
        if (!all_series.empty())
        {
            // CSV clipboard
            if (ImGui::Button("Copy CSV"))
            {
                std::string csv = "frame,sim_time_s";
                for (const auto &s : all_series)
                {
                    csv += ',';
                    csv += track_target_name(s.target);
                    csv += '_';
                    csv += std::to_string(s.id);
                }
                csv += '\n';

                // Linearise all series first
                std::vector<std::vector<double>> all_xs(all_series.size());
                std::vector<std::vector<double>> all_ys(all_series.size());
                size_t max_n = 0;
                for (size_t si = 0; si < all_series.size(); ++si)
                {
                    linearise(all_series[si], all_xs[si], all_ys[si]);
                    max_n = std::max(max_n, all_xs[si].size());
                }

                for (size_t k = 0; k < max_n; ++k)
                {
                    // Frame + sim_time from the longest series
                    const uint64_t fr = (k < all_series[0].samples.size())
                        ? all_series[0].samples[k].frame : k;
                    const double   t  = (k < all_xs[0].size()) ? all_xs[0][k] : 0.0;
                    char row[64];
                    std::snprintf(row, sizeof(row), "%llu,%.6f",
                                  (unsigned long long)fr, t);
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
                    }
                    csv += '\n';
                }
                ImGui::SetClipboardText(csv.c_str());
            }

            ImGui::SameLine();

            // Plot PNG export — opens the modal
            if (ImGui::Button("Export Plot PNG"))
                g_export_modal_open = true;

            // Toast: show result for ~4 seconds
            if (!g_export_status.empty())
            {
                const double elapsed = GetTime() - g_export_status_time;
                if (elapsed < 4.0)
                {
                    const float alpha = (elapsed < 3.0) ? 1.0f
                                      : 1.0f - (float)((elapsed - 3.0) / 1.0);
                    ImGui::SameLine(0, 16);
                    ImGui::TextColored({0.4f, 1.0f, 0.4f, alpha},
                                       "%s", g_export_status.c_str());
                }
            }

            ImGui::Separator();
        }
        else
        {
            ImGui::TextDisabled("No series tracked yet.  Add one above.");
            ImGui::Separator();
        }

        // ── Active series list (stop buttons + last-value readout) ─────────
        {
            uint32_t    stop_id     = UINT32_MAX;
            TrackTarget stop_target = TrackTarget::BodyLinearSpeed;

            for (const auto &s : all_series)
            {
                char lbl[80];
                std::snprintf(lbl, sizeof(lbl), "Stop %s[%u]",
                              track_target_name(s.target), s.id);
                if (ImGui::SmallButton(lbl))
                { stop_id = s.id; stop_target = s.target; }
                ImGui::SameLine(0, 10);
                ImGui::TextDisabled("= %.5f  (t=%.3f s)",
                                    s.last_value(), s.last_sim_time());
            }
            if (stop_id != UINT32_MAX)
                transport.push_command(CmdTrackQuantity{stop_target, stop_id, false});
        }

        if (!all_series.empty()) ImGui::Separator();

        // ── ImPlot ────────────────────────────────────────────────────────
        if (!all_series.empty())
        {
            // Linearise all series and compute full-data axis ranges.
            // These are used only when the user requests a fit, NOT every frame —
            // that was the bug: setting limits with ImGuiCond_Always each frame
            // overrode any scroll/zoom the user had applied.
            std::vector<std::vector<double>> all_xs(all_series.size());
            std::vector<std::vector<double>> all_ys(all_series.size());
            double x_min =  1e18, x_max = -1e18;
            double y_min =  1e18, y_max = -1e18;

            for (size_t si = 0; si < all_series.size(); ++si)
            {
                linearise(all_series[si], all_xs[si], all_ys[si]);
                for (double v : all_xs[si]) { x_min = std::min(x_min, v); x_max = std::max(x_max, v); }
                for (double v : all_ys[si]) { y_min = std::min(y_min, v); y_max = std::max(y_max, v); }
            }

            if (x_max <= x_min) x_max = x_min + 1.0;
            const double ym = (y_max - y_min) * 0.08 + 1e-6;
            y_min -= ym; y_max += ym;

            // ── Fit / zoom controls ───────────────────────────────────────
            // Fit button: snap view to data once.
            // Auto-fit checkbox: re-fit every frame (good while data is growing;
            // turn off when you want to inspect a specific time window).
            if (ImGui::SmallButton("Fit")) g_fit_plot = true;
            ImGui::SameLine(0, 8);
            ImGui::Checkbox("Auto-fit##af", &g_auto_fit);
            ImGui::SameLine(0, 16);
            ImGui::TextDisabled("scroll=zoom  drag=pan  dbl-click=reset");

            // When auto-fit is on OR the Fit button was just pressed, push
            // the full-data limits into ImPlot for the coming frame.
            // ImGuiCond_Always forces the update even after user interaction.
            const bool do_fit = g_auto_fit || g_fit_plot;
            if (do_fit)
            {
                ImPlot::SetNextAxesLimits(x_min, x_max, y_min, y_max,
                                          ImGuiCond_Always);
            }
            g_fit_plot = false; // consume the one-shot flag regardless

            // ── Capture the FULL widget rect (including axis decorations) ──
            // Must be done BEFORE BeginPlot so the cursor hasn't been consumed.
            // We save this as the export crop rect — it covers tick labels and
            // axis names that live outside the inner data canvas.
            static constexpr float PLOT_H = 220.0f; // matches BeginPlot height
            g_last_plot_pos  = ImGui::GetCursorScreenPos();
            g_last_plot_size = ImVec2(ImGui::GetContentRegionAvail().x, PLOT_H);

            if (ImPlot::BeginPlot("##tracked", ImVec2(-1, PLOT_H), ImPlotFlags_NoTitle))
            {
                // ImPlotAxisFlags_None: ImPlot owns zoom state between frames.
                // Limits only change via SetNextAxesLimits (above) or user input.
                ImPlot::SetupAxes(g_x_label, g_y_label,
                                  ImPlotAxisFlags_None,
                                  ImPlotAxisFlags_None);

                for (size_t si = 0; si < all_series.size(); ++si)
                {
                    if (all_xs[si].empty()) continue;
                    char label[64];
                    std::snprintf(label, sizeof(label), "%s[%u]",
                                  track_target_name(all_series[si].target),
                                  all_series[si].id);
                    ImPlot::PlotLine(label,
                                     all_xs[si].data(), all_ys[si].data(),
                                     (int)all_xs[si].size());
                }

                ImPlot::EndPlot();
            }
        }

        // ── Export modal (drawn every frame; visible only when open) ───────
        draw_export_modal();

        ImGui::End();
    }

    // =========================================================================
    //  Master draw
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
        draw_graph_panel         (channel, transport);
        draw_help_panel          ();
    }

} // namespace visr::ui