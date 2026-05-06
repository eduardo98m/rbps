#pragma once
#include <imgui.h>
#include "State.hpp"
#include "ShapeFactory.hpp"
#include "PipelineRun.hpp"
#include <cmath>
#include <cstdio>

namespace cdbg
{

    inline const char *gjk_status_name(rbc::GJK::Status s)
    {
        switch (s)
        {
        case rbc::GJK::Valid:  return "Valid (separated)";
        case rbc::GJK::Inside: return "Inside (overlap)";
        case rbc::GJK::Failed: return "Failed";
        }
        return "?";
    }

    inline const char *epa_status_name(rbc::EPA::Status s)
    {
        switch (s)
        {
        case rbc::EPA::Valid:         return "Valid";
        case rbc::EPA::Failed:        return "Failed";
        case rbc::EPA::OutOfFaces:    return "OutOfFaces";
        case rbc::EPA::OutOfVertices: return "OutOfVertices";
        case rbc::EPA::NonLinear:     return "NonLinear";
        }
        return "?";
    }

    // Editable double[3] via DragFloat3 (ImGui has no DragDouble3 in stable API).
    inline bool drag_vec3(const char *label, m3d::vec3 &v, float speed, float min_v, float max_v)
    {
        float buf[3] = {(float)v.x, (float)v.y, (float)v.z};
        const bool changed = ImGui::DragFloat3(label, buf, speed, min_v, max_v, "%.3f");
        if (changed)
        {
            v.x = buf[0];
            v.y = buf[1];
            v.z = buf[2];
        }
        return changed;
    }

    inline bool drag_scalar(const char *label, m3d::scalar &s, float speed,
                            float min_v, float max_v, const char *fmt = "%.3f")
    {
        float v = (float)s;
        const bool changed = ImGui::DragFloat(label, &v, speed, min_v, max_v, fmt);
        if (changed) s = v;
        return changed;
    }

    inline void draw_shape_params(const char *id_suffix, ShapeParams &p)
    {
        static const char *kind_names[Kind_Count] = {
            "Sphere", "Box", "Capsule", "Cylinder", "Cone", "Ellipsoid", "ConvexHull"};

        char combo_id[32];
        std::snprintf(combo_id, sizeof(combo_id), "Kind##%s", id_suffix);
        ImGui::Combo(combo_id, &p.kind, kind_names, Kind_Count);

        char id_buf[64];
        switch (p.kind)
        {
        case Kind_Sphere:
            std::snprintf(id_buf, sizeof(id_buf), "radius##%s", id_suffix);
            drag_scalar(id_buf, p.sphere_radius, 0.01f, 0.05f, 5.0f);
            break;
        case Kind_Box:
            std::snprintf(id_buf, sizeof(id_buf), "half-extents##%s", id_suffix);
            drag_vec3(id_buf, p.box_half_ext, 0.01f, 0.05f, 5.0f);
            break;
        case Kind_Capsule:
            std::snprintf(id_buf, sizeof(id_buf), "radius##%s", id_suffix);
            drag_scalar(id_buf, p.capsule_radius, 0.01f, 0.05f, 3.0f);
            std::snprintf(id_buf, sizeof(id_buf), "half-height##%s", id_suffix);
            drag_scalar(id_buf, p.capsule_half_h, 0.01f, 0.0f, 3.0f);
            break;
        case Kind_Cylinder:
            std::snprintf(id_buf, sizeof(id_buf), "base radius##%s", id_suffix);
            drag_scalar(id_buf, p.cyl_radius, 0.01f, 0.05f, 3.0f);
            std::snprintf(id_buf, sizeof(id_buf), "half-height##%s", id_suffix);
            drag_scalar(id_buf, p.cyl_half_h, 0.01f, 0.05f, 3.0f);
            break;
        case Kind_Cone:
            std::snprintf(id_buf, sizeof(id_buf), "base radius##%s", id_suffix);
            drag_scalar(id_buf, p.cone_radius, 0.01f, 0.05f, 3.0f);
            std::snprintf(id_buf, sizeof(id_buf), "half-height##%s", id_suffix);
            drag_scalar(id_buf, p.cone_half_h, 0.01f, 0.05f, 3.0f);
            break;
        case Kind_Ellipsoid:
            std::snprintf(id_buf, sizeof(id_buf), "semi-axes##%s", id_suffix);
            drag_vec3(id_buf, p.ellipsoid_axes, 0.01f, 0.05f, 3.0f);
            break;
        case Kind_ConvexHull:
        {
            static const char *hull_names[Hull_Count] = {
                "Tetrahedron", "Octahedron", "Tri-prism", "Hex-prism"};
            std::snprintf(id_buf, sizeof(id_buf), "preset##%s", id_suffix);
            ImGui::Combo(id_buf, &p.hull_preset, hull_names, Hull_Count);
            ImGui::TextDisabled("(hull data is fixed per preset)");
            break;
        }
        }
    }

    inline void draw_pose_controls(const char *id_suffix, PoseControl &pose)
    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "position##%s", id_suffix);
        drag_vec3(buf, pose.position, 0.01f, -10.0f, 10.0f);

        std::snprintf(buf, sizeof(buf), "rpy (deg)##%s", id_suffix);
        drag_vec3(buf, pose.rpy_deg, 0.5f, -180.0f, 180.0f);

        std::snprintf(buf, sizeof(buf), "Reset##%s", id_suffix);
        if (ImGui::SmallButton(buf))
        {
            pose.position = m3d::vec3{0, 0, 0};
            pose.rpy_deg  = m3d::vec3{0, 0, 0};
        }
    }

    inline void draw_inputs_panel(DebuggerState &s)
    {
        ImGui::SetNextWindowSize({360, 540}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({10, 10}, ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Inputs"))
        {
            ImGui::End();
            return;
        }

        ImGui::SeparatorText("Shape A");
        draw_shape_params("A", s.params_a);
        draw_pose_controls("A", s.pose_a);

        ImGui::SeparatorText("Shape B");
        draw_shape_params("B", s.params_b);
        draw_pose_controls("B", s.pose_b);

        ImGui::SeparatorText("Pipeline");
        ImGui::Checkbox("Live recompute", &s.live_recompute);
        if (!s.live_recompute)
        {
            ImGui::SameLine(0, 12);
            if (ImGui::Button("Recompute"))
                s.recompute_now = true;
        }
        if (ImGui::Button("Swap A <-> B"))
        {
            std::swap(s.params_a, s.params_b);
            std::swap(s.pose_a,   s.pose_b);
        }
        ImGui::SameLine(0, 8);
        if (ImGui::Button("Reset both poses"))
        {
            s.pose_a = PoseControl{};
            s.pose_b = PoseControl{};
        }

        ImGui::End();
    }

    inline void draw_viz_panel(VizFlags &v)
    {
        ImGui::SetNextWindowSize({320, 380}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({380, 10}, ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Visualization"))
        {
            ImGui::End();
            return;
        }

        ImGui::SeparatorText("World layers");
        ImGui::Checkbox("Shape A wireframe",      &v.shape_a);
        ImGui::Checkbox("Shape B wireframe",      &v.shape_b);
        ImGui::Checkbox("World grid",             &v.world_grid);
        ImGui::Checkbox("EPA normal arrow",       &v.epa_normal);
        ImGui::Checkbox("EPA contact point",      &v.epa_contact_point);
        ImGui::Checkbox("Reference face (A)",     &v.ref_face);
        ImGui::Checkbox("Incident face (B)",      &v.inc_face);
        ImGui::Checkbox("Clipped polygon",        &v.clipped_polygon);
        ImGui::Checkbox("Final manifold points",  &v.manifold_points);

        ImGui::SeparatorText("Minkowski-difference space (offset)");
        ImGui::Checkbox("GJK simplex",            &v.gjk_simplex);
        ImGui::Checkbox("EPA polytope",           &v.epa_polytope);
        ImGui::Checkbox("MD-space axes",          &v.md_space_axes);
        drag_vec3("MD offset", v.md_space_offset, 0.05f, -20.0f, 20.0f);

        ImGui::End();
    }

    inline void draw_readout_panel(const PipelineResult &r)
    {
        ImGui::SetNextWindowSize({420, 360}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({710, 10}, ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Pipeline state"))
        {
            ImGui::End();
            return;
        }

        ImGui::SeparatorText("GJK");
        const ImVec4 c_inside = {0.4f, 1.0f, 0.5f, 1.0f};
        const ImVec4 c_other  = {1.0f, 0.7f, 0.3f, 1.0f};
        ImGui::TextColored(r.gjk_status == rbc::GJK::Inside ? c_inside : c_other,
                           "status: %s", gjk_status_name(r.gjk_status));
        ImGui::Text("simplex rank: %d", r.gjk_simplex_rank);
        if (r.gjk_status == rbc::GJK::Valid)
            ImGui::Text("closest distance: %.6f", r.gjk_distance);

        ImGui::SeparatorText("EPA");
        ImGui::TextColored(r.epa_status == rbc::EPA::Valid ? c_inside : c_other,
                           "status: %s", epa_status_name(r.epa_status));
        if (r.epa_converged)
        {
            ImGui::Text("normal:  (%.4f, %.4f, %.4f)",
                        r.epa_normal.x, r.epa_normal.y, r.epa_normal.z);
            const float n_mag = (float)std::sqrt(r.epa_normal.x * r.epa_normal.x +
                                                 r.epa_normal.y * r.epa_normal.y +
                                                 r.epa_normal.z * r.epa_normal.z);
            const bool n_ok = std::fabs(n_mag - 1.0f) < 0.01f;
            ImGui::TextColored(n_ok ? c_inside : ImVec4{1, 0.3f, 0.3f, 1},
                               "|normal| = %.5f %s", n_mag, n_ok ? "OK" : "WARNING");
            ImGui::Text("depth:   %.6f", r.epa_depth);
            ImGui::Text("contact: (%.4f, %.4f, %.4f)",
                        r.epa_contact_point.x,
                        r.epa_contact_point.y,
                        r.epa_contact_point.z);
            ImGui::Text("polytope faces: %zu", r.epa_faces.size());
        }

        ImGui::SeparatorText("Manifold generation");
        ImGui::Text("ref face corners: %d", r.ref_face_n);
        ImGui::Text("inc face corners: %d", r.inc_face_n);
        ImGui::Text("post-clip polygon: %zu", r.post_clip_polygon.size());
        ImGui::Text("kept (post depth-test): %zu", r.kept_points.size());

        ImGui::SeparatorText("Final manifold");
        ImGui::Text("num_points: %d", r.manifold.num_points);
        if (r.manifold.num_points > 0 &&
            ImGui::BeginTable("mfd", 5,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingFixedFit))
        {
            ImGui::TableSetupColumn("#",     ImGuiTableColumnFlags_WidthFixed, 24);
            ImGui::TableSetupColumn("x",     ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("y",     ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("z",     ImGuiTableColumnFlags_WidthFixed, 70);
            ImGui::TableSetupColumn("depth", ImGuiTableColumnFlags_WidthFixed, 80);
            ImGui::TableHeadersRow();
            for (int i = 0; i < r.manifold.num_points; ++i)
            {
                const auto &p = r.manifold.points[i];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("%d", i);
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", p.position.x);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f", p.position.y);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%.4f", p.position.z);
                ImGui::TableSetColumnIndex(4); ImGui::Text("%.5f", p.penetration_depth);
            }
            ImGui::EndTable();
        }

        ImGui::End();
    }

} // namespace cdbg
