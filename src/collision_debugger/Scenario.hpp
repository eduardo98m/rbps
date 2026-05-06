#pragma once
#include "State.hpp"
#include "ShapeFactory.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>

namespace cdbg
{

    // ── Conversions: kind / hull-preset ↔ token ──────────────────────────

    inline const char *kind_token(int k)
    {
        switch (k)
        {
        case Kind_Sphere:     return "Sphere";
        case Kind_Box:        return "Box";
        case Kind_Capsule:    return "Capsule";
        case Kind_Cylinder:   return "Cylinder";
        case Kind_Cone:       return "Cone";
        case Kind_Ellipsoid:  return "Ellipsoid";
        case Kind_ConvexHull: return "ConvexHull";
        }
        return "Box";
    }

    inline int parse_kind(const std::string &s)
    {
        if (s == "Sphere")     return Kind_Sphere;
        if (s == "Box")        return Kind_Box;
        if (s == "Capsule")    return Kind_Capsule;
        if (s == "Cylinder")   return Kind_Cylinder;
        if (s == "Cone")       return Kind_Cone;
        if (s == "Ellipsoid")  return Kind_Ellipsoid;
        if (s == "ConvexHull") return Kind_ConvexHull;
        return Kind_Box;
    }

    inline const char *hull_token(int p)
    {
        switch (p)
        {
        case Hull_Tetrahedron: return "Tetrahedron";
        case Hull_Octahedron:  return "Octahedron";
        case Hull_TriPrism:    return "TriPrism";
        case Hull_HexPrism:    return "HexPrism";
        }
        return "HexPrism";
    }

    inline int parse_hull(const std::string &s)
    {
        if (s == "Tetrahedron") return Hull_Tetrahedron;
        if (s == "Octahedron")  return Hull_Octahedron;
        if (s == "TriPrism")    return Hull_TriPrism;
        if (s == "HexPrism")    return Hull_HexPrism;
        return Hull_HexPrism;
    }

    // ── Save ─────────────────────────────────────────────────────────────

    inline bool save_scenario(const char *path, const DebuggerState &s)
    {
        FILE *fp = std::fopen(path, "wb");
        if (!fp) return false;

        std::fprintf(fp, "# rbps collision_debugger scenario\n");
        std::fprintf(fp, "version = 1\n");

        // Shape A
        std::fprintf(fp, "shape_a = %s\n", kind_token(s.params_a.kind));
        std::fprintf(fp, "sphere_a = %.10f\n", s.params_a.sphere_radius);
        std::fprintf(fp, "box_a = %.10f %.10f %.10f\n",
                     s.params_a.box_half_ext.x, s.params_a.box_half_ext.y, s.params_a.box_half_ext.z);
        std::fprintf(fp, "capsule_a = %.10f %.10f\n",
                     s.params_a.capsule_radius, s.params_a.capsule_half_h);
        std::fprintf(fp, "cyl_a = %.10f %.10f\n",
                     s.params_a.cyl_radius, s.params_a.cyl_half_h);
        std::fprintf(fp, "cone_a = %.10f %.10f\n",
                     s.params_a.cone_radius, s.params_a.cone_half_h);
        std::fprintf(fp, "ellipsoid_a = %.10f %.10f %.10f\n",
                     s.params_a.ellipsoid_axes.x, s.params_a.ellipsoid_axes.y, s.params_a.ellipsoid_axes.z);
        std::fprintf(fp, "hull_a = %s\n", hull_token(s.params_a.hull_preset));

        // Shape B
        std::fprintf(fp, "shape_b = %s\n", kind_token(s.params_b.kind));
        std::fprintf(fp, "sphere_b = %.10f\n", s.params_b.sphere_radius);
        std::fprintf(fp, "box_b = %.10f %.10f %.10f\n",
                     s.params_b.box_half_ext.x, s.params_b.box_half_ext.y, s.params_b.box_half_ext.z);
        std::fprintf(fp, "capsule_b = %.10f %.10f\n",
                     s.params_b.capsule_radius, s.params_b.capsule_half_h);
        std::fprintf(fp, "cyl_b = %.10f %.10f\n",
                     s.params_b.cyl_radius, s.params_b.cyl_half_h);
        std::fprintf(fp, "cone_b = %.10f %.10f\n",
                     s.params_b.cone_radius, s.params_b.cone_half_h);
        std::fprintf(fp, "ellipsoid_b = %.10f %.10f %.10f\n",
                     s.params_b.ellipsoid_axes.x, s.params_b.ellipsoid_axes.y, s.params_b.ellipsoid_axes.z);
        std::fprintf(fp, "hull_b = %s\n", hull_token(s.params_b.hull_preset));

        // Poses
        std::fprintf(fp, "pose_a_pos = %.10f %.10f %.10f\n",
                     s.pose_a.position.x, s.pose_a.position.y, s.pose_a.position.z);
        std::fprintf(fp, "pose_a_rpy = %.10f %.10f %.10f\n",
                     s.pose_a.rpy_deg.x, s.pose_a.rpy_deg.y, s.pose_a.rpy_deg.z);
        std::fprintf(fp, "pose_b_pos = %.10f %.10f %.10f\n",
                     s.pose_b.position.x, s.pose_b.position.y, s.pose_b.position.z);
        std::fprintf(fp, "pose_b_rpy = %.10f %.10f %.10f\n",
                     s.pose_b.rpy_deg.x, s.pose_b.rpy_deg.y, s.pose_b.rpy_deg.z);

        std::fflush(fp);
        std::fclose(fp);
        return true;
    }

    // ── Load ─────────────────────────────────────────────────────────────

    namespace scenario_detail
    {
        inline std::string trim(const std::string &s)
        {
            size_t a = 0, b = s.size();
            while (a < b && std::isspace((unsigned char)s[a])) ++a;
            while (b > a && std::isspace((unsigned char)s[b - 1])) --b;
            return s.substr(a, b - a);
        }

        inline void parse_vec3(const std::string &v, m3d::vec3 &out)
        {
            std::istringstream iss(v);
            double x = 0, y = 0, z = 0;
            iss >> x >> y >> z;
            out.x = x; out.y = y; out.z = z;
        }

        inline void parse_pair(const std::string &v, m3d::scalar &a, m3d::scalar &b)
        {
            std::istringstream iss(v);
            double x = 0, y = 0;
            iss >> x >> y;
            a = x; b = y;
        }

        inline m3d::scalar parse_scalar(const std::string &v)
        {
            return std::strtod(v.c_str(), nullptr);
        }
    }

    inline bool load_scenario(const char *path, DebuggerState &s)
    {
        FILE *fp = std::fopen(path, "rb");
        if (!fp) return false;

        char buf[256];
        while (std::fgets(buf, sizeof(buf), fp))
        {
            std::string line = scenario_detail::trim(buf);
            if (line.empty() || line[0] == '#') continue;

            const auto eq = line.find('=');
            if (eq == std::string::npos) continue;
            const std::string key = scenario_detail::trim(line.substr(0, eq));
            const std::string val = scenario_detail::trim(line.substr(eq + 1));

            if      (key == "shape_a")     s.params_a.kind        = parse_kind(val);
            else if (key == "sphere_a")    s.params_a.sphere_radius = scenario_detail::parse_scalar(val);
            else if (key == "box_a")       scenario_detail::parse_vec3(val, s.params_a.box_half_ext);
            else if (key == "capsule_a")   scenario_detail::parse_pair(val, s.params_a.capsule_radius, s.params_a.capsule_half_h);
            else if (key == "cyl_a")       scenario_detail::parse_pair(val, s.params_a.cyl_radius, s.params_a.cyl_half_h);
            else if (key == "cone_a")      scenario_detail::parse_pair(val, s.params_a.cone_radius, s.params_a.cone_half_h);
            else if (key == "ellipsoid_a") scenario_detail::parse_vec3(val, s.params_a.ellipsoid_axes);
            else if (key == "hull_a")      s.params_a.hull_preset = parse_hull(val);

            else if (key == "shape_b")     s.params_b.kind        = parse_kind(val);
            else if (key == "sphere_b")    s.params_b.sphere_radius = scenario_detail::parse_scalar(val);
            else if (key == "box_b")       scenario_detail::parse_vec3(val, s.params_b.box_half_ext);
            else if (key == "capsule_b")   scenario_detail::parse_pair(val, s.params_b.capsule_radius, s.params_b.capsule_half_h);
            else if (key == "cyl_b")       scenario_detail::parse_pair(val, s.params_b.cyl_radius, s.params_b.cyl_half_h);
            else if (key == "cone_b")      scenario_detail::parse_pair(val, s.params_b.cone_radius, s.params_b.cone_half_h);
            else if (key == "ellipsoid_b") scenario_detail::parse_vec3(val, s.params_b.ellipsoid_axes);
            else if (key == "hull_b")      s.params_b.hull_preset = parse_hull(val);

            else if (key == "pose_a_pos")  scenario_detail::parse_vec3(val, s.pose_a.position);
            else if (key == "pose_a_rpy")  scenario_detail::parse_vec3(val, s.pose_a.rpy_deg);
            else if (key == "pose_b_pos")  scenario_detail::parse_vec3(val, s.pose_b.position);
            else if (key == "pose_b_rpy")  scenario_detail::parse_vec3(val, s.pose_b.rpy_deg);
        }

        std::fclose(fp);
        return true;
    }

    // ── Filesystem helpers (no <filesystem> dep — keeps build light) ─────

    inline bool file_exists(const char *path)
    {
        FILE *fp = std::fopen(path, "rb");
        if (!fp) return false;
        std::fclose(fp);
        return true;
    }

    inline bool ensure_dir(const char *path)
    {
        // Use system mkdir -p for portability across CWD assumptions.
        // path is internal, never user-controlled (always a fixed string).
        std::string cmd = "mkdir -p '";
        cmd += path;
        cmd += "'";
        return std::system(cmd.c_str()) == 0;
    }

    inline bool delete_file(const char *path)
    {
        return std::remove(path) == 0;
    }

    inline bool write_marker(const char *path)
    {
        FILE *fp = std::fopen(path, "wb");
        if (!fp) return false;
        std::fclose(fp);
        return true;
    }

} // namespace cdbg
