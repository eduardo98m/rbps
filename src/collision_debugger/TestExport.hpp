#pragma once
#include "State.hpp"
#include "ShapeFactory.hpp"
#include "PipelineRun.hpp"
#include "Scenario.hpp"     // for kind_token / hull_token
#include <cstdio>
#include <string>
#include <sstream>

namespace cdbg
{

    enum ExpectedOutcome : int
    {
        Expect_Success           = 0, // GJK Inside, EPA Valid, manifold count match
        Expect_GJKSeparates      = 1, // GJK Valid (no overlap)
        Expect_EPAFails          = 2, // GJK Inside, EPA != Valid
        Expect_CrashRegression   = 3, // Just call gjk_epa_manifold; pass if returns
    };

    inline const char *expected_label(int e)
    {
        switch (e)
        {
        case Expect_Success:         return "Success (manifold built)";
        case Expect_GJKSeparates:    return "GJK separates (no overlap)";
        case Expect_EPAFails:        return "EPA fails";
        case Expect_CrashRegression: return "Crash regression (must not crash)";
        }
        return "?";
    }

    namespace export_detail
    {
        // Sanitize an arbitrary user-typed name into an identifier suitable
        // for both filenames and TEST(...) macros: alnum + underscore only.
        inline std::string sanitize(const std::string &name)
        {
            std::string out;
            out.reserve(name.size());
            for (char c : name)
            {
                if ((c >= 'a' && c <= 'z') ||
                    (c >= 'A' && c <= 'Z') ||
                    (c >= '0' && c <= '9') ||
                    c == '_')
                    out.push_back(c);
                else if (c == ' ' || c == '-')
                    out.push_back('_');
            }
            if (out.empty()) out = "unnamed";
            // Identifier cannot begin with a digit
            if (out[0] >= '0' && out[0] <= '9') out.insert(out.begin(), '_');
            return out;
        }

        // Per-shape inline construction. For ConvexHull we emit the vertex
        // and face arrays as static const at file scope (returned via the
        // hull_arrays stream) and a constructor expression that wraps the
        // resulting ConvexHullData* in an rbc::Shape.
        //
        // `id` is "a" or "b" — used to disambiguate hull arrays per scenario.
        // Returns the C++ expression that constructs the rbc::Shape.
        // Appends any required hull-data definitions to `hull_arrays`.
        // Appends any cleanup statements to `cleanup`.
        inline std::string emit_shape_ctor(const ShapeParams &p,
                                           const char *id,
                                           std::ostringstream &hull_arrays,
                                           std::ostringstream &hull_setup,
                                           std::ostringstream &cleanup)
        {
            std::ostringstream e;
            e.precision(10);
            switch (p.kind)
            {
            case Kind_Sphere:
                e << "rbc::Shape(rbc::Sphere(" << p.sphere_radius << "))";
                break;
            case Kind_Box:
                e << "rbc::Shape(rbc::Box(m3d::vec3("
                  << p.box_half_ext.x << ", " << p.box_half_ext.y << ", " << p.box_half_ext.z << ")))";
                break;
            case Kind_Capsule:
                e << "rbc::Shape(rbc::Capsule("
                  << p.capsule_half_h << ", " << p.capsule_radius << "))";
                break;
            case Kind_Cylinder:
                e << "rbc::Shape(rbc::Cylinder("
                  << p.cyl_half_h << ", " << p.cyl_radius << "))";
                break;
            case Kind_Cone:
                e << "rbc::Shape(rbc::Cone("
                  << p.cone_half_h << ", " << p.cone_radius << "))";
                break;
            case Kind_Ellipsoid:
                e << "rbc::Shape(rbc::Ellipsoid(m3d::vec3("
                  << p.ellipsoid_axes.x << ", " << p.ellipsoid_axes.y << ", " << p.ellipsoid_axes.z << ")))";
                break;
            case Kind_ConvexHull:
            {
                rbc::ConvexHullData *data = get_hull_data(p.hull_preset);
                hull_arrays.precision(10);
                hull_arrays << "    static const m3d::vec3 hull_" << id << "_verts["
                            << data->vert_count << "] = {\n";
                for (uint32_t i = 0; i < data->vert_count; ++i)
                {
                    hull_arrays << "        m3d::vec3("
                                << data->vertices[i].x << ", "
                                << data->vertices[i].y << ", "
                                << data->vertices[i].z << "),\n";
                }
                hull_arrays << "    };\n";
                hull_arrays << "    static const uint32_t hull_" << id << "_faces["
                            << (data->face_count * 3) << "] = {\n";
                for (uint32_t i = 0; i < data->face_count; ++i)
                {
                    hull_arrays << "        "
                                << data->face_indices[i * 3 + 0] << ", "
                                << data->face_indices[i * 3 + 1] << ", "
                                << data->face_indices[i * 3 + 2] << ",\n";
                }
                hull_arrays << "    };\n";
                hull_setup << "    rbc::ConvexHullData *hd_" << id
                           << " = rbc::convex_hull_data_create(hull_" << id
                           << "_verts, " << data->vert_count
                           << ", hull_" << id << "_faces, "
                           << data->face_count << ");\n";
                cleanup << "    rbc::convex_hull_data_destroy(hd_" << id << ");\n";
                e << "rbc::Shape(rbc::ConvexHull(hd_" << id << "))";
                break;
            }
            }
            return e.str();
        }

        inline std::string emit_tf(const PoseControl &p)
        {
            const m3d::tf tf = pose_to_tf(p);
            std::ostringstream e;
            e.precision(10);
            e << "{ m3d::vec3(" << tf.pos.x << ", " << tf.pos.y << ", " << tf.pos.z << "), "
              << "m3d::quat(" << tf.rot.w << ", " << tf.rot.x << ", " << tf.rot.y << ", " << tf.rot.z << ") }";
            return e.str();
        }
    } // namespace export_detail

    // Build the C++ contents of a regression test. `name` is sanitized.
    inline std::string format_test_cpp(const DebuggerState &state,
                                       const PipelineResult &result,
                                       int expected,
                                       const std::string &name)
    {
        const std::string id = export_detail::sanitize(name);

        std::ostringstream hull_arrays;
        std::ostringstream hull_setup;
        std::ostringstream cleanup;
        const std::string ctor_a = export_detail::emit_shape_ctor(
            state.params_a, "a", hull_arrays, hull_setup, cleanup);
        const std::string ctor_b = export_detail::emit_shape_ctor(
            state.params_b, "b", hull_arrays, hull_setup, cleanup);
        const std::string tf_a = export_detail::emit_tf(state.pose_a);
        const std::string tf_b = export_detail::emit_tf(state.pose_b);

        std::ostringstream out;
        out.precision(10);

        out << "// AUTO-GENERATED by collision_debugger.\n";
        out << "// Promote out of tests/rbc/regression/ to commit.\n";
        out << "// Expected outcome: " << expected_label(expected) << "\n";
        out << "//\n";
        out << "// Shape A: " << kind_token(state.params_a.kind);
        if (state.params_a.kind == Kind_ConvexHull)
            out << " (" << hull_token(state.params_a.hull_preset) << ")";
        out << "\n";
        out << "// Shape B: " << kind_token(state.params_b.kind);
        if (state.params_b.kind == Kind_ConvexHull)
            out << " (" << hull_token(state.params_b.hull_preset) << ")";
        out << "\n\n";

        out << "#include \"tests/test_helper.hpp\"\n";
        out << "#include \"rbc/gjk/GJK.hpp\"\n";
        out << "#include \"rbc/gjk/EPA.hpp\"\n";
        out << "#include \"rbc/gjk/MinkowskiDiff.hpp\"\n";
        out << "#include \"rbc/gjk/ContactManifoldGenerator.hpp\"\n";
        out << "#include \"rbc/shapes/ShapeTypes.hpp\"\n\n";

        if (!hull_arrays.str().empty())
        {
            out << "namespace {\n" << hull_arrays.str() << "}\n\n";
        }

        const std::string test_name = "regression_" + id;
        out << "TEST(" << test_name << ")\n{\n";
        if (!hull_setup.str().empty())
            out << hull_setup.str();
        out << "    rbc::Shape sa = " << ctor_a << ";\n";
        out << "    rbc::Shape sb = " << ctor_b << ";\n";
        out << "    m3d::tf tf_a = " << tf_a << ";\n";
        out << "    m3d::tf tf_b = " << tf_b << ";\n\n";

        switch (expected)
        {
        case Expect_Success:
        {
            out << "    rbc::MinkowskiDiff md(&sa, &sb, tf_a, tf_b);\n";
            out << "    rbc::GJK gjk;\n";
            out << "    ASSERT_TRUE(gjk.evaluate(md, tf_b.pos - tf_a.pos) == rbc::GJK::Inside);\n";
            out << "    rbc::EPA epa;\n";
            out << "    ASSERT_TRUE(epa.evaluate(gjk, md) == rbc::EPA::Valid);\n";
            // Tolerance 5% of captured depth or 0.01 floor.
            const m3d::scalar tol = m3d::max((m3d::scalar)0.01,
                                             (m3d::scalar)(0.05 * result.epa_depth));
            out << "    ASSERT_NEAR(epa.depth, " << result.epa_depth
                << ", " << tol << ");\n";
            out << "    ASSERT_NORMAL_UNIT(epa.normal);\n";
            out << "    rbc::ContactManifold mfd;\n";
            out << "    rbc::generate_manifold(epa.normal, epa.depth, epa.contact_point,\n";
            out << "                           sa, tf_a, sb, tf_b, mfd);\n";
            out << "    ASSERT_TRUE(mfd.num_points == "
                << result.manifold.num_points << ");\n";
            break;
        }
        case Expect_GJKSeparates:
        {
            out << "    rbc::MinkowskiDiff md(&sa, &sb, tf_a, tf_b);\n";
            out << "    rbc::GJK gjk;\n";
            out << "    ASSERT_TRUE(gjk.evaluate(md, tf_b.pos - tf_a.pos) == rbc::GJK::Valid);\n";
            break;
        }
        case Expect_EPAFails:
        {
            out << "    rbc::MinkowskiDiff md(&sa, &sb, tf_a, tf_b);\n";
            out << "    rbc::GJK gjk;\n";
            out << "    ASSERT_TRUE(gjk.evaluate(md, tf_b.pos - tf_a.pos) == rbc::GJK::Inside);\n";
            out << "    rbc::EPA epa;\n";
            out << "    ASSERT_TRUE(epa.evaluate(gjk, md) != rbc::EPA::Valid);\n";
            break;
        }
        case Expect_CrashRegression:
        {
            out << "    // Crash regression: any return value is acceptable.\n";
            out << "    // The test only fails if the call segfaults / aborts.\n";
            out << "    rbc::ContactManifold mfd;\n";
            out << "    (void)rbc::gjk_epa_manifold(sa, tf_a, sb, tf_b, mfd);\n";
            break;
        }
        }

        if (!cleanup.str().empty())
        {
            out << "\n" << cleanup.str();
        }

        out << "}\n\n";
        out << "TEST_SUITE(\n    RUN_TEST(" << test_name << ")\n)\n";

        return out.str();
    }

    // Write the regression test file. Returns the absolute or relative
    // path that was written, or empty on failure.
    inline std::string write_test_file(const DebuggerState &state,
                                       const PipelineResult &result,
                                       int expected,
                                       const std::string &name,
                                       const char *regression_dir)
    {
        const std::string id = export_detail::sanitize(name);
        std::string path = regression_dir;
        if (!path.empty() && path.back() != '/') path += '/';
        path += "test_";
        path += id;
        path += ".cpp";

        FILE *fp = std::fopen(path.c_str(), "wb");
        if (!fp) return {};

        const std::string content = format_test_cpp(state, result, expected, name);
        std::fwrite(content.data(), 1, content.size(), fp);
        std::fclose(fp);
        return path;
    }

} // namespace cdbg
