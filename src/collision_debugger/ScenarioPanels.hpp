#pragma once
#include <imgui.h>
#include "State.hpp"
#include "Scenario.hpp"
#include "TestExport.hpp"
#include "PipelineRun.hpp"
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <string>
#include <vector>
#include <algorithm>

namespace cdbg
{

    // Filesystem paths relative to the current working directory. Documented
    // in main.cpp: the binary expects to be launched from the project root.
    constexpr const char *kScenarioDir       = "scenarios";
    constexpr const char *kLastInputPath     = "scenarios/_last_input.scn";
    constexpr const char *kCleanShutdownPath = "scenarios/_last_input.ok";
    constexpr const char *kRegressionDir     = "tests/rbc/regression";

    inline std::vector<std::string> list_scenario_files()
    {
        std::vector<std::string> out;
        DIR *d = opendir(kScenarioDir);
        if (!d) return out;
        while (dirent *e = readdir(d))
        {
            const std::string name = e->d_name;
            if (name.size() < 5) continue;
            if (name.compare(name.size() - 4, 4, ".scn") != 0) continue;
            // Hide auto-save markers from the user-facing list
            if (name.rfind("_last_input", 0) == 0) continue;
            out.push_back(name);
        }
        closedir(d);
        std::sort(out.begin(), out.end());
        return out;
    }

    inline void draw_scenario_panel(DebuggerState        &state,
                                    const PipelineResult &result)
    {
        ImGui::SetNextWindowSize({420, 420}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos ({710, 380}, ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Scenarios"))
        {
            ImGui::End();
            return;
        }

        // ── Crash recovery banner ───────────────────────────────────────
        if (state.crash_recovery_pending)
        {
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.30f, 0.20f, 0.05f, 0.9f));
            ImGui::BeginChild("crash_banner",
                              ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 4),
                              true);
            ImGui::TextColored({1.0f, 0.85f, 0.30f, 1.0f},
                               "Last session ended without a clean shutdown.");
            ImGui::TextColored({1.0f, 0.85f, 0.30f, 1.0f},
                               "(_last_input.scn exists but no .ok marker)");
            ImGui::TextDisabled("Live recompute is paused so it doesn't fault on first frame.");
            if (ImGui::Button("Replay last crash"))
            {
                load_scenario(kLastInputPath, state);
                state.crash_recovery_pending = false;
                // Stay in not-live mode until user explicitly opts back in.
            }
            ImGui::SameLine(0, 12);
            if (ImGui::Button("Dismiss"))
                state.crash_recovery_pending = false;
            ImGui::EndChild();
            ImGui::PopStyleColor();
        }

        // ── Save current ────────────────────────────────────────────────
        ImGui::SeparatorText("Save current");
        static char save_name[64] = "scenario_01";
        ImGui::SetNextItemWidth(220);
        ImGui::InputText("name##save", save_name, sizeof(save_name));
        ImGui::SameLine(0, 8);
        if (ImGui::Button("Save##scn"))
        {
            ensure_dir(kScenarioDir);
            const std::string id = export_detail::sanitize(save_name);
            const std::string path = std::string(kScenarioDir) + "/" + id + ".scn";
            save_scenario(path.c_str(), state);
        }

        // ── Load existing ───────────────────────────────────────────────
        ImGui::SeparatorText("Load");
        const auto files = list_scenario_files();
        if (files.empty())
            ImGui::TextDisabled("(no saved scenarios in '%s/')", kScenarioDir);
        else if (ImGui::BeginChild("scn_list", ImVec2(0, 110), true))
        {
            for (const std::string &f : files)
            {
                const std::string path = std::string(kScenarioDir) + "/" + f;
                if (ImGui::Selectable(f.c_str()))
                    load_scenario(path.c_str(), state);
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - 60);
                std::string del_id = "Delete##" + f;
                if (ImGui::SmallButton(del_id.c_str()))
                    delete_file(path.c_str());
            }
            ImGui::EndChild();
        }

        // ── Export as test ──────────────────────────────────────────────
        ImGui::SeparatorText("Export as regression test");
        static char export_name[64] = "octahedron_vs_box";
        static int  expected_kind   = Expect_Success;
        static std:: string last_msg;
        static ImVec4         last_msg_col{0.6f, 0.9f, 0.6f, 1.0f};

        ImGui::SetNextItemWidth(260);
        ImGui::InputText("name##export", export_name, sizeof(export_name));

        const char *expected_labels[4] = {
            expected_label(Expect_Success),
            expected_label(Expect_GJKSeparates),
            expected_label(Expect_EPAFails),
            expected_label(Expect_CrashRegression),
        };
        ImGui::SetNextItemWidth(260);
        ImGui::Combo("expected##exp", &expected_kind, expected_labels, 4);

        // Quick-pick from current pipeline state — guesses the right mode
        if (ImGui::SmallButton("Auto-detect from current run"))
        {
            if (!result.overlap)
                expected_kind = Expect_GJKSeparates;
            else if (!result.epa_converged)
                expected_kind = Expect_EPAFails;
            else
                expected_kind = Expect_Success;
        }

        if (ImGui::Button("Export .cpp", ImVec2(160, 0)))
        {
            ensure_dir(kRegressionDir);
            const std::string path = write_test_file(
                state, result, expected_kind, export_name, kRegressionDir);
            if (path.empty())
            {
                last_msg     = "Failed to write file";
                last_msg_col = ImVec4(1.0f, 0.5f, 0.5f, 1.0f);
            }
            else
            {
                last_msg     = "Wrote " + path + " — re-run cmake to register";
                last_msg_col = ImVec4(0.6f, 0.95f, 0.6f, 1.0f);
            }
        }
        ImGui::SameLine(0, 8);
        if (ImGui::Button("Copy .cpp to clipboard", ImVec2(200, 0)))
        {
            const std::string s = format_test_cpp(state, result, expected_kind, export_name);
            ImGui::SetClipboardText(s.c_str());
            last_msg     = "Copied to clipboard";
            last_msg_col = ImVec4(0.6f, 0.95f, 0.6f, 1.0f);
        }

        if (!last_msg.empty())
            ImGui::TextColored(last_msg_col, "%s", last_msg.c_str());

        ImGui::End();
    }

} // namespace cdbg
