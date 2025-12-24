/**
 * @file plan_lego.cpp
 * @brief Entry point for running the Lego Skill Graph planner as a standalone executable.
 *
 * This program initializes the Lego Skill Graph, parses configuration, and runs the AssemblyPlanner.
 * ROS processes are now managed by MoveitInstance in moveit_backend.cpp for natural shutdown.
 */

#include "skillgraph.hpp"
#include "lego/lego_skillgraph.hpp"
#include "skillplan.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "planner.h"

#include <cstdlib>
#include <filesystem>

/**
 * @brief Main function for the Lego Skill Graph planner executable.
 *
 * Initializes the skill graph, planner, and executes the planning sequence.
 * Handles command-line arguments for configuration file selection.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code.
 */
int main(int argc, char* argv[]) {
    try {
        // Default path using PathResolver
        std::string config_path = skillgraph::utils::PathResolver::getDefaultSkillgraphConfig();
        bool execute_plan = true;
        bool config_set = false;
        bool out_set = false;
        std::string skillplan_out_path;
        
        auto print_usage = [&](const std::string &prog) {
            std::cout << "Usage: " << prog << " [--config <path>] [--out <skillplan.json>] [--no-exec]\n"
                      << "  Positional args are also supported: " << prog << " [config_path] [out_path]\n";
        };

        // Parse command-line arguments (flags + positional)
        std::vector<std::string> positional;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                print_usage(argv[0]);
                return 0;
            }
            if (arg == "--no-exec" || arg == "--export-only") {
                execute_plan = false;
                continue;
            }
            if (arg == "--config") {
                if (i + 1 >= argc) {
                    print_usage(argv[0]);
                    return 2;
                }
                config_path = argv[++i];
                config_set = true;
                continue;
            }
            if (arg == "--out") {
                if (i + 1 >= argc) {
                    print_usage(argv[0]);
                    return 2;
                }
                skillplan_out_path = argv[++i];
                out_set = true;
                continue;
            }
            if (!arg.empty() && arg[0] == '-') {
                std::cerr << "Unknown option: " << arg << "\n";
                print_usage(argv[0]);
                return 2;
            }
            positional.push_back(arg);
        }

        if (!positional.empty()) {
            if (!config_set) {
                config_path = positional[0];
            }
            if (positional.size() > 1 && !out_set) {
                skillplan_out_path = positional[1];
                out_set = true;
            }
        }

        if (!config_set) {
            log("Using default config path: " + config_path, LogLevel::INFO);
        }

        // Main Logic
        auto sg = std::make_shared<skillgraph::LegoSkillGraph>(config_path);
        sg->initialize();
        log("Lego Skill Graph Initialized", LogLevel::INFO);
        sg->print_skillgraph();

        // Create the planner
        auto planner = std::make_shared<planner::AssemblyPlanner>(sg);
        planner->plan();

        // Export skillplan.json (M1)
        if (!out_set) {
            std::filesystem::path cfg(config_path);
            std::filesystem::path parent = cfg.has_parent_path() ? cfg.parent_path() : std::filesystem::current_path();
            skillplan_out_path = (parent / "skillplan.json").string();
        }

        skillgraph::skillplan::ExportOptions export_opts;
        export_opts.include_environment_spec = true;
        export_opts.include_action_constraints = true;
        Json::Value skillplan_json = skillgraph::skillplan::export_plan(*sg, planner->plan_skill_seq, sg->get_initial_state(), export_opts);
        std::string export_error;
        if (!skillgraph::skillplan::write_json_to_file(skillplan_json, skillplan_out_path, &export_error)) {
            throw std::runtime_error("Failed to write skillplan.json: " + export_error);
        }
        log("Wrote skillplan to: " + skillplan_out_path, LogLevel::INFO);

        // get plan solution
        if (execute_plan) {
            skillgraph::State state = sg->get_initial_state();
            for (auto &skill : planner->plan_skill_seq) {
                // print atomic skills
                skillgraph::MetaSkillPtr meta_skill = std::dynamic_pointer_cast<skillgraph::MetaSkill>(skill);
                for (const auto &atomic_skill : meta_skill->atomic_skills) {
                    log(atomic_skill->to_string(), LogLevel::INFO);
                }

                if (!skill->executor->execute(state)) {
                    log("Failed to execute skill: " + skill->to_string(), LogLevel::ERROR);
                    break;
                }
            }
        } else {
            log("Skipping execution (--no-exec).", LogLevel::INFO);
        }

        // Shutdown Logic (for natural exit)
        std::cout << "Main logic finished.  Shutting down..." << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        //Consider setting program_running = false; here to initiate shutdown.
    }

    return 0;
}
