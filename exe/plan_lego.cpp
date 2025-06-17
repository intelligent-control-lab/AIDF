/**
 * @file plan_lego.cpp
 * @brief Entry point for running the Lego Skill Graph planner as a standalone executable.
 *
 * This program initializes the Lego Skill Graph, parses configuration, and runs the AssemblyPlanner.
 * ROS processes are now managed by MoveitInstance in moveit_backend.cpp for natural shutdown.
 */

#include "skillgraph.hpp"
#include "lego/lego_skillgraph.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "planner.h"

#include <cstdlib>

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
        
        // Parse command-line arguments
        if (argc > 1) {
            config_path = argv[1];
        } else {
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

        // get plan solution
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

        // Shutdown Logic (for natural exit)
        std::cout << "Main logic finished.  Shutting down..." << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        //Consider setting program_running = false; here to initiate shutdown.
    }

    return 0;
}