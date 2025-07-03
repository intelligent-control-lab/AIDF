/**
 * @file plan_magblock.cpp
 * @brief Entry point for running the MagBlock Skill Graph planner as a standalone executable.
 *
 * This program initializes the MagBlock Skill Graph, parses configuration, and runs the AssemblyPlanner,
 * exactly mirroring the LEGO implementation approach.
 */

#include "skillgraph.hpp"
#include "magblock/magblock_skillgraph.hpp"
#include "magblock/magblock_tasks.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "planner.h"

#include <cstdlib>

/**
 * @brief Main function for the MagBlock Skill Graph planner executable.
 *
 * Initializes the skill graph, planner, and executes the planning sequence,
 * exactly mirroring the LEGO implementation.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code.
 */
int main(int argc, char* argv[]) {
    try {
        // Default paths - using existing magblock task structure
        std::string config_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/skillgraph.json";
        std::string task_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/I.json";
        std::string env_setup_file = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_I.json";
        
        // Parse command-line arguments - following LEGO pattern
        if (argc > 1) {
            config_path = argv[1];
        }
        if (argc > 2) {
            task_file = argv[2];
        }
        if (argc > 3) {
            env_setup_file = argv[3];
        }
        
        log("Using config path: " + config_path, LogLevel::INFO);
        log("Using task file: " + task_file, LogLevel::INFO);
        log("Using env setup file: " + env_setup_file, LogLevel::INFO);

        // Initialize MagBlock Skill Graph - exactly mirroring LEGO approach
        auto sg = std::make_shared<skillgraph::MagBlockSkillGraph>(config_path);
        sg->initialize();
        log("MagBlock Skill Graph Initialized", LogLevel::INFO);
        sg->print_skillgraph();

        // Parse assembly tasks - exactly mirroring LEGO approach
        auto assembly_seq = std::make_shared<skillgraph::MagBlockAssemblySeq>(task_file);
        assembly_seq->print();
        // Note: setAssemblySeq method not available, task loading handled internally

        // Create the planner - exactly mirroring LEGO approach
        auto planner = std::make_shared<planner::AssemblyPlanner>(sg);
        planner->plan();

        // Execute plan solution - exactly mirroring LEGO approach
        skillgraph::State state = sg->get_initial_state();
        for (auto &skill : planner->plan_skill_seq) {
            log("Executing skill: " + std::to_string(static_cast<int>(skill->type)), LogLevel::INFO);
            
            // Execute the skill using get_next_state method
            skillgraph::State next_state;
            double cost;
            if (sg->get_next_state(state, skill, next_state, cost)) {
                log("Skill executed successfully", LogLevel::INFO);
                state = next_state;
            } else {
                log("Skill execution failed", LogLevel::ERROR);
                break;
            }
        }

        log("MagBlock assembly planning and execution completed", LogLevel::INFO);
        
    } catch (const std::exception& e) {
        log("Exception in MagBlock planner: " + std::string(e.what()), LogLevel::ERROR);
        return 1;
    }

    return 0;
}
