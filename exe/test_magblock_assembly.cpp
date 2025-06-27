#include <iostream>
#include <string>
#include <memory>
#include <filesystem>

#include "skillgraph/api/skillgraph.hpp"
#include "skillgraph/include/magblock/magblock_skillgraph.hpp"
#include "skillgraph/api/data_structure.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    std::cout << "Testing MagBlock Assembly SkillGraph..." << std::endl;

    // Initialize paths
    fs::path config_path = fs::path(AIDF_CONFIG_PATH) / "mag_block_tasks";
    fs::path skillgraph_config = config_path / "skillgraph.json";
    fs::path assembly_path = config_path / "assembly_tasks" / "I.json";

    if (!fs::exists(skillgraph_config)) {
        std::cerr << "Skillgraph config not found at: " << skillgraph_config << std::endl;
        return 1;
    }

    if (!fs::exists(assembly_path)) {
        std::cerr << "Assembly task file not found at: " << assembly_path << std::endl;
        return 1;
    }

    try {
        // Initialize skillgraph
        std::cout << "Initializing SkillGraph from: " << skillgraph_config << std::endl;
        auto skillgraph = std::make_shared<MagBlockSkillGraph>(skillgraph_config.string());

        // Load assembly sequence
        std::cout << "Loading assembly sequence from: " << assembly_path << std::endl;
        skillgraph->loadAssemblySequence(assembly_path.string());

        // Get initial state
        auto current_state = skillgraph->getInitialState();
        std::cout << "Initial state loaded." << std::endl;

        // Process each step in the assembly
        int step = 1;
        while (!skillgraph->isGoalState(current_state)) {
            std::cout << "\nProcessing Step " << step << ":" << std::endl;

            // Get feasible skills for current state
            auto feasible_skills = skillgraph->feasibleU(current_state);
            std::cout << "Found " << feasible_skills.size() << " feasible skills" << std::endl;

            if (feasible_skills.empty()) {
                std::cerr << "No feasible skills found for step " << step << std::endl;
                break;
            }

            // For each feasible skill, check state transition
            for (const auto& skill : feasible_skills) {
                std::cout << "Testing skill: " << skill->getName() << std::endl;
                
                // Get next state
                auto next_state = skillgraph->getNextState(current_state, skill);
                if (next_state) {
                    std::cout << "State transition successful" << std::endl;
                    current_state = next_state;
                    break;
                }
            }

            step++;
        }

        std::cout << "\nAssembly sequence test completed." << std::endl;
        if (skillgraph->isGoalState(current_state)) {
            std::cout << "Successfully reached goal state!" << std::endl;
        } else {
            std::cout << "Failed to reach goal state." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
