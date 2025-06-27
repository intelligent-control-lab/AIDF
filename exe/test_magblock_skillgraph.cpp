#include "magblock/magblock_skillgraph.hpp"
#include <iostream>
#include <memory>

using namespace skillgraph;

int main(int argc, char** argv) {
    // Path to the skillgraph config and assembly sequence
    std::string config_path = "config/mag_block_tasks/skillgraph.json";
    std::string assembly_path = "config/mag_block_tasks/assembly/simple_stack.json";

    // Initialize the skillgraph
    auto sg = std::make_shared<MagBlockSkillGraph>(config_path);
    sg->initialize();
    std::cout << "MagBlock Skill Graph Initialized" << std::endl;
    sg->print_skillgraph();

    // Parse the assembly sequence
    auto mag_seq = std::make_shared<MagBlockAssemblySeq>();
    if (!mag_seq->parse_from_json(assembly_path)) {
        std::cerr << "Failed to parse assembly sequence." << std::endl;
        return 1;
    }
    std::cout << "Assembly sequence loaded. Number of tasks: " << mag_seq->num_tasks() << std::endl;

    // Set the task sequence in the skillgraph
    sg->task_seq_ = mag_seq;

    // Get initial state
    State state = sg->get_initial_state();

    // For each task, get feasible skills and simulate state transitions
    for (int i = 0; i < mag_seq->num_tasks(); ++i) {
        std::cout << "\n--- Task " << i+1 << " ---" << std::endl;
        auto feasible_skills = sg->feasible_u(state);
        if (feasible_skills.empty()) {
            std::cerr << "No feasible skills found for task " << i+1 << std::endl;
            break;
        }
        for (const auto& skill : feasible_skills) {
            std::cout << "Feasible skill: " << skill->to_string() << std::endl;
        }
        // Simulate applying the first feasible skill
        State next_state;
        double cost = 0.0;
        if (sg->get_next_state(state, feasible_skills[0], next_state, cost)) {
            std::cout << "Transitioned to next state. Cost: " << cost << std::endl;
            state = next_state;
        } else {
            std::cerr << "Failed to transition to next state." << std::endl;
            break;
        }
    }
    std::cout << "\nTest complete. Final state assembled steps: " << state.assembled_steps << std::endl;
    return 0;
}
