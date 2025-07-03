#include "magblock/magblock_skillgraph.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace skillgraph;

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("magblock_test");

    // Path to the skillgraph config and assembly sequence
    std::string config_path = "config/mag_block_tasks/skillgraph.json";
    std::string assembly_path = "config/mag_block_tasks/assembly_tasks/I.json";

    // Initialize the skillgraph
    auto sg = std::make_shared<MagBlockSkillGraph>(config_path);
    sg->initialize();
    std::cout << "MagBlock Skill Graph Initialized" << std::endl;
    sg->print_skillgraph();

    // Parse the assembly sequence
    auto mag_seq = std::make_shared<MagBlockAssemblySeq>(assembly_path);
    std::cout << "Assembly sequence loaded. Number of tasks: " << mag_seq->num_tasks() << std::endl;

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
        double cost;
        if (sg->get_next_state(state, feasible_skills[0], next_state, cost)) {
            std::cout << "Transitioned to next state." << std::endl;
            state = next_state;
        } else {
            std::cerr << "Failed to transition to next state." << std::endl;
            break;
        }
    }
    rclcpp::shutdown();
    return 0;
}
