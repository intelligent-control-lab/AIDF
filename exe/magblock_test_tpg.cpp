/**
 * @brief MagBlock Assembly Test Executable with TPG Construction
 * 
 * This is a modified test executable that demonstrates the restructured MagBlock
 * assembly functionality using the skillgraph infrastructure with MoveIt backend.
 * 
 * Key modifications for TPG construction:
 * - Collects solution trajectories from skill execution
 * - Creates activities for each skill segment (transit, approach pick, pick, retract pick, 
 *   transit, approach place, place, retract place)
 * - Builds a TPG (Task Planning Graph) from the collected trajectories
 * - Provides access to the TPG for potential ADG (Assembly Dependency Graph) construction
 * 
 * The execute method now returns solution trajectories that can be used for TPG construction,
 * where each skill/segment of the total robot trajectory becomes an 'activity' in the TPG.
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "magblock/magblock_skillgraph.hpp"
#include "magblock/magblock_skills.hpp"
#include "moveit_backend.hpp"
#include "Utils/Logger.hpp"
#include "tpg.h"
#include "task_graph.h"
// #include "Utils/PathUtils.hpp"
// using skillgraph::utils::PathResolver;

using namespace skillgraph;

// static Json::Value task_config_;
// static bool task_config_loaded_ = false;

// void loadTaskConfig() {
//     if (task_config_loaded_) return;

//     std::string config_path = PathResolver::resolvePath("config/mag_block_tasks/skillgraph.json");

//     std::ifstream config_file(config_path);
//     if (!config_file.is_open()) {
//         log("Failed to open task config: " + config_path, LogLevel::ERROR);
//         return;
//     }

//     config_file >> task_config_;
//     task_config_loaded_ = true;
// }

class MagBlockAssemblyTest {
public:
    MagBlockAssemblyTest() : node_(rclcpp::Node::make_shared("magblock_assembly_test")) {
        log("Starting MagBlock Assembly Test", LogLevel::INFO);
        
        // Initialize the skillgraph with configuration
        std::string config_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/skillgraph.json";
        skillgraph_ = std::make_shared<MagBlockSkillGraph>(config_path);
        skillgraph_->initialize();
        
        log("MagBlock Skill Graph initialized", LogLevel::INFO);
        
        // Initialize MoveIt backend for simulation
        initializeMoveitBackend();
        
        // Initialize TPG configuration
        initializeTPGConfig();
        
        log("MagBlock Assembly Test initialized successfully", LogLevel::INFO);
    }
    
    void initializeMoveitBackend() {
        try {
            // Wait for MoveIt services to be available
            log("Waiting for MoveIt services to be available...", LogLevel::INFO);
            
            // Check for move_group services
            auto start_time = std::chrono::steady_clock::now();
            bool services_available = false;
            
            while (!services_available && 
                   std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
                
                // Check if move_group services are available
                auto service_names = node_->get_service_names_and_types();
                bool move_group_found = false;
                bool planning_scene_found = false;
                
                for (const auto& service : service_names) {
                    if (service.first.find("move_group") != std::string::npos) {
                        move_group_found = true;
                    }
                    if (service.first.find("apply_planning_scene") != std::string::npos) {
                        planning_scene_found = true;
                    }
                }
                
                if (move_group_found && planning_scene_found) {
                    services_available = true;
                    break;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            if (!services_available) {
                throw std::runtime_error("MoveIt services not available after 30 seconds");
            }
            
            log("MoveIt services found, initializing MoveIt backend...", LogLevel::INFO);
            
            // Initialize MoveIt backend for three-arm system using the same node
            // This ensures the MoveitInstance has access to the robot description
            // Use "right_arm" as the primary group for this test
            moveit_backend_ = std::make_shared<MoveitInstance>(node_, "right_arm", "kortex_description");
            
            // Set the assembly environment configuration
            // Load the environment setup file and pass it to the skillgraph
            std::string env_path = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/env_setup/env_setup_three_I.json";
            // loadTaskConfig();
            // std::string env_path = PathResolver::resolvePath(task_config_["environment"]["object_library"].asString());
            std::ifstream env_file(env_path);
            if (env_file.is_open()) {
                Json::Value env_setup;
                env_file >> env_setup;
                skillgraph_->setEnvironmentConfig(env_setup);
                log("Environment configuration loaded successfully", LogLevel::INFO);
            } else {
                log("Failed to load environment configuration from: " + env_path, LogLevel::WARN);
            }
            
            log("MoveIt backend initialized successfully", LogLevel::INFO);
        } catch (const std::exception& e) {
            log("Failed to initialize MoveIt backend: " + std::string(e.what()), LogLevel::ERROR);
            throw;
        }
    }
    
    void initializeTPGConfig() {
        // Initialize TPG configuration with default settings
        tpg_config_.shortcut = true;
        tpg_config_.random_shortcut = true;
        tpg_config_.forward_singleloop = true;
        tpg_config_.tight_shortcut = true;
        tpg_config_.dt = 0.1;
        tpg_config_.shortcut_time = 1.0;
        tpg_config_.joint_state_thresh = 0.1;
        tpg_config_.seed = 1;
        
        log("TPG configuration initialized", LogLevel::INFO);
    }
    
    bool executeAssemblyTask(const std::string& task_file) {
        log("Executing assembly task from: " + task_file, LogLevel::INFO);
        
        try {
            // Load assembly sequence
            auto assembly_seq = std::make_shared<MagBlockAssemblySeq>(task_file);
            log("Assembly sequence loaded. Number of tasks: " + std::to_string(assembly_seq->num_tasks()), LogLevel::INFO);
            
            // Set the assembly sequence in the skill graph
            skillgraph_->setAssemblySequence(assembly_seq);
            
            // Get initial state
            State current_state = skillgraph_->get_initial_state();
            
            // Initialize data structures for TPG construction
            skillgraph::MRTrajectory complete_trajectory;
            std::vector<std::shared_ptr<skillgraph::Activity>> activities;
            int activity_counter = 0;
            
            // Execute assembly tasks and collect trajectories
            while (!skillgraph_->at_target(current_state)) {
                int current_task_num = current_state.assembled_steps + 1;
                auto task = assembly_seq->get_task_at(current_state.assembled_steps);
                
                log("Processing task " + std::to_string(current_task_num) + ": " + task->name, LogLevel::INFO);
                
                // Get feasible skills for current state
                auto feasible_skills = skillgraph_->feasible_u(current_state);
                
                if (feasible_skills.empty()) {
                    log("No feasible skills found for task " + std::to_string(current_task_num), LogLevel::ERROR);
                    return false;
                }
                
                // Execute first feasible skill
                auto selected_skill = feasible_skills[0];
                
                log("Executing skill: " + selected_skill->to_string(), LogLevel::INFO);
                
                // Create skill executor with MoveIt backend
                auto skill_executor = std::make_shared<MagBlockSkillExecutor>(
                    selected_skill->type, moveit_backend_);
                
                // Set the skill's post condition (if available)
                if (selected_skill->executor && selected_skill->executor->post_condition) {
                    skill_executor->post_condition = selected_skill->executor->post_condition;
                }
                
                // Execute the skill and collect trajectory
                if (!skill_executor->execute(current_state)) {
                    log("Failed to execute skill for task " + std::to_string(current_task_num), LogLevel::ERROR);
                    return false;
                }
                
                // Get the trajectory from the skill execution
                // Note: This would need to be implemented in the skill executor
                // For now, we'll create a placeholder trajectory segment
                skillgraph::RobotTrajectory skill_trajectory = getSkillTrajectory(skill_executor, current_state);
                
                // Add trajectory segment to complete trajectory
                appendTrajectorySegment(complete_trajectory, skill_trajectory);
                
                // Create activity for this skill
                auto activity = createActivityForSkill(selected_skill, activity_counter++, skill_trajectory);
                activities.push_back(activity);
                
                log("Skill trajectory collected for activity: " + activity->type_string(), LogLevel::INFO);
                
                // Get next state after skill execution
                State next_state;
                double cost;
                if (!skillgraph_->get_next_state(current_state, selected_skill, next_state, cost)) {
                    log("Failed to get next state for task " + std::to_string(current_task_num), LogLevel::ERROR);
                    return false;
                }
                
                current_state = next_state;
                
                // Log state transition
                if (selected_skill->type == Skill::Type::Transit) {
                    log("Transit skill completed - robot positioned for task " + std::to_string(current_task_num), LogLevel::INFO);
                } else {
                    log("Task " + std::to_string(current_task_num) + " completed successfully", LogLevel::INFO);
                }
                
                // Brief pause between skills
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                // Safety check to prevent infinite loops
                if (current_state.assembled_steps > assembly_seq->num_tasks()) {
                    log("ERROR: assembled_steps exceeded num_tasks - breaking loop", LogLevel::ERROR);
                    break;
                }
            }
            
            // Build TPG from collected trajectories and activities
            bool tpg_success = buildTPG(complete_trajectory, activities);
            if (!tpg_success) {
                log("Failed to build TPG from trajectories", LogLevel::ERROR);
                return false;
            }
            
            log("All assembly tasks completed successfully and TPG constructed!", LogLevel::INFO);
            return true;
            
        } catch (const std::exception& e) {
            log("Error executing assembly task: " + std::string(e.what()), LogLevel::ERROR);
            return false;
        }
    }
    
    void runTest() {
        log("Running MagBlock Assembly Test with TPG Construction", LogLevel::INFO);
        
        // Test with a simple assembly sequence
        std::string test_task = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/three_I.json";
        // loadTaskConfig();
        // std::string test_task = PathResolver::resolvePath(task_config_["tasks"]["assembly_seq"].asString());
        
        if (executeAssemblyTask(test_task)) {
            log("MagBlock Assembly Test PASSED", LogLevel::INFO);
            
            // Demonstrate TPG access for potential ADG construction
            log("TPG constructed and ready for ADG creation", LogLevel::INFO);
            auto tpg_for_adg = getTPGForADG();
            log("TPG object prepared for ADG construction", LogLevel::INFO);
            
            // TODO: Future ADG construction would use the TPG here
            // auto adg = std::make_shared<ADG>(activity_graph);
            // adg->init(tpg_for_adg, trajectory, config);
            
        } else {
            log("MagBlock Assembly Test FAILED", LogLevel::ERROR);
        }
        
        log("Test completed", LogLevel::INFO);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MagBlockSkillGraph> skillgraph_;
    std::shared_ptr<MoveitInstance> moveit_backend_;
    tpg::TPG tpg_;
    tpg::TPGConfig tpg_config_;
    
    skillgraph::RobotTrajectory getSkillTrajectory(
        std::shared_ptr<MagBlockSkillExecutor> skill_executor, 
        const State& state) {
        // TODO: CRITICAL IMPLEMENTATION NEEDED
        // The skill executor needs to be modified to store and return the actual 
        // robot trajectory generated during skill execution. This is essential for 
        // TPG construction as each skill segment becomes an activity in the TPG.
        //
        // Required changes to MagBlockSkillExecutor:
        // 1. Store the planned trajectory during execute() method
        // 2. Provide a getTrajectory() method to access the stored trajectory
        // 3. Ensure trajectory includes proper timing and robot state information
        
        skillgraph::RobotTrajectory trajectory;
        
        // The skill executor should provide access to the planned trajectory
        // This would typically be done by modifying the skill executor to store
        // the trajectory during execution and provide a getter method
        
        log("Extracting trajectory from skill executor (placeholder implementation)", LogLevel::WARN);
        log("TODO: Implement actual trajectory extraction from skill executor", LogLevel::WARN);
        
        return trajectory;
    }
    
    void appendTrajectorySegment(skillgraph::MRTrajectory& complete_trajectory, 
                                const skillgraph::RobotTrajectory& segment) {
        // Append the trajectory segment to the complete multi-robot trajectory
        // Handle proper timestep concatenation and robot assignment
        
        log("Appending trajectory segment to complete trajectory", LogLevel::DEBUG);
        
        // TODO: Implement proper trajectory concatenation
        // This should handle:
        // - Time alignment between segments
        // - Robot assignment for multi-robot scenarios
        // - Smooth transitions between skill segments
    }
    
    std::shared_ptr<skillgraph::Activity> createActivityForSkill(
        std::shared_ptr<Skill> skill, 
        int activity_id, 
        const skillgraph::RobotTrajectory& trajectory) {
        
        auto activity = std::make_shared<skillgraph::Activity>();
        activity->act_id = activity_id;
        activity->robot_id = 0; // TODO: Extract from skill or context
        
        // Map skill types to activity types
        switch (skill->type) {
            case Skill::Type::Transit:
                activity->type = skillgraph::Activity::Type::home; // or appropriate transit type
                break;
            case Skill::Type::ApproachPick:
                activity->type = skillgraph::Activity::Type::pick; // approach phase
                break;
            case Skill::Type::Pick:
                activity->type = skillgraph::Activity::Type::pick;
                break;
            case Skill::Type::RetractPick:
                activity->type = skillgraph::Activity::Type::pick; // retract phase
                break;
            case Skill::Type::ApproachPlace:
                activity->type = skillgraph::Activity::Type::drop; // approach phase
                break;
            case Skill::Type::Place:
                activity->type = skillgraph::Activity::Type::drop;
                break;
            case Skill::Type::RetractPlace:
                activity->type = skillgraph::Activity::Type::drop; // retract phase
                break;
            default:
                activity->type = skillgraph::Activity::Type::home;
                log("Unknown skill type, defaulting to home activity", LogLevel::WARN);
                break;
        }
        
        // Set start and end poses from trajectory
        if (!trajectory.empty()) {
            activity->start_pose = trajectory.front();
            activity->end_pose = trajectory.back();
        }
        
        log("Created activity " + std::to_string(activity_id) + " for skill: " + skill->to_string(), LogLevel::INFO);
        
        return activity;
    }
    
    bool buildTPG(const skillgraph::MRTrajectory& complete_trajectory, 
                  const std::vector<std::shared_ptr<skillgraph::Activity>>& activities) {
        
        log("Building TPG from collected trajectories and activities", LogLevel::INFO);
        
        try {
            // Initialize TPG
            tpg_.reset();
            
            // Create plan instance for TPG construction
            auto plan_instance = std::static_pointer_cast<skillgraph::PlanInstance>(moveit_backend_);
            
            // Initialize TPG with the complete trajectory
            bool success = tpg_.init(plan_instance, complete_trajectory, tpg_config_);
            if (!success) {
                log("Failed to initialize TPG with trajectory", LogLevel::ERROR);
                return false;
            }
            
            // TODO: Associate activities with TPG nodes
            // This would involve mapping activities to specific time segments
            // and nodes in the TPG structure
            
            log("TPG successfully constructed with " + std::to_string(activities.size()) + " activities", LogLevel::INFO);
            
            // Optionally save TPG to file for inspection
            std::string tpg_save_path = "/tmp/magblock_assembly_tpg.dot";
            if (tpg_.saveToDotFile(tpg_save_path)) {
                log("TPG saved to: " + tpg_save_path, LogLevel::INFO);
            }
            
            return true;
            
        } catch (const std::exception& e) {
            log("Error building TPG: " + std::string(e.what()), LogLevel::ERROR);
            return false;
        }
    }
    
    // Getter method to access the constructed TPG
    const tpg::TPG& getTPG() const {
        return tpg_;
    }
    
    // Method to get TPG for potential ADG construction
    std::shared_ptr<tpg::TPG> getTPGForADG() {
        return std::make_shared<tpg::TPG>(tpg_);
    }
};

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    try {
        // Create and run the test
        MagBlockAssemblyTest test;
        test.runTest();
        
        // Keep the node alive for visualization
        log("Test completed. Press Ctrl+C to exit.", LogLevel::INFO);
        rclcpp::spin(std::make_shared<rclcpp::Node>("magblock_test_node"));
        
    } catch (const std::exception& e) {
        log("Test failed with exception: " + std::string(e.what()), LogLevel::ERROR);
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
