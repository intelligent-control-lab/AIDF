/**
 * @brief MagBlock Assembly Test Executable
 * 
 * This is a simple test executable that demonstrates the restructured MagBlock
 * assembly functionality using the skillgraph infrastructure with MoveIt backend.
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
#include "adg.h"
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
            
            skillgraph::MRTrajectory sync_solution(3); // Assuming 3 robots
            std::shared_ptr<ActivityGraph> act_graph = std::make_shared<ActivityGraph>(3);

            // Execute assembly tasks using skill execution loop
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
                
                // Execute the skill
                std::vector<skillgraph::RobotTrajectory> planned_trajectory;
                if (!skill_executor->execute(current_state, planned_trajectory)) {
                    log("Failed to execute skill for task " + std::to_string(current_task_num), LogLevel::ERROR);
                    return false;
                }
                

                // -------------------------- Construct synchronous solution and activity graph -------------------------- //
                // setup new activity
                for (const auto& traj : planned_trajectory) {
                    for (auto act_id : traj.act_ids) {
                        auto activity = std::make_shared<Activity>();
                        activity->robot_id = traj.robot_id;
                        activity->act_id = act_id;
                        activity->type = static_cast<Activity::Type>(act_id);
                        act_graph->add_act(traj.robot_id, activity->type);
                    }
                
                    // determine active robot trajectory duration
                    double skill_duration = traj.times.back() - traj.times.front();
                    double time_start = sync_solution[traj.robot_id].times.empty() ? 0.0 : sync_solution[traj.robot_id].times.back();

                    int robot_id = traj.robot_id;
                    skillgraph::RobotTrajectory &traj_accum = sync_solution[robot_id];
                    traj_accum.robot_id = robot_id;

                    // handle active robot
                    for (size_t i = 0; i < traj.trajectory.size(); ++i) {
                        traj_accum.trajectory.push_back(traj.trajectory[i]);
                        traj_accum.times.push_back(traj.times[i] + time_start);
                        traj_accum.act_ids.push_back(traj.act_ids.empty() ? i : traj.act_ids[i]);
                    }
                    traj_accum.cost += traj.cost;
                    
                    // handle idle robots
                    for (int other_robot_id = 0; other_robot_id < sync_solution.size(); ++other_robot_id) {
                        if (other_robot_id == robot_id) continue;

                        // add idle activity
                        auto idle_activity = std::make_shared<Activity>();
                        idle_activity->robot_id = other_robot_id;
                        idle_activity->act_id = 0; // 0 = home
                        idle_activity->type = Activity::Type::home;
                        idle_activity->start_pose.joint_values = {0.0, 0.0, 0.0, 2.5299, 0.0, 0.6120, 1.570}; // hard coded home pose for now
                        idle_activity->end_pose.joint_values = {0.0, 0.0, 0.0, 2.5299, 0.0, 0.6120, 1.570}; // hard coded home pose for now
                        act_graph->add_act(other_robot_id, Activity::Type::home);

                        skillgraph::RobotTrajectory &other_traj = sync_solution[other_robot_id];
                        other_traj.robot_id = other_robot_id;
                        std::vector<double> home_pose = {0.0, 0.0, 0.0, 2.5299, 0.0, 0.6120, 1.570};  // hard coded home pose for now

                        skillgraph::RobotState home_state;
                        home_state.robot_id = other_robot_id;
                        //home_state.robot_name = moveit_backend_->getRobotName(other_robot_id);
                        home_state.joint_values = home_pose;

                        double other_start_time = other_traj.times.empty() ? 0.0 : other_traj.times.back();

                        // one waypoint at start of active robot execution
                        other_traj.trajectory.push_back(home_state);
                        other_traj.times.push_back(other_start_time);
                        other_traj.act_ids.push_back(0); // 0 = home

                        // one waypoint at end
                        other_traj.trajectory.push_back(home_state);
                        other_traj.times.push_back(other_start_time + skill_duration);
                        other_traj.act_ids.push_back(0); // 0 = home

                        other_traj.cost += other_traj.times.back();
                    }
                }
                // ----------------------------------------------------------------------------------- //

                // complete_trajectory.push_back(planned_trajectory);
                moveit_backend_->executeJointTrajectory(planned_trajectory, 0.1);
                
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
            
            log("All assembly tasks completed successfully!", LogLevel::INFO);

            // int task_id = 0;
            // std::shared_ptr<ActivityGraph> act_graph = std::make_shared<ActivityGraph>(3); // Assuming 3 robots
            // std::vector<std::vector<skillgraph::RobotTrajectory>> robot_trajectories;

            // // CREATE THE TAS GRAPH BASED ON THE COMPLETE TRAJECTORY
            // for (const auto& multi_traj : complete_trajectory) {
            //     for (const auto& traj : multi_traj) {
            //         int robot_id = traj.robot_id;
            //         const auto& robot_traj = traj.trajectory;
            //         const auto& times = traj.times;
            //         const auto& act_ids = traj.act_ids;

            //         for (size_t i = 0; i + 1 < robot_traj.size(); ++i) {
            //             auto act = std::make_shared<Activity>();
            //             act->robot_id = robot_id;
            //             act->start_pose.joint_values = robot_traj[i].joint_values;
            //             act->end_pose.joint_values = robot_traj[i + 1].joint_values;
            //             act->act_id = act_ids.empty() ? i : act_ids[i];
            //             // Start time / end time?
            //             act->type = static_cast<Activity::Type>(traj.act_ids.empty() ? i : act_ids[i]);

            //             act_graph->add_act(robot_id, act->type, act);
            //         }

            //     }
            // }

            // skillgraph::MRTrajectory flat_trajectories;
            // for (const auto& robot_trajs : complete_trajectory) {
            //     flat_trajectories.insert(flat_trajectories.end(), robot_trajs.begin(), robot_trajs.end());
            // }

            // try {
            //     act_graph->saveGraphToFile("/home/arcs-arm/threearm_moveit_ws/src/AIDF/test_tpg_output/activity_graph.txt");
            // } catch (const std::exception& e) {
            //     log("Error saving activity graph: " + std::string(e.what()), LogLevel::ERROR);
            // }

            tpg::TPG tpg;
            std::cerr << "Dumping sync_solution before TPG init:\n";
            for (int i = 0; i < sync_solution.size(); ++i) {
                std::cerr << "- Robot " << i << " trajectory size: " << sync_solution[i].trajectory.size()
                        << ", act_ids: " << sync_solution[i].act_ids.size()
                        << ", cost: " << sync_solution[i].cost
                        << ", robot_id in struct: " << sync_solution[i].robot_id << "\n";
            }

            bool success = tpg.init(moveit_backend_, sync_solution, tpg_config_);
            if (!success) {
                log("Failed to initialize TPG", LogLevel::ERROR);
                return false;
            }
            tpg.saveToDotFile("/home/arcs-arm/threearm_moveit_ws/src/AIDF/test_tpg_output/tpg.dot");

            auto adg = std::make_shared<tpg::ADG>(act_graph);
            adg->init_from_asynctrajs(moveit_backend_, tpg_config_, sync_solution);
            adg->saveToDotFile("/home/arcs-arm/threearm_moveit_ws/src/AIDF/test_tpg_output/adg.dot");

            return true;
            
        } catch (const std::exception& e) {
            log("Error executing assembly task: " + std::string(e.what()), LogLevel::ERROR);
            return false;
        }
    }
    
    void runTest() {
        log("Running MagBlock Assembly Test", LogLevel::INFO);
        
        // Test with a simple assembly sequence
        std::string test_task = "/home/arcs-arm/threearm_moveit_ws/src/AIDF/config/mag_block_tasks/assembly_tasks/three_I.json";
        // loadTaskConfig();
        // std::string test_task = PathResolver::resolvePath(task_config_["tasks"]["assembly_seq"].asString());
        
        if (executeAssemblyTask(test_task)) {
            log("MagBlock Assembly Test PASSED", LogLevel::INFO);
        } else {
            log("MagBlock Assembly Test FAILED", LogLevel::ERROR);
        }
        
        log("Test completed", LogLevel::INFO);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MagBlockSkillGraph> skillgraph_;
    std::shared_ptr<MoveitInstance> moveit_backend_;
    tpg::TPGConfig tpg_config_;
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
