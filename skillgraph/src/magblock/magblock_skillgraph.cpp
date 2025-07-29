#include "magblock/magblock_skills.hpp"
#include "magblock/magblock_skillgraph.hpp"
#include "magblock/magblock_algorithms.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {

/**
 * @brief Construct a MagBlockSkillGraph from a configuration file.
 * @param config_file Path to the configuration file.
 */
MagBlockSkillGraph::MagBlockSkillGraph(const std::string &config_file) : SkillGraph(config_file)
{
    // No need for robot configurations - use magblock_algorithms functions
}

/**
 * @brief Get the initial state with properly initialized robot states.
 * @return Initial state with robot states initialized.
 */
State MagBlockSkillGraph::get_initial_state() {
    // Get base initial state
    State initial_state = SkillGraph::get_initial_state();
    
    // Initialize robot states
    initial_state.robot_states.clear();
    for (int i = 0; i < robots.size(); i++) {
        RobotState robot_state;
        robot_state.robot_id = i;
        robot_state.robot_name = robots[i]->robot_name;
        robot_state.attributes["at_approach_position"] = false;
        
        // Initialize joint values to zeros (or home position)
        int dof = robots[i]->getDOF();
        robot_state.joint_values = {0.0, 0.0, 0.0, 2.5299, 0.0, 0.6120, 1.57};
        
        initial_state.robot_states.push_back(robot_state);
    }
    
    log("Initialized robot states for " + std::to_string(robots.size()) + " robots", LogLevel::INFO);
    
    return initial_state;
}

/**
 * @brief Parse the environment configuration for the MagBlock skill graph.
 *
 * This method sets up the backend, MoveIt instance, and robot parameters from the JSON configuration
 * @param root_config Root JSON configuration.
 */
void MagBlockSkillGraph::parse_env(const Json::Value &root_config) {
    // First, use the base class implementation
    SkillGraph::parse_env(root_config);
 
    const Json::Value& env_config = root_config["environment"];
    const Json::Value& backend_config = env_config["backend"];
    std::string moveitConfigPkg = backend_config["moveitConfigPkg"].asString();
    
    std::string backend = backend_config["type"].asString();
    if (backend != "moveit") {
        throw std::runtime_error("Backend must be moveit for MagBlock assembly! " + backend);
    }

    // Create the moveit instance backend
    auto plan_instance_ = std::make_shared<MoveitInstance>(backend_config["moveit_group_name"].asString(), 
                            moveitConfigPkg);

    // Set the number of robots and their names
    plan_instance_->setNumberOfRobots(num_robots_);
    std::vector<std::string> robot_names;
    for (int i = 0; i < num_robots_; i++) {
        plan_instance_->setRobotDOF(i, robots[i]->getDOF()+1); // the moveit backend adds a dummy gripper dof
    }
    plan_instance_->setRobotNames(get_robot_names());
    if (backend_config.isMember("l1_vmax")) {
        plan_instance_->setVmax(backend_config["l1_vmax"].asDouble());
    }
    
    initial_state_ = get_initial_state();
    env_->setBackend(plan_instance_);
    for (int i = 0; i < num_robots_; i++) {
        // print the initial robot state
        std::cout << "Initial state for robot " << i << ": " << std::endl;
        for (const auto& joint_value : initial_state_.robot_states[i].joint_values) {
            std::cout << joint_value << " ";
        }
        std::cout << std::endl;
        env_->backend_->moveRobot(i, initial_state_.robot_states[i]);
    }
    
    log("MagBlock environment initialized with " + std::to_string(num_robots_) + " robots", LogLevel::INFO);

    // Parse object library for initial MagBlock positions
    std::string root_pwd = env_config["rootPwd"].asString();
    const char* env_root = std::getenv("AIDF_PROJECT_ROOT");
    if (env_root && root_pwd.find("${AIDF_PROJECT_ROOT}") == 0) {
        root_pwd = std::string(env_root) + root_pwd.substr(20); // remove the placeholder
    }

    std::string object_lib_path = root_pwd + env_config["object_library"].asString();
    std::ifstream obj_lib_file(object_lib_path);
    if (!obj_lib_file.is_open()) {
        throw std::runtime_error("Failed to open object library at: " + object_lib_path);
    }

    Json::Value object_lib_json;
    obj_lib_file >> object_lib_json;

    // Insert each block into the environment
    EnvState init_state;

    for (const auto& key : object_lib_json.getMemberNames()) {
        const Json::Value& block = object_lib_json[key];

        double x = block["x"].asDouble();
        double y = block["y"].asDouble();
        double z = block["z"].asDouble();
        std::cout << x << " " << y << " " << z << std::endl;
        // GET ROBOT BASE LINK, GET BLOCK TO ROBOT
        // const Eigen::Isometry3d& ee_to_world = robot_state.getGlobalLinkTransform(base_link);
        // Eigen::Isometry3d target_transform_world = ee_to_world * target_transform;

        // Create the MagBlock object
        ObjPtr mag_block = std::make_shared<Object>();
        mag_block->name = key;
        mag_block->state = Object::State::Static;
        mag_block->parent_link = "world";
        mag_block->shape = Object::Shape::Box;  

        double block_in_robot_x, block_in_robot_y, block_in_robot_z, rx_deg, ry_deg, rz_deg;
        transformSkillgraphToRobot("right_arm", x, y, z, 
                              block_in_robot_x, block_in_robot_y, block_in_robot_z, rx_deg, ry_deg, rz_deg);

        geometry_msgs::msg::Pose target_transform_world;
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = block_in_robot_x;
        target_pose.position.y = block_in_robot_y;
        target_pose.position.z = block_in_robot_z;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 1.0;
        auto moveit_backend = std::dynamic_pointer_cast<MoveitInstance>(env_->backend_);
        if (moveit_backend) {
            moveit_backend->transformRobotToWorld("right_arm", target_pose, target_transform_world);
        } else {
            log("Failed to cast backend_ to MoveitInstance", LogLevel::ERROR);
        }
        
        // Set position
        mag_block->x = target_transform_world.position.x;
        mag_block->y = target_transform_world.position.y;
        mag_block->z = target_transform_world.position.z - 0.159;
        mag_block->qx = target_transform_world.orientation.x;
        mag_block->qy = target_transform_world.orientation.y;
        mag_block->qz = target_transform_world.orientation.z;
        mag_block->qw = target_transform_world.orientation.w;
        mag_block->length = 0.025;
        mag_block->width = 0.025;
        mag_block->height = 0.025;

        // Add to MoveIt and initial environment state
        env_->backend_->addMoveableObject(*mag_block);
        init_state.objects.push_back(mag_block);
    }

    // Update planning scene
    env_->backend_->updateScene();

    // Save as initial environment state
    env_->setState(init_state);
    initial_state_.env_state = init_state;

}

/**
 * @brief Check if we are at the target state.
 *
 * This method checks if all magblock assembly tasks have been completed
 * @param state Current state.
 * @return True if at target.
 */
bool MagBlockSkillGraph::at_target(const State &state) {
    // Check if all assembly tasks are complete
    return state.assembled_steps >= task_seq_->num_tasks();
}

/**
 * @brief Get feasible skills for the current state.
 *
 * This method determines which skills can be executed in the current state, 
 * implementing proper precondition checks for robot positioning.
 * @param state Current state.
 * @return Vector of feasible skills.
 */
std::vector<SkillPtr> MagBlockSkillGraph::feasible_u(const State &state) {
    std::vector<SkillPtr> feasible_skills;
    
    // Get the next task to execute
    int assembled_steps = state.assembled_steps;
    if (assembled_steps >= task_seq_->num_tasks()) {
        log("All magblock assembly tasks completed", LogLevel::INFO);
        return feasible_skills;
    }
    
    TaskPtr current_task = task_seq_->get_task_at(assembled_steps);
    
    // Check if task is valid
    if (!current_task) {
        log("Invalid task at step " + std::to_string(assembled_steps), LogLevel::ERROR);
        return feasible_skills;
    }
    
    // Get task constraints for robot and position information
    const Json::Value& constraints = current_task->post_condition->constraints_json;
    const std::vector<Skill::Type>& allowed_skills = current_task->post_condition->allowed_skill_type;

    // GETTING HARDCODED ROBOT ID
    int assigned_robot_id = constraints.get("robot_id", 2).asInt();
    std::string robot_name = get_robot_names()[assigned_robot_id];
    
    // Extract target positions from constraints
    double target_x = constraints.get("x", 0.0).asDouble();
    double target_y = constraints.get("y", 0.0).asDouble();
    double target_z = constraints.get("z", 0.0).asDouble();
    
    log("Processing task " + std::to_string(assembled_steps) + " at position (" + 
        std::to_string(target_x) + "," + std::to_string(target_y) + "," + std::to_string(target_z) + 
        ") with robot " + robot_name, LogLevel::INFO);
    
    // Check robot's current state and position
    bool robot_at_approach_position = false;
    if (assigned_robot_id < state.robot_states.size()) {
        robot_at_approach_position = isRobotAtApproachPosition(state.robot_states[assigned_robot_id], 
                                                               target_x, target_y, target_z, robot_name);
    } else {
        log("Warning: Robot states not initialized properly. Robot ID " + std::to_string(assigned_robot_id) + 
            " >= state.robot_states.size() " + std::to_string(state.robot_states.size()), LogLevel::WARN);
        // Robot states not initialized - assume robot is not at approach position
        robot_at_approach_position = false;
    }
    
    log("Robot " + robot_name + " at approach position: " + (robot_at_approach_position ? "YES" : "NO"), LogLevel::INFO);
    log("Allowed skills for this task: " + std::to_string(allowed_skills.size()), LogLevel::INFO);
    for (const auto& skill_type : allowed_skills) {
        log("  - " + skillTypeToString(skill_type), LogLevel::INFO);
    }
    
    // Process each allowed skill type with proper precondition checks
    for (const auto& skill_type : allowed_skills) {
        // Check skill preconditions based on robot position and skill type
        if (!checkSkillPreconditions(skill_type, robot_at_approach_position, state, current_task)) {
            log("Skill preconditions not met for " + skillTypeToString(skill_type), LogLevel::DEBUG);
            continue;
        }
        
        // Handle different skill types with specific logic
        if (skill_type == Skill::Type::PickAndPlace) {
            // For meta skills, generate and validate complete skill sequence
            auto meta_skill = generateMetaSkill(skill_type, state, current_task, assigned_robot_id);
            if (meta_skill) {
                feasible_skills.push_back(meta_skill);
                log("Added feasible meta skill: PickAndPlace for robot " + robot_name, LogLevel::INFO);
            }
        } else {
            // For atomic skills, create with proper validation
            auto atomic_skill = generateAtomicSkill(skill_type, state, current_task, assigned_robot_id);
            if (atomic_skill) {
                feasible_skills.push_back(atomic_skill);
                log("Added feasible atomic skill: " + skillTypeToString(skill_type) + " for robot " + robot_name, LogLevel::INFO);
            }
        }
    }
    
    // If no skills are feasible from the allowed list AND robot is not at approach position,
    // add a Transit skill to move robot to approach position
    if (feasible_skills.empty() && !robot_at_approach_position) {
        log("No allowed skills feasible and robot not at approach - adding Transit skill", LogLevel::INFO);
        auto transit_skill = generateAtomicSkill(Skill::Type::Transit, state, current_task, assigned_robot_id);
        if (transit_skill) {
            feasible_skills.push_back(transit_skill);
            log("Added Transit skill to move robot to approach position", LogLevel::INFO);
        }
    }
    
    // Additional check: if we still have no feasible skills, something is wrong
    if (feasible_skills.empty()) {
        log("ERROR: No feasible skills found for task " + std::to_string(assembled_steps) + 
            " - this should not happen!", LogLevel::ERROR);
        log("  Robot at approach: " + std::string(robot_at_approach_position ? "YES" : "NO"), LogLevel::ERROR);
        log("  Allowed skills count: " + std::to_string(allowed_skills.size()), LogLevel::ERROR);
        
        // Force add the first allowed skill for debugging
        if (!allowed_skills.empty()) {
            log("Force-adding first allowed skill for debugging", LogLevel::WARN);
            auto debug_skill = generateAtomicSkill(allowed_skills[0], state, current_task, assigned_robot_id);
            if (debug_skill) {
                feasible_skills.push_back(debug_skill);
            }
        }
    }
    
    log("Found " + std::to_string(feasible_skills.size()) + " feasible skills for task " + 
        std::to_string(assembled_steps), LogLevel::INFO);
    
    return feasible_skills;
}

/**
 * @brief Get the next state given a skill.
 * @param state The current state.
 * @param gs The skill to apply.
 * @param next_state The resulting next state.
 * @return True if successful.
 */
bool MagBlockSkillGraph::get_next_state(const State& state, SkillPtr gs, State &next_state, double &cost) {
    // Copy current state
    next_state = state;

    int executing_robot_id = -1;
    std::string executing_robot_name;

    auto atomic_skill = std::dynamic_pointer_cast<AtomicSkill>(gs);
    auto meta_skill = std::dynamic_pointer_cast<MetaSkill>(gs);

    if (atomic_skill && atomic_skill->robot) {
        executing_robot_name = atomic_skill->robot->robot_name;
        executing_robot_id = getRobotIdFromName(executing_robot_name);
    } else if (meta_skill && !meta_skill->robots.empty()) {
        executing_robot_name = meta_skill->robots[0]->robot_name; // Use first robot in meta skill
        executing_robot_id = getRobotIdFromName(executing_robot_name);
    } else {
        // Try to get from executor's post-condition
        if (atomic_skill && atomic_skill->executor && atomic_skill->executor->post_condition) {
            executing_robot_id = atomic_skill->executor->post_condition->constraints_json.get("robot_id", -1).asInt();
            executing_robot_name = getRobotNameFromId(executing_robot_id);
        }
    }

    // Ensure robot states are initialized
    if (next_state.robot_states.empty()) {
        log("Initializing robot states for state tracking", LogLevel::INFO);
        for (int i = 0; i < robots.size(); i++) {
            RobotState robot_state;
            robot_state.robot_id = i;
            robot_state.robot_name = robots[i]->robot_name;
            robot_state.attributes["at_approach_position"] = false;
            
            // Initialize joint values to current robot state or default home position
            robot_state.joint_values = {0.0, 0.0, 0.0, 2.5299, 0.0, 0.6120, 1.57};
            
            next_state.robot_states.push_back(robot_state);
        }
    }
    
    // Update robot joint states with current values from backend if available -- ADDED WITH EMPTY SEED FIX
    if (executing_robot_id >= 0 && executing_robot_id < next_state.robot_states.size() && env_->backend_) {
        auto moveit_instance = std::dynamic_pointer_cast<MoveitInstance>(env_->backend_);
        if (moveit_instance) {
            std::vector<double> current_joints;
            if (moveit_instance->getCurrentJointValues(executing_robot_name, current_joints)) {
                next_state.robot_states[executing_robot_id].joint_values = current_joints;
                log("Updated robot " + executing_robot_name + " joint values from MoveIt backend", LogLevel::DEBUG);
            } else {
                log("Failed to get current joint values for " + executing_robot_name + ", keeping previous values", LogLevel::WARN);
            }
        }
    }
    
    // Handle different skill types differently for state updates
    if (gs->type == Skill::Type::Transit) {
        // For Transit skills, DO NOT increment assembled_steps
        // Transit is a positioning skill that prepares for the actual task
        log("Transit skill completed - robot is now at approach position", LogLevel::INFO);
        
        if (executing_robot_id >= 0 && executing_robot_id < next_state.robot_states.size()) {
            // Set robot at approach position
            next_state.robot_states[executing_robot_id].attributes["at_approach_position"] = true;
        }
        
        // Don't increment assembled_steps for transit - it's preparatory
        // next_state.assembled_steps = state.assembled_steps; // Keep same task step
        log("Transit complete - staying on task step " + std::to_string(next_state.assembled_steps) + " for actual operation", LogLevel::INFO);
        
    } else {
        // For actual task skills (Pick, Place, PickAndPlace), increment assembled_steps
        next_state.assembled_steps++;
        log("Task skill completed, advanced from step " + std::to_string(state.assembled_steps) + 
            " to " + std::to_string(next_state.assembled_steps), LogLevel::INFO);
            
        // Update object positions in EnvState and sync with MoveIt backend
        if (gs->type == Skill::Type::Pick || gs->type == Skill::Type::PlaceTop || 
            gs->type == Skill::Type::PlaceBottom || gs->type == Skill::Type::PickAndPlace) {
            
            // Get target position from executor's post-condition
            TaskParamPtr post_condition = nullptr;
            if (atomic_skill && atomic_skill->executor) {
                post_condition = atomic_skill->executor->post_condition;
            } else if (meta_skill && meta_skill->executor) {
                post_condition = meta_skill->executor->post_condition;
            }
            
            if (post_condition) {
                const Json::Value& constraints = post_condition->constraints_json;
                
                // Find the object being manipulated (use first object for now, can be enhanced)
                if (!next_state.env_state.objects.empty()) {
                    ObjPtr target_object = nullptr;
                    std::string target_name = "b" + std::to_string(next_state.assembled_steps);
                    std::cout << "TARGET NAME: " << target_name << std::endl;

                    for (const auto& obj : next_state.env_state.objects) {
                        if (obj->name == target_name) {
                            target_object = obj;
                            break;
                        }
                    }
                    std::cout << "TARGET OBJECT: " << target_object->name << std::endl;
                    std::string block_name = constraints["block_name"].asString();
                    std::cout << "CONSTRAINTS: " << constraints.toStyledString() << std::endl;
                    std::cout << "ASSEMBLED STEPS: " << next_state.assembled_steps << std::endl;
                    std::cout << "BLOCK NAME: " << block_name << std::endl;
                    for (const auto& obj : next_state.env_state.objects) {
                        std::cout << "OBJECT NAME: " << obj->name << std::endl;
                    }

                    //ObjPtr target_object = next_state.env_state.objects[state.assembled_steps];
                    std::cout << "TARGET OBJECT NAME: " << target_object->name << std::endl;
                    
                    // For Pick skills, object stays in current position (robot picks it up)
                    if (gs->type == Skill::Type::Pick) {
                        log("Pick skill - object " + target_object->name + " picked up by " + executing_robot_name, LogLevel::INFO);
                        // Object position doesn't change in world frame when picked
                    }
                    // For Place skills or PickAndPlace, move object to target position
                    else if (gs->type == Skill::Type::PlaceTop || gs->type == Skill::Type::PlaceBottom || 
                             gs->type == Skill::Type::PickAndPlace) {
                        
                        // Get target coordinates from constraints
                        if (constraints.isMember("x") && constraints.isMember("y") && constraints.isMember("z")) {
                            double target_x = constraints["x"].asDouble();
                            double target_y = constraints["y"].asDouble(); 
                            double target_z = constraints["z"].asDouble();
                            
                            // Transform from skillgraph coordinates to world frame
                            double world_x, world_y, world_z, rx_deg, ry_deg, rz_deg;
                            transformSkillgraphToRobot(executing_robot_name, target_x, target_y, target_z,
                                                     world_x, world_y, world_z, rx_deg, ry_deg, rz_deg);
                            
                            // Transform to world coordinates using MoveIt backend
                            geometry_msgs::msg::Pose robot_pose, world_pose;
                            robot_pose.position.x = world_x;
                            robot_pose.position.y = world_y;
                            robot_pose.position.z = world_z;
                            robot_pose.orientation.x = 0.0;
                            robot_pose.orientation.y = 0.0;
                            robot_pose.orientation.z = 0.0;
                            robot_pose.orientation.w = 1.0;
                            
                            auto moveit_backend = std::dynamic_pointer_cast<MoveitInstance>(env_->backend_);
                            if (moveit_backend) {
                                moveit_backend->transformRobotToWorld(executing_robot_name, robot_pose, world_pose);
                                
                                // Update object position in EnvState
                                target_object->x = world_pose.position.x;
                                target_object->y = world_pose.position.y;
                                target_object->z = world_pose.position.z - 0.159; // Adjust for block height
                                target_object->qx = world_pose.orientation.x;
                                target_object->qy = world_pose.orientation.y;
                                target_object->qz = world_pose.orientation.z;
                                target_object->qw = world_pose.orientation.w;
                                
                                // Sync with MoveIt backend using moveObject
                                moveit_backend->moveObject(*target_object);
                                log("Moved object " + target_object->name + " to position (" + 
                                    std::to_string(world_pose.position.x) + "," + 
                                    std::to_string(world_pose.position.y) + "," + 
                                    std::to_string(world_pose.position.z - 0.159) + ")", LogLevel::INFO);
                                
                                // Update the environment state in the backend
                                env_->setState(next_state.env_state);
                                moveit_backend->updateScene();
                            } else {
                                log("Failed to cast backend to MoveitInstance for object movement", LogLevel::ERROR);
                            }
                        } else {
                            log("Missing target coordinates in constraints for place operation", LogLevel::WARN);
                        }
                    }
                }
            }
        }
            
        // Reset approach position status for next task
        if (executing_robot_id >= 0 && executing_robot_id < next_state.robot_states.size()) {
            next_state.robot_states[executing_robot_id].attributes["at_approach_position"] = false;
        }
    }
    
    return true;
}

/**
 * @brief Check if a skill is feasible in the given state.
 * @param state The current state.
 * @param skill_config JSON configuration for the skill.
 * @param gs Output parameter for the feasible skill.
 * @return True if the skill is feasible.
 */
bool MagBlockSkillGraph::is_feasible(const State&state, Json::Value &skill_config, SkillPtr &gs) {
    std::string skillname = skill_config["type"].asString();
    Skill::Type skill_type = Skill::from_string(skillname);
    
    if (skill_type == Skill::Type::PickAndPlace) {
        // For PickAndPlace skills, create meta skill
        auto base_skill = std::dynamic_pointer_cast<skillgraph::MetaSkill>(get_skill(skillname));
        skillgraph::MetaSkillPtr gs_meta = std::make_shared<skillgraph::MetaSkill>(*base_skill);
        gs = gs_meta;
        
        // Set robot for the skill
        int rid = skill_config.get("robot_id", 2).asInt();
        skillgraph::RobotPtr robot = robots[rid];
        gs_meta->set_robot({robot});
        
        // Set target object
        for (auto& obj : state.env_state.objects) {
            gs_meta->set_object(obj);
            break; // Use first available object for now
        }
        
        // Create executor for the meta skill
        auto meta_executor = std::make_shared<skillgraph::MetaSkillExecutor>(
            gs_meta->type, skillgraph::MetaSkill::ComposeType::Temporal, gs_meta->atomic_skills);
        
        // Set post-condition from skill config
        skillgraph::TaskParamPtr post_condition = std::make_shared<skillgraph::TaskParam>();
        post_condition->constraints_json = skill_config["target_location"];
        
        meta_executor->set_post_condition(post_condition);
        
        // Create atomic executors for each atomic skill in the meta skill
        State end_state_i = state;
        bool skill_feasible = true;
        
        for (int i = 0; i < gs_meta->atomic_skills.size(); i++) {
            auto atomic_skill = gs_meta->atomic_skills[i];
            auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(atomic_skill->type, env_->backend_);
            atomic_skill->executor = atomic_executor;
            atomic_executor->set_post_condition(post_condition);
            meta_executor->add_atomic_executor(atomic_executor);
            
            // Generate pose for this atomic skill
            skillgraph::TaskParamPtr task_param = atomic_executor->post_condition;
            task_param->target_state = end_state_i;
            
            // Generate poses based on skill type
            if (atomic_skill->type == Skill::Type::Transit) {
                // For transit skills, generate approach poses based on sequence position
                // Transit skills inherit constraints from the meta skill but may modify target positions
                
                log("Generating transit pose for skill " + std::to_string(i) + " in sequence", LogLevel::DEBUG);
                
                // Get robot and object from the meta skill
                auto skill_robot = atomic_skill->robot ? atomic_skill->robot : robot;
                auto skill_object = gs_meta->get_object();
                
                // Set up transit constraints based on the overall task
                Json::Value transit_constraints = task_param->constraints_json;
                
                // For transit skills, we need to determine appropriate waypoint positions
                // This will be refined during execution based on the specific transit context
                if (!transit_constraints.isMember("speed_scale")) {
                    transit_constraints["speed_scale"] = 0.5;  // Default speed
                }
                
                // Generate grasp pose using the generator (this will set up basic robot state)
                auto generator = std::make_shared<MagBlockGraspGenerator>(env_->backend_, magblock_config_, skill_robot, skill_object);
                if (!generator->generate(transit_constraints, atomic_skill->type, i, task_param->target_state)) {
                    log("Failed to generate transit pose for skill " + std::to_string(i) + " " + atomic_skill->to_string(), LogLevel::WARN);
                    // For transit skills, this is not necessarily a failure - continue with default setup
                }
                
                // Update constraints for transit execution
                task_param->constraints_json = transit_constraints;
                
            } else {
                // For Pick and Place skills, use existing pose generation
                // Get robot and object from the meta skill
                auto skill_robot = atomic_skill->robot ? atomic_skill->robot : robot;
                auto skill_object = gs_meta->get_object();
                
                auto generator = std::make_shared<MagBlockGraspGenerator>(env_->backend_, magblock_config_, skill_robot, skill_object);
                if (!generator->generate(task_param->constraints_json, atomic_skill->type, i, task_param->target_state)) {
                    log("Failed to generate grasp pose for skill " + std::to_string(i) + " " + atomic_skill->to_string(), LogLevel::ERROR);
                    skill_feasible = false;
                    break;
                }
            }
            
            // Update state for next skill
            end_state_i = task_param->target_state;
        }
        
        gs_meta->set_executor(meta_executor);
        
        return skill_feasible;
    }
    
    // For other atomic skills (Pick, Place, Transit), create atomic skill
    auto atomic_skill = std::make_shared<AtomicSkill>(skillname);
    gs = atomic_skill;
    
    // Create and set executor with proper post condition
    auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(atomic_skill->type, env_->backend_);
    
    // Set post-condition from skill config
    skillgraph::TaskParamPtr post_condition = std::make_shared<skillgraph::TaskParam>();
    post_condition->constraints_json = skill_config;
    
    atomic_executor->set_post_condition(post_condition);
    atomic_skill->executor = atomic_executor;
    
    return true;
}


/**
 * @brief Load assembly task from JSON file.
 */
bool MagBlockSkillGraph::loadAssemblyTask(const std::string& task_file, const std::string& env_setup_file) {
    try {
        // Load assembly tasks
        std::ifstream task_stream(task_file);
        if (!task_stream.is_open()) {
            log("Failed to open task file: " + task_file, LogLevel::ERROR);
            return false;
        }
        task_stream >> assembly_tasks_;
        task_stream.close();
        
        // Load environment setup
        std::ifstream env_stream(env_setup_file);
        if (!env_stream.is_open()) {
            log("Failed to open env setup file: " + env_setup_file, LogLevel::ERROR);
            return false;
        }
        env_stream >> env_setup_;
        env_stream.close();
        
        // Create assembly sequence
        assembly_seq_ = std::make_shared<MagBlockAssemblySeq>(task_file);
        task_seq_ = assembly_seq_;  // For compatibility
        
        log("Successfully loaded magblock assembly task and environment", LogLevel::INFO);
        return true;
    } catch (const std::exception& e) {
        log("Exception loading assembly task: " + std::string(e.what()), LogLevel::ERROR);
        return false;
    }
}

/**
 * @brief Get the assembly sequence for trajectory planning.
 */
std::shared_ptr<MagBlockAssemblySeq> MagBlockSkillGraph::getAssemblySequence() const {
    return assembly_seq_;
}

/**
 * @brief Set the assembly sequence.
 */
void MagBlockSkillGraph::setAssemblySequence(std::shared_ptr<MagBlockAssemblySeq> assembly_seq) {
    assembly_seq_ = assembly_seq;
    task_seq_ = assembly_seq;  // For compatibility
}

/**
 * @brief Set environment configuration.
 */
void MagBlockSkillGraph::setEnvironmentConfig(const Json::Value& env_config) {
    env_setup_ = env_config;
}

/**
 * @brief Transform block coordinates to robot frame - renamed for clarity.
 */
bool MagBlockSkillGraph::transformBlockToRobot(const std::string& robot_name, 
                                              double x_blocks, double y_blocks, double z_blocks,
                                              double& x_robot, double& y_robot, double& z_robot,
                                              double& rx_deg, double& ry_deg, double& rz_deg) {
    transformSkillgraphToRobot(robot_name, x_blocks, y_blocks, z_blocks,
                              x_robot, y_robot, z_robot, rx_deg, ry_deg, rz_deg);
    return true;
}

/**
 * @brief Get pick pose for a task from skillgraph logic.
 */
bool MagBlockSkillGraph::getPickPose(TaskPtr task, double& x, double& y, double& z) {
    // Implementation: Get storage/source location for the block
    // For now, return false to use default table pick location
    // This can be enhanced with actual storage location logic
    return false;
}

/**
 * @brief Check if robot is at the correct approach position for the target.
 */
bool MagBlockSkillGraph::isRobotAtApproachPosition(const RobotState& robot_state, 
                                                  double target_x, double target_y, double target_z,
                                                  const std::string& robot_name) {
    // Check if robot has a position attribute indicating it completed transit
    auto attr_it = robot_state.attributes.find("at_approach_position");
    if (attr_it != robot_state.attributes.end()) {
        try {
            bool at_approach = std::any_cast<bool>(attr_it->second);
            return at_approach;
        } catch (const std::bad_any_cast& e) {
            log("Failed to cast approach position attribute", LogLevel::WARN);
        }
    }
    
    // For now, return false initially to force transit
    return false;
}

/**
 * @brief Check preconditions for executing a specific skill type.
 */
bool MagBlockSkillGraph::checkSkillPreconditions(Skill::Type skill_type, bool robot_at_approach, 
                                                 const State& state, TaskPtr current_task) {
    switch (skill_type) {
        case Skill::Type::Pick:
        case Skill::Type::PickAndPlace:
            // Pick operations require robot to be at approach position
            if (!robot_at_approach) {
                return false;
            }
            return true;
            
        case Skill::Type::PlaceTop:
        case Skill::Type::PlaceBottom:
            // Place operations require robot to be holding an object and at approach position
            if (!robot_at_approach) {
                return false;
            }
            // TODO: Add check for robot holding object
            return true;
            
        case Skill::Type::Transit:
            // Transit is feasible when:
            // 1. Robot is not at correct position (needs to move)
            // 2. OR it's explicitly allowed in the task (part of planned sequence)
            return true; // Always allow transit skills - let execution decide if movement is needed
            
        default:
            return true;
    }
}

/**
 * @brief Convert skill type enum to string.
 */
std::string MagBlockSkillGraph::skillTypeToString(Skill::Type skill_type) {
    switch(skill_type) {
        case Skill::Type::Pick: return "Pick";
        case Skill::Type::PlaceTop: return "PlaceTop";
        case Skill::Type::PlaceBottom: return "PlaceBottom";
        case Skill::Type::Transit: return "Transit";
        case Skill::Type::PickAndPlace: return "PickAndPlace";
        default: return "Unknown";
    }
}

/**
 * @brief Generate a meta skill with proper validation.
 */
SkillPtr MagBlockSkillGraph::generateMetaSkill(Skill::Type skill_type, const State& state, 
                                              TaskPtr current_task, int robot_id) {
    // Get base skill from skill map
    auto base_skill = skill_map_[skill_type];
    if (!base_skill) {
        log("Base skill not found in skill_map_ for type: " + std::to_string(static_cast<int>(skill_type)), LogLevel::ERROR);
        return nullptr;
    }
    
    // Create meta skill copy
    auto meta_skill_base = std::dynamic_pointer_cast<MetaSkill>(base_skill);
    if (!meta_skill_base) {
        log("Failed to cast base skill to MetaSkill", LogLevel::ERROR);
        return nullptr;
    }
    
    auto meta_skill = std::make_shared<MetaSkill>(*meta_skill_base);
    
    // Set robot
    if (robot_id < robots.size()) {
        meta_skill->set_robot({robots[robot_id]});
    }
    
    // Set object (for now, use a simple object reference)
    // TODO: This should come from the task constraints
    // auto dummy_object = std::make_shared<Object>("magblock_" + std::to_string(state.assembled_steps), 
    //                                             "world", Object::State::Static, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    // meta_skill->set_object(dummy_object);
    
    // Create meta executor
    auto meta_executor = std::make_shared<MetaSkillExecutor>(
        meta_skill->type, MetaSkill::ComposeType::Temporal, meta_skill->atomic_skills);
    
    // Set post-condition
    meta_executor->set_post_condition(current_task->post_condition);
    
    // Create and validate atomic skill executors
    State end_state_i = state;
    bool skill_feasible = true;
    
    for (int i = 0; i < meta_skill->atomic_skills.size(); i++) {
        auto atomic_skill = meta_skill->atomic_skills[i];
        auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(atomic_skill->type, env_->backend_);
        atomic_skill->executor = atomic_executor;
        atomic_executor->set_post_condition(current_task->post_condition);
        meta_executor->add_atomic_executor(atomic_executor);
        
        // Generate pose and validate feasibility for this atomic skill
        TaskParamPtr task_param = atomic_executor->post_condition;
        task_param->target_state = end_state_i;
        
        // For transit skills, validate trajectory planning
        if (atomic_skill->type == Skill::Type::Transit) {
            log("Validating transit skill " + std::to_string(i) + " in meta sequence", LogLevel::DEBUG);
            // Transit validation logic here
        } else {
            // Validate pick/place skills
            log("Validating " + skillTypeToString(atomic_skill->type) + " skill " + std::to_string(i), LogLevel::DEBUG);
        }
        
        // Update state for next skill
        end_state_i = task_param->target_state;
    }
    
    if (skill_feasible) {
        meta_skill->set_executor(meta_executor);
        return meta_skill;
    }
    
    return nullptr;
}

/**
 * @brief Generate an atomic skill with proper validation.
 */
SkillPtr MagBlockSkillGraph::generateAtomicSkill(Skill::Type skill_type, const State& state, 
                                                TaskPtr current_task, int robot_id) {
    // Create atomic skill
    auto atomic_skill = std::make_shared<AtomicSkill>(skillTypeToString(skill_type));
    atomic_skill->type = skill_type;
    
    // Set robot
    if (robot_id < robots.size()) {
        atomic_skill->robot = robots[robot_id];
    }
    
    // Set object if needed
    if (skill_type == Skill::Type::Pick || skill_type == Skill::Type::PlaceTop || skill_type == Skill::Type::PlaceBottom) {
        auto dummy_object = std::make_shared<Object>("magblock_" + std::to_string(state.assembled_steps), 
                                                    "world", Object::State::Static, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        atomic_skill->object = dummy_object;
    }
    
    // Create executor
    auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(skill_type, env_->backend_);
    
    // Create enhanced post-condition for transit skills
    if (skill_type == Skill::Type::Transit) {
        // Clone the original post-condition
        auto enhanced_post_condition = std::make_shared<TaskParam>();
        enhanced_post_condition->constraints_json = current_task->post_condition->constraints_json;
        enhanced_post_condition->target_state = current_task->post_condition->target_state;
        enhanced_post_condition->allowed_skill_type = current_task->post_condition->allowed_skill_type;
        
        // Calculate and add the exact approach pose that matches pick/place operations
        std::string robot_name = atomic_skill->robot ? atomic_skill->robot->robot_name : get_robot_names()[robot_id];
        geometry_msgs::msg::Pose approach_pose;
        
        // Use the magblock_algorithms function to calculate proper approach pose
        if (calculateApproachPose(env_->backend_, current_task->post_condition->constraints_json, robot_name, approach_pose)) {
            // Add the calculated approach pose to constraints
            enhanced_post_condition->constraints_json["target_x"] = approach_pose.position.x;
            enhanced_post_condition->constraints_json["target_y"] = approach_pose.position.y;
            enhanced_post_condition->constraints_json["target_z"] = approach_pose.position.z;
            enhanced_post_condition->constraints_json["orient_x"] = approach_pose.orientation.x;
            enhanced_post_condition->constraints_json["orient_y"] = approach_pose.orientation.y;
            enhanced_post_condition->constraints_json["orient_z"] = approach_pose.orientation.z;
            enhanced_post_condition->constraints_json["orient_w"] = approach_pose.orientation.w;
            
            // Add robot information to constraints for proper execution
            enhanced_post_condition->constraints_json["robot_name"] = robot_name;
            enhanced_post_condition->constraints_json["robot_id"] = robot_id;
            
            log("Enhanced transit skill with calculated approach pose for " + robot_name + 
                " at position (" + std::to_string(approach_pose.position.x) + "," + 
                std::to_string(approach_pose.position.y) + "," + std::to_string(approach_pose.position.z) + ")", LogLevel::INFO);
        } else {
            log("Failed to calculate approach pose for transit, using original constraints", LogLevel::WARN);
            // Still add robot information even if approach calculation failed
            enhanced_post_condition->constraints_json["robot_name"] = robot_name;
            enhanced_post_condition->constraints_json["robot_id"] = robot_id;
        }
        
        atomic_executor->set_post_condition(enhanced_post_condition);
    } else {
        // For non-transit skills, also add robot information to constraints
        auto enhanced_post_condition = std::make_shared<TaskParam>();
        enhanced_post_condition->constraints_json = current_task->post_condition->constraints_json;
        enhanced_post_condition->target_state = current_task->post_condition->target_state;
        enhanced_post_condition->allowed_skill_type = current_task->post_condition->allowed_skill_type;
        
        std::string robot_name = atomic_skill->robot ? atomic_skill->robot->robot_name : get_robot_names()[robot_id];
        enhanced_post_condition->constraints_json["robot_name"] = robot_name;
        enhanced_post_condition->constraints_json["robot_id"] = robot_id;
        
        atomic_executor->set_post_condition(enhanced_post_condition);
    }
    
    atomic_skill->executor = atomic_executor;
    
    return atomic_skill;
}

} // namespace skillgraph