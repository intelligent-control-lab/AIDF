#include "magblock/magblock_skillgraph.hpp"
#include "Utils/Logger.hpp"
#include "Utils/PathUtils.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {

MagBlockSkillGraph::MagBlockSkillGraph(const std::string &config_file)
    : SkillGraph(config_file) {
}

void MagBlockSkillGraph::parse_env(const Json::Value &root_config) {
    // First use the base class implementation
    SkillGraph::parse_env(root_config);

    const Json::Value& env_config = root_config["environment"];
    const Json::Value& backend_config = env_config["backend"];
    
    // Configure backend
    std::string backend = backend_config["type"].asString();
    if (backend != "moveit") {
        throw std::runtime_error("Only MoveIt backend is supported for magnetic block assembly");
    }

    // Initialize ROS 2 node if needed
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("magblock_skillgraph");
}

bool MagBlockSkillGraph::at_target(const State& state) {
    // Check if all tasks in the sequence have been completed
    return state.assembled_steps >= task_seq_->num_tasks();
}

std::vector<SkillPtr> MagBlockSkillGraph::feasible_u(const skillgraph::State &state) {
    std::vector<SkillPtr> feasible_set;

    // Get the next task
    int assembled_steps = state.assembled_steps;
    if (assembled_steps >= task_seq_->num_tasks()) {
        log("All tasks have been completed", LogLevel::INFO);
        return feasible_set;
    }

    TaskPtr task = task_seq_->get_task_at(assembled_steps);
    const auto& task_constraints = task->post_condition->constraints_json;

    // Get the block that needs to be manipulated
    std::string block_name = task_constraints["block_name"].asString();
    int required_block_id = task_constraints["block_id"].asInt();

    // Find usable blocks in storage
    std::vector<MagBlockPtr> usable_blocks;
    for (const auto& obj : state.env_state.objects) {
        auto mag_block = std::dynamic_pointer_cast<MagBlock>(obj);
        if (!mag_block) continue;

        if (mag_block->in_storage && mag_block->block_id == required_block_id) {
            usable_blocks.push_back(mag_block);
        }
    }

    // Generate feasible skills based on available blocks and robots
    for (const auto& skill_type : task->post_condition->allowed_skill_type) {
        if (skill_type == Skill::Type::PickAndPlace) {
            // For each usable block and robot combination
            for (auto obj : usable_blocks) {
                for (int ri = 0; ri < robots.size(); ri++) {
                    auto base_skill = skill_map_[skill_type];
                    auto skill = std::dynamic_pointer_cast<MetaSkill>(base_skill);
                    MetaSkillPtr gs = std::make_shared<MetaSkill>(*skill);

                    // Configure the skill
                    gs->set_robot({robots[ri]});
                    gs->set_object(obj);

                    // Set up executor
                    auto meta_executor = std::make_shared<MetaSkillExecutor>(
                        gs->type, MetaSkill::ComposeType::Temporal, gs->atomic_skills);
                    gs->set_executor(meta_executor);
                    meta_executor->set_post_condition(task->post_condition);

                    // Generate grasp poses and check feasibility
                    bool skill_feasible = true;
                    State end_state_i = state;

                    for (int i = 0; i < gs->atomic_skills.size() && skill_feasible; i++) {
                        auto atomic_skill = gs->atomic_skills[i];
                        auto atomic_executor = std::make_shared<MagBlockSkillExecutor>(atomic_skill->type);
                        atomic_skill->executor = atomic_executor;
                        meta_executor->add_atomic_executor(atomic_executor);

                        // Set up task parameters for the atomic skill
                        TaskParamPtr task_param = atomic_executor->post_condition;
                        task_param->target_state = end_state_i;

                        // Generate grasp poses
                        auto generator = std::make_shared<MagBlockGraspGenerator>(
                            env_->backend_, MagBlockPolicyCfg(), atomic_skill->robot, obj);
                        
                        if (!generator->generate(task_param->constraints_json, 
                                              atomic_skill->type, 
                                              i, 
                                              task_param->target_state)) {
                            skill_feasible = false;
                            break;
                        }

                        end_state_i = task_param->target_state;
                    }

                    if (skill_feasible) {
                        feasible_set.push_back(gs);
                    }
                }
            }
        }
    }

    return feasible_set;
}

bool MagBlockSkillGraph::get_next_state(const State& state, 
                                      SkillPtr gs, 
                                      State &next_state, 
                                      double &cost) {
    // Increment assembled steps
    next_state = state;
    next_state.assembled_steps++;

    // Update object state based on the skill execution
    auto obj_to_move = gs->get_object();
    if (!obj_to_move) {
        log("No object associated with skill", LogLevel::ERROR);
        return false;
    }

    // Get target position from task parameters
    const auto& constraints = gs->executor->post_condition->constraints_json;
    auto obj_moved = std::make_shared<MagBlock>(*std::dynamic_pointer_cast<MagBlock>(obj_to_move));
    obj_moved->in_storage = false;

    // Update object pose
    obj_moved->x = constraints["x"].asDouble();
    obj_moved->y = constraints["y"].asDouble();
    obj_moved->z = constraints["z"].asDouble();

    // Update the object in the environment state
    auto& existing_objs = next_state.env_state.objects;
    for (size_t i = 0; i < existing_objs.size(); i++) {
        if (existing_objs[i]->name == obj_to_move->name) {
            existing_objs[i] = obj_moved;
            break;
        }
    }

    cost = 1.0; // Simple constant cost for now
    return true;
}

bool MagBlockSkillGraph::is_feasible(const State& state,
                                   Json::Value &skill_config,
                                   SkillPtr &gs) {
    std::string skillname = skill_config["skill"].asString();
    Skill::Type skill_type = Skill::from_string(skillname);

    if (skill_type == Skill::Type::PickAndPlace) {
        // Get base skill
        auto base_skill = std::dynamic_pointer_cast<MetaSkill>(get_skill(skillname));
        MetaSkillPtr gs_meta = std::make_shared<MetaSkill>(*base_skill);
        gs = gs_meta;

        // Set robot
        int rid = skill_config["robot"].asInt();
        RobotPtr robot = robots[rid];
        gs_meta->set_robot({robot});

        // Set object
        std::string object_name = skill_config["object"].asString();
        for (auto& obj : state.env_state.objects) {
            if (obj->name == object_name) {
                gs_meta->set_object(obj);
                break;
            }
        }

        // Set executor
        auto meta_executor = std::make_shared<MetaSkillExecutor>(
            gs_meta->type, MetaSkill::ComposeType::Temporal, gs_meta->atomic_skills);
        gs_meta->set_executor(meta_executor);

        // Set task parameters
        TaskParamPtr post_condition = std::make_shared<TaskParam>();
        post_condition->constraints_json = skill_config["target_location"];

        meta_executor->set_post_condition(post_condition);

        return true;
    }

    return false;
}


bool MagBlockSkillGraph::loadAssemblySequence(const std::string& assembly_file) {
    // Create a MagBlockAssemblySeq and parse the file
    auto assembly_seq = std::make_shared<MagBlockAssemblySeq>();
    if (!assembly_seq->parse_from_json(assembly_file)) {
        log("Failed to load assembly sequence from: " + assembly_file, LogLevel::ERROR);
        return false;
    }
    
    // Set the task sequence (using protected member access)
    task_seq_ = assembly_seq;
    log("Successfully loaded assembly sequence with " + std::to_string(assembly_seq->num_tasks()) + " tasks", LogLevel::INFO);
    return true;
}

State MagBlockSkillGraph::getInitialState() {
    State initial_state;
    initial_state.assembled_steps = 0;
    // Initialize other state components as needed
    return initial_state;
}

bool MagBlockSkillGraph::isGoalState(const State& state) {
    return at_target(state);
}

std::shared_ptr<State> MagBlockSkillGraph::getNextState(const State& current_state, SkillPtr skill) {
    auto next_state = std::make_shared<State>();
    double cost = 0.0;
    
    if (get_next_state(current_state, skill, *next_state, cost)) {
        return next_state;
    }
    
    return nullptr;
}
}
