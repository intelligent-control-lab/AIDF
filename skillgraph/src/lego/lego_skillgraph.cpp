#include "lego/lego_skillgraph.hpp"
#include "Utils/Logger.hpp"
#include "moveit_backend.hpp"

namespace skillgraph {

LegoSkillGraph::LegoSkillGraph(const std::string &config_file) : SkillGraph(config_file)
{
}

void LegoSkillGraph::parse_env(const Json::Value &root_config) {
    // first, use the base class implementation
    SkillGraph::parse_env(root_config);
 
   
    const Json::Value& env_config = root_config["environment"];
    const Json::Value& backend_config = env_config["backend"];
    std::string moveitConfigPkg = backend_config["moveitConfigPkg"].asString();
    
    std::string backend = backend_config["type"].asString();
    if (backend != "moveit") {
        throw std::runtime_error("Backend must be moveit for Lego assembly! " + backend);
    }

    // create the moveit instance backend
    auto plan_instance_ = std::make_shared<MoveitInstance>(backend_config["moveit_group_name"].asString(), 
                            moveitConfigPkg);

    // set the number of robots and their names
    plan_instance_->setNumberOfRobots(num_robots_);
    std::vector<std::string> robot_names;
    for (int i = 0; i < num_robots_; i++) {
        plan_instance_->setRobotDOF(i, robots[i]->getDOF()+1); // the moveit backend adds a dummy gripper dof
    }
    plan_instance_->setRobotNames(get_robot_names());
    plan_instance_->setVmax(backend_config["l1_vmax"].asDouble());
    
    env_->setBackend(plan_instance_);
}

void LegoSkillGraph::parse_tasks(const Json::Value &root_config) {
    // Open the JSON file
    std::cout << "Parsing Lego Task" << std::endl;
    if (root_config.isMember("environment") && root_config.isMember("tasks")) {
        std::cout << "Parsing Lego Task" << std::endl;
        const Json::Value& env_config = root_config["environment"];
        std::string env_name = env_config["name"].asString();
        std::string env_type = env_config["type"].asString();
        if (env_type != "Lego") {
            throw std::runtime_error("Environment type is not Lego! " + env_type);
        }

        std::string root_pwd = env_config["rootPwd"].asString();
        auto calib_config = env_config["calibration"];

        // read calibration
        std::string r1_DH_fname = root_pwd + calib_config["r1"]["DH"].asString();
        std::string r1_DH_tool_fname = root_pwd + calib_config["r1"]["DH_tool"].asString();
        std::string r1_DH_tool_assemble_fname = root_pwd + calib_config["r1"]["DH_tool_assemble"].asString();
        std::string r1_DH_tool_disassemble_fname = root_pwd + calib_config["r1"]["DH_tool_disassemble"].asString();
        std::string r1_DH_tool_alt_fname = root_pwd + calib_config["r1"]["DH_tool_alt"].asString();
        std::string r1_DH_tool_alt_assemble_fname = root_pwd + calib_config["r1"]["DH_tool_alt_assemble"].asString();
        std::string r1_DH_tool_handover_assemble_fname = root_pwd + calib_config["r1"]["DH_tool_handover_assemble"].asString();
        std::string r1_robot_base_fname = root_pwd + calib_config["r1"]["robot_base"].asString();
        
        std::string r2_DH_fname = root_pwd + calib_config["r2"]["DH"].asString();
        std::string r2_DH_tool_fname = root_pwd + calib_config["r2"]["DH_tool"].asString();
        std::string r2_DH_tool_assemble_fname = root_pwd + calib_config["r2"]["DH_tool_assemble"].asString();
        std::string r2_DH_tool_disassemble_fname = root_pwd + calib_config["r2"]["DH_tool_disassemble"].asString();
        std::string r2_DH_tool_alt_fname = root_pwd + calib_config["r2"]["DH_tool_alt"].asString();
        std::string r2_DH_tool_alt_assemble_fname = root_pwd + calib_config["r2"]["DH_tool_alt_assemble"].asString();
        std::string r2_DH_tool_handover_assemble_fname = root_pwd + calib_config["r2"]["DH_tool_handover_assemble"].asString();
        std::string r2_robot_base_fname = root_pwd + calib_config["r2"]["robot_base"].asString();


        std::string plate_calibration_fname = root_pwd + calib_config["plate"].asString();
        std::string env_setup_fname = root_pwd + env_config["object_library"].asString();
        std::string lego_lib_fname = root_pwd + env_config["lego_library"].asString();
        std::string world_base_fname = root_pwd + calib_config["world"].asString();
    
        // read task file
        const Json::Value &task_config = root_config["tasks"];
        std::string task_name = task_config["name"].asString();
        std::string task_fname = root_pwd + task_config["assembly_seq"].asString();
        
        std::ifstream task_file(task_fname, std::ifstream::binary);
        task_file >> task_json_;

        bool assemble = task_config["Start_with_Assemble"].asBool(); 

        // gazebo client for updating the simulation
        nh_ = std::make_shared<ros::NodeHandle>();
        set_state_client_ = nh_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        // initialize the lego library
        lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr_->setup(env_setup_fname, lego_lib_fname, plate_calibration_fname, assemble, task_json_, world_base_fname,
                    r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_assemble_fname, 
                    r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, 
                    r1_DH_tool_handover_assemble_fname, r1_robot_base_fname, 
                    r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_assemble_fname,
                    r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, 
                    r2_DH_tool_handover_assemble_fname, r2_robot_base_fname,
                    set_state_client_);
    
        // initialize the task sequence
        task_seq_ = std::make_shared<skillgraph::LegoAssemblySeq>(lego_ptr_, task_fname);

        // add the existing lego objects to the initial state of the environment
        EnvState init_state;

        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        for (const auto & name : brick_names) {
            if (name.find("station") != std::string::npos) {
                continue;
            }
            auto lego_brick = std::make_shared<LegoBrick>(getLegoStart(name));
            lego_brick->in_storage = true; // Explicitly set in_storage flag to true
            env_->backend_->addMoveableObject(*lego_brick);
            env_->backend_->updateScene();
            init_state.objects.push_back(lego_brick);
        }
        
        // add the table to the initial state
        ObjPtr table = std::make_shared<Object>();
        table->name = "table";
        table->state = Object::State::Static;
        table->parent_link = "world";
        table->shape = Object::Shape::Box;
        
        lego_ptr_->get_table_size(table->length, table->width, table->height);
        geometry_msgs::Pose box_pose = lego_ptr_->get_table_pose();
        table->x = box_pose.position.x;
        table->y = box_pose.position.y;
        table->z = box_pose.position.z - table->height/2;
        table->qx = box_pose.orientation.x;
        table->qy = box_pose.orientation.y;
        table->qz = box_pose.orientation.z;
        table->qw = box_pose.orientation.w;

        env_->backend_->addMoveableObject(*table);
        env_->backend_->updateScene();
        std::dynamic_pointer_cast<MoveitInstance>(env_->backend_)->setObjectColor("table", 0.0, 0.0, 1.0, 0.0);
        init_state.objects.push_back(table);

        // set the initial state to environment
        env_->setState(init_state);
        initial_state_.env_state = init_state;
    }

}

LegoBrick LegoSkillGraph::getLegoStart(const std::string &brick_name) {
    LegoBrick obj;
    
    // define the object
    obj.name = brick_name;
    obj.state = Object::State::Static;
    obj.parent_link = "world";
    obj.shape = Object::Shape::Box;

    // get the starting pose and size
    geometry_msgs::Pose box_pose;
    lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
    box_pose = lego_ptr_->get_init_brick_pose(brick_name);
    
    obj.brick_id = lego_ptr_->get_brick_type(brick_name);
    obj.x = box_pose.position.x;
    obj.y = box_pose.position.y;
    obj.z = box_pose.position.z - obj.height/2;
    obj.qx = box_pose.orientation.x;
    obj.qy = box_pose.orientation.y;
    obj.qz = box_pose.orientation.z;
    obj.qw = box_pose.orientation.w; 

    return obj;
}


LegoBrick LegoSkillGraph::getLegoTarget(int task_idx) {
    auto cur_graph_node =  task_json_[std::to_string(task_idx)];
    std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asString());

    Eigen::Matrix4d brick_pose_mtx;
    lego_ptr_->calc_bric_asssemble_pose(brick_name, cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(),
             cur_graph_node["z"].asInt(), cur_graph_node["ori"].asInt(), brick_pose_mtx);
    
    LegoBrick obj;
    // define the object
    obj.name = brick_name;
    obj.state = Object::State::Static;
    obj.parent_link = "world";
    obj.shape = Object::Shape::Box;
    obj.brick_id = cur_graph_node["brick_id"].asInt();
    obj.in_storage = false;
    lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);

    obj.x = brick_pose_mtx(0, 3);
    obj.y = brick_pose_mtx(1, 3);
    obj.z = brick_pose_mtx(2, 3) - obj.height/2;
    Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
    obj.qx = quat.x();
    obj.qy = quat.y();
    obj.qz = quat.z();
    obj.qw = quat.w();

    return obj;
}

LegoBrick LegoSkillGraph::getLegoHandover(int task_idx, const RobotState &start_pose) {
    auto cur_graph = task_json_[std::to_string(task_idx)];
    std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph["brick_id"].asInt(), cur_graph["brick_seq"].asString());
    int sup_robot = cur_graph["sup_robot_id"].asInt() - 1;
    assert (sup_robot == start_pose.robot_id);
    int brick_id = cur_graph["brick_id"].asInt();
    int press_side = cur_graph["press_side"].asInt();
    int press_offset = cur_graph["press_offset"].asInt();


    LegoBrick obj;
    // define the object
    obj.name = brick_name;
    obj.state = Object::State::Handover;
    obj.parent_link = robots[sup_robot]->end_effector_link;
    obj.shape = Object::Shape::Box;
    obj.brick_id = brick_id;

    // define the object
    Eigen::Matrix4d brick_loc;
    lego_manipulation::math::VectorJd press_joints = Eigen::MatrixXd::Zero(6, 1);
    press_joints << start_pose.joint_values[0], start_pose.joint_values[1], start_pose.joint_values[2], 
                    start_pose.joint_values[3], start_pose.joint_values[4], start_pose.joint_values[5];
    press_joints = press_joints * M_PI / 180.0;
    lego_ptr_->lego_pose_from_press_pose(press_joints, sup_robot, brick_id, 
        press_side, press_offset, brick_loc);
    
    obj.x = brick_loc(0, 3);
    obj.y = brick_loc(1, 3);
    obj.z = brick_loc(2, 3) - obj.height/2;
    Eigen::Quaterniond quat(brick_loc.block<3, 3>(0, 0));
    obj.qx = quat.x();
    obj.qy = quat.y();
    obj.qz = quat.z();
    obj.qw = quat.w();
     
    lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
    return obj;
}

bool LegoSkillGraph::at_target(const State &state) {
    // check if the task sequence has been completed
    return state.assembled_steps >= task_seq_->num_tasks();
}

std::vector<SkillPtr> LegoSkillGraph::feasible_u(const skillgraph::State &state)
{
    std::vector<SkillPtr> feasible_set;
    
    // identify what has been
    const auto &objects = state.env_state.objects;

    // get the next task
    int assembled_steps = state.assembled_steps;
    if (assembled_steps >= task_seq_->num_tasks()) {
        log("All tasks have been completed", LogLevel::INFO);
        return feasible_set;
    }
    TaskPtr task = task_seq_->get_task_at(assembled_steps);

    // get the task constraints
    const std::vector<Skill::Type> &allowed_skills = task->post_condition->allowed_skill_type;
    const Json::Value &constraints = task->post_condition->constraints_json;
    int required_brick_id = constraints["brick_id"].asInt();
    
    //std::cout << "Looking for bricks with ID: " << required_brick_id << std::endl;

    // find all the bricks that can be picked with the correct type
    std::vector<LegoBrickPtr> usable_bricks;
    //std::cout << "Number of objects" << objects.size() << std::endl;
    for (auto obj : objects) {
        // check if this object is a Lego Brick
        if (auto lego_brick = std::dynamic_pointer_cast<LegoBrick>(obj)) {
            int brick_id = lego_brick->brick_id;
            bool in_storage = lego_brick->in_storage;
            //std::cout << "Found brick: " << lego_brick->name << " with ID: " << brick_id << ", in_storage: " << (in_storage ? "true" : "false") << std::endl;
            if (in_storage && brick_id == required_brick_id) {
                usable_bricks.push_back(lego_brick);
                //std::cout << "Added usable brick: " << lego_brick->name << std::endl;
            }
        }
    }

    // 1. generate symbolic meta skills, based on feasible skill type
    // 2. enumerate all robots and objects that can be used
    // 3. set task param for each atomic skills in the meta skill 
    //    (e.g. robot, object, press side, atomic skill type)
    // 4. generrate the robot poses for each atomic skill with a lego grasp pose generator algorithm
    
    for (const auto &skill_type : allowed_skills) {
        //std::cout << "Skill Type: " << skill_type << std::endl;
        auto base_skill = skill_map_[skill_type];
        if (skill_type == Skill::Type::PickAndPlace) {
            for (auto obj : usable_bricks) {
                // this block is symbolically feasible
                for (int ri = 0; ri < robots.size(); ri++) {
                    //make a copy of skill_map_[skill_type];
                    auto skill = std::dynamic_pointer_cast<MetaSkill>(base_skill);
                    MetaSkillPtr gs = std::make_shared<MetaSkill>(*skill);
                    gs->set_robot({robots[ri]}, 0);
                    gs->set_object(obj);
                    auto executor = std::make_shared<MetaSkillExecutor>(gs->type, gs->atomic_skills);
                    gs->set_executor(executor);

                    // set the task parameters
                    executor->set_post_condition(task->post_condition);

                    // call grasp pose generator
                    auto generator = std::make_shared<LegoGraspGenerator>(lego_ptr_, env_->backend_, lego_config_, robots[ri], obj);
                    bool skill_feasible = true;
                    for (int i = 0; i < gs->atomic_skills.size(); i++) {
                        AtomicSkillPtr atomic_skill = std::dynamic_pointer_cast<AtomicSkill>(gs->atomic_skills[i]);
                        SkillExecutorPtr atomic_executor = executor->atomic_executors[i];
                        TaskParamPtr task_param = atomic_executor->post_condition;
                        task_param->target_state = state;
                        if (!generator->generate(task_param->constraints_json, atomic_skill->type, i, task_param->target_state)) {
                            log("Failed to generate grasp pose for skill " + atomic_skill->to_string(), LogLevel::DEBUG);
                            skill_feasible = false;
                            break;
                        }
                    }
                    
                    if (skill_feasible) {
                        feasible_set.push_back(gs);
                    }
                }
            }
        }
        else if (skill_type == Skill::Type::PickAndPlaceWithSupport) {
            for (auto obj : usable_bricks) {
                for (int ri = 0; ri < robots.size(); ri++) {
                    // check if the robot has the capability to support
                    auto skill = std::dynamic_pointer_cast<MetaSkill>(base_skill);
                    MetaSkillPtr gs = std::make_shared<MetaSkill>(*skill);
                    gs->set_robot(robots, ri);
                    gs->set_object(obj);
                    auto executor = std::make_shared<MetaSkillExecutor>(gs->type, gs->atomic_skills);
                    gs->set_executor(executor);

                    // set the task parameters
                    executor->set_post_condition(task->post_condition);

                    // call grasp pose generator
                    auto generator = std::make_shared<LegoGraspGenerator>(lego_ptr_, env_->backend_, lego_config_, robots[ri], obj);
                    bool skill_feasible = true;
                    for (int i = 0; i < gs->atomic_skills.size(); i++) {
                        AtomicSkillPtr atomic_skill = std::dynamic_pointer_cast<AtomicSkill>(gs->atomic_skills[i]);
                        SkillExecutorPtr atomic_executor = executor->atomic_executors[i];
                        TaskParamPtr task_param = atomic_executor->post_condition;
                        task_param->target_state = state;
                        if (!generator->generate(task_param->constraints_json, atomic_skill->type, i, task_param->target_state)) {
                            log("Failed to generate grasp pose for skill " + atomic_skill->to_string(), LogLevel::DEBUG);
                            skill_feasible = false;
                            continue;
                        }
                    }

                    if (skill_feasible) {
                        feasible_set.push_back(gs);
                    }
                }
            }
        }
        else if (skill_type == Skill::Type::PickHandoverAndPlace) {
            for (auto obj : usable_bricks) {
                for (int ri = 0; ri < robots.size(); ri++) {
                    // check if the robot has the capability to handover
                    auto skill = std::dynamic_pointer_cast<MetaSkill>(base_skill);
                    MetaSkillPtr gs = std::make_shared<MetaSkill>(*skill);
                    gs->set_robot(robots, ri);
                    gs->set_object(obj);
                    auto executor = std::make_shared<MetaSkillExecutor>(gs->type, gs->atomic_skills);
                    gs->set_executor(executor);

                    // set the task parameters
                    executor->set_post_condition(task->post_condition);
                    bool skill_feasible = true;

                    // call grasp pose generator
                    auto generator = std::make_shared<LegoGraspGenerator>(lego_ptr_, env_->backend_, lego_config_, robots[ri], obj);
                    for (int i = 0; i < gs->atomic_skills.size(); i++) {
                        AtomicSkillPtr atomic_skill = std::dynamic_pointer_cast<AtomicSkill>(gs->atomic_skills[i]);
                        SkillExecutorPtr atomic_executor = executor->atomic_executors[i];
                        TaskParamPtr task_param = atomic_executor->post_condition;
                        task_param->target_state = state;
                        if (!generator->generate(task_param->constraints_json, atomic_skill->type, i, task_param->target_state)) {
                            log("Failed to generate grasp pose for skill " + atomic_skill->to_string(), LogLevel::DEBUG);
                            skill_feasible = false;
                            continue;
                        }
                    }

                    if (skill_feasible) {
                        feasible_set.push_back(gs);
                    }
                }
            }
        }
    }

    if (feasible_set.empty()) {
        log("No feasible skills found", LogLevel::ERROR);
    }
    
    return feasible_set;
}

bool LegoSkillGraph::get_next_state(const State& state, SkillPtr gs, State &next_state, double &cost) {
    next_state = state;
    // incrase the assembled step
    next_state.assembled_steps = state.assembled_steps + 1;
    // keep the robot state the same for now (at home pose)
    next_state.robot_states = state.robot_states;

    // update env state based on the grounded skill
    auto obj_to_move = gs->get_object();
    if (obj_to_move == nullptr) {
        log("Object is null", LogLevel::ERROR);
        return false;
    }

    if (gs->type == Skill::Type::PickAndPlace || gs->type == Skill::Type::PickAndPlaceWithSupport
        || gs->type == Skill::Type::PickHandoverAndPlace) {
        // get a new object at goal location
        auto obj_moved = std::make_shared<LegoBrick>(getLegoTarget(state.assembled_steps));
        obj_moved->name = obj_to_move->name;
        
        // update the pointer to the object moved to the new object
        auto &existing_objs = next_state.env_state.objects;
        for (int i = 0; i < existing_objs.size(); i++) {
            if (existing_objs[i] == obj_to_move) {
               existing_objs[i] = obj_moved;
            }
        }
        
    }

    cost = 0;

    return true;
}

}