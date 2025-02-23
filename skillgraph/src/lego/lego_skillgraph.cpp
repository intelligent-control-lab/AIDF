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
 
    // create moveit and ROS backend
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "lego_skillgraph");
    nh_ = std::make_shared<ros::NodeHandle>();

    // TODO: setup the moveit backend
    const Json::Value& env_config = root_config["environment"];
    std::string moveitConfigPkg = env_config["moveitConfigPkg"].asString();
    std::string backend = env_config["backend"].asString();
    if (backend != "moveit") {
        throw std::runtime_error("Backend must be moveit for Lego assembly! " + backend);
    }

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
    }

}

std::set<GroundedSkill> LegoSkillGraph::feasible_u(const skillgraph::State &state)
{
    std::set<GroundedSkill> feasible_set;
    
    return feasible_set;
}

}