#include "skillgraph.hpp"

namespace skillgraph {

LegoSkillGraph::LegoSkillGraph(const std::string &config_file) : SkillGraph(config_file)
{
    // Parse task sequence
    init_task_seq(root_config_);
}

void LegoSkillGraph::init_task_seq(const Json::Value &root_config) {
    // Open the JSON file
    if (!root_config.isMember("legoConig")) {
        throw std::runtime_error("Lego Config file not found in config");
    }

    std::string task_name = root_config["task_name"].asString();
    Json::Value legoConfig = root_config["legoConfig"];
    std::string root_pwd = legoConfig["root_pwd"].asString();
    std::string lego_config_name = root_pwd + legoConfig["user_config"].asString();

    std::ifstream config_file(lego_config_name, std::ifstream::binary);
    if (!config_file.is_open()) {
        throw std::runtime_error("Unable to open config file: " + lego_config_name);
    }

    Json::Value config;
    config_file >> config;
    std::string r1_DH_fname = root_pwd + config["r1_DH_fname"].asString();
    std::string r1_DH_tool_fname = root_pwd + config["r1_DH_tool_fname"].asString();
    std::string r1_DH_tool_assemble_fname = root_pwd + config["r1_DH_tool_assemble_fname"].asString();
    std::string r1_DH_tool_disassemble_fname = root_pwd + config["r1_DH_tool_disassemble_fname"].asString();
    std::string r1_DH_tool_alt_fname = root_pwd + config["r1_DH_tool_alt_fname"].asString();
    std::string r1_DH_tool_alt_assemble_fname = root_pwd + config["r1_DH_tool_alt_assemble_fname"].asString();
    std::string r1_DH_tool_handover_assemble_fname = root_pwd + config["r1_DH_tool_handover_assemble_fname"].asString();
    std::string r1_robot_base_fname = root_pwd + config["Robot1_Base_fname"].asString();
    std::string r2_DH_fname = root_pwd + config["r2_DH_fname"].asString();
    std::string r2_DH_tool_fname = root_pwd + config["r2_DH_tool_fname"].asString();
    std::string r2_DH_tool_assemble_fname = root_pwd + config["r2_DH_tool_assemble_fname"].asString();
    std::string r2_DH_tool_disassemble_fname = root_pwd + config["r2_DH_tool_disassemble_fname"].asString();
    std::string r2_DH_tool_alt_fname = root_pwd + config["r2_DH_tool_alt_fname"].asString();
    std::string r2_DH_tool_alt_assemble_fname = root_pwd + config["r2_DH_tool_alt_assemble_fname"].asString();
    std::string r2_DH_tool_handover_assemble_fname = root_pwd + config["r2_DH_tool_handover_assemble_fname"].asString();
    std::string r2_robot_base_fname = root_pwd + config["Robot2_Base_fname"].asString();

    std::string plate_calibration_fname = root_pwd + config["plate_calibration_fname"].asString();
    std::string env_setup_fname = root_pwd + config["env_setup_folder"].asString() + "env_setup_" + task_name + ".json";
    std::string lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();

    std::string task_fname = root_pwd + legoConfig["assembly_config"].asString();
    std::string world_base_fname = root_pwd + config["world_base_fname"].asString();

    bool assemble = config["Start_with_Assemble"].asBool();
    double twist_rad = config["Twist_Deg"].asInt() * M_PI / 180.0;
    double handover_twist_rad_ = config["Handover_Twist_Deg"].asInt() * M_PI / 180.0;

    set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
    lego_ptr_->setup(env_setup_fname, lego_lib_fname, plate_calibration_fname, assemble, task_json_, world_base_fname,
                    r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_assemble_fname, 
                    r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, 
                    r1_DH_tool_handover_assemble_fname, r1_robot_base_fname, 
                    r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_assemble_fname,
                    r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, 
                    r2_DH_tool_handover_assemble_fname, r2_robot_base_fname,
                    set_state_client_);
    
    task_seq_ = std::make_shared<task_def::LegoAssemblySeq>(lego_ptr_, task_fname);
}

std::set<Skill> LegoSkillGraph::feasible_u(const task_def::State &state)
{
    std::set<Skill> feasible_set;
    
    return feasible_set;
}

}