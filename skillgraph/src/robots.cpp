#include "robots.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {


Robot::Robot(const std::string &type, const std::string &gripperType, const std::string &sensorType,
        int robot_id, const std::string &name, const std::vector<std::string> &capabilities)
{
    this->robot_id = robot_id;
    this->robot_name = name;
    if (type == "gp4") {
        this->type = Robot::Type::GP4;
        this->robot_dof = 6;
        if (robot_name == "left_arm") {
            end_effector_link = "left_arm_link_tool";
        }
        else if (robot_name == "right_arm") {
            end_effector_link = "right_arm_link_tool";
        }
        else {
            log("Robot end-effector link name not set", LogLevel::WARN);
        }
    }
    else {
        // not supported raise error
        throw std::runtime_error("Robot type not supported");
    }

    if (gripperType == "lego") {
        this->tool = Robot::Tool::LegoTool;
        this->hand_dof = 0;
    }
    else if (gripperType == "two_finger") {
        this->tool = Robot::Tool::TwoFingerGripper;
        this->hand_dof = 2;
    }
    else {
        // not supported raise error
        throw std::runtime_error("Gripper type not supported");
    }

    this->capabilities = capabilities;

}

std::string Robot::type_string() const {
    if (type == Type::GP4) {
        return "GP4";
    }
    else if (type == Type::Kinova) {
        return "Kinova";
    }
    else if (type == Type::Panda) {
        return "Panda";
    }
    else {
        return "Unknown";
    }
}

std::string Robot::tool_string() const {
    if (tool == Tool::LegoTool) {
        return "LegoTool";
    }
    else if (tool == Tool::TwoFingerGripper) {
        return "TwoFingerGripper";
    }
    else if (tool == Tool::ThreeFingerGripper) {
        return "ThreeFingerGripper";
    }
    else if (tool == Tool::SuctionCup) {
        return "SuctionCup";
    }
    else {
        return "Unknown";
    }
}

std::string Robot::to_string() const {
    std::string str = "Robot: " + robot_name + "\n";
    str += "  Type: " + type_string() + "\n";
    str += "  Tool: " + tool_string() + "\n";
    str += "  Capabilities:\n";
    for (const auto& cap : capabilities) {
        str += "    - " + cap + "\n";
    }
    return str;
}

int Robot::getDOF() const
{
    return robot_dof + hand_dof;
}

}