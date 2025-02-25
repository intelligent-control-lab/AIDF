#include "robots.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {


Robot::Robot(const std::string &type, const std::string &gripperType, const std::string &sensorType,
        const std::string &name, const std::vector<std::string> &capabilities)
{
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

int Robot::getDOF() const
{
    return robot_dof + hand_dof;
}

}