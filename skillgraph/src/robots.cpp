/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu 
    Peiqi Yu peiqiy@andrew.cmu.edu 
    Chaitanya Chawla cchawla@cs.cmu.edu 
    Changliu Liu cliu6@andrew.cmu.edu 
    Jiaoyang Li jiaoyanl@andrew.cmu.edu 
    Guanya Shi guanyas@andrew.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

/**
 * @file robots.cpp
 * @brief Implements the Robot class and related methods for the skillgraph framework.
 */

#include "robots.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {

/**
 * @brief Construct a Robot with specified properties.
 * @param type Type of the robot (string).
 * @param gripperType Type of the end-effector (string).
 * @param sensorType Type of the sensor (string).
 * @param robot_id Robot ID.
 * @param name Name of the robot.
 * @param capabilities List of robot capabilities.
 * @throws std::runtime_error if the robot or gripper type is not supported.
 */
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

/**
 * @brief Get the robot type as a string.
 * @return Robot type string.
 */
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

/**
 * @brief Get the tool (end-effector) type as a string.
 * @return Tool type string.
 */
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

/**
 * @brief Get a string representation of the robot.
 * @return String with robot details.
 */
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

/**
 * @brief Get the total degrees of freedom (DOF) of the robot.
 * @return Total DOF (robot + hand).
 */
int Robot::getDOF() const
{
    return robot_dof + hand_dof;
}

}