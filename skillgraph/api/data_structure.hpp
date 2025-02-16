#pragma once
#include "Utils/Common.hpp"
#include "objects.hpp"

namespace skillgraph {
    struct RobotState {
        /*
        * RobotState Class containing the robot state, name, and joint values
        */
        int robot_id;
        std::string robot_name; // same as group name in moveit
        std::vector<double> joint_values;
        std::vector<double> hand_values;

        // Define the equality operator
        bool operator==(const RobotState& other) const {
            return robot_id == other.robot_id &&
                robot_name == other.robot_name &&
                joint_values == other.joint_values &&
                hand_values == other.hand_values;
        }

        
        std::unordered_map<std::string, std::any> attributes;
    };

    struct EnvState {
        std::vector<skillgraph::Object> objects;
    };

    struct State {
        /*
        * State Class containing the robot state and environment state
        */
        RobotState robot_state;
        EnvState env_state;
    };


    struct RobotTrajectory {
        /*
        * RobotTrajectory Class containing the robot trajectory, times, action ids, and cost
        */ 
        int robot_id;
        std::vector<RobotState> trajectory;
        std::vector<double> times;
        std::vector<int> act_ids;
        double cost;
    };

    typedef std::vector<RobotTrajectory> MRTrajectory;

}