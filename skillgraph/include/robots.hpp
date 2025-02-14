#pragma once
#include "Utils/Common.hpp"

namespace robot {
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

    class Robot {
        /*
        * Robot Class containing the robot type, name, degree of freedom, end-effector type, and capabilities
        */
    public:
        Robot() = default;
        enum Type {
            GP4 = 0,
            Kinova = 1,
            Panda = 2
        };


        enum Tool {
            // end effector type
            NoTool = 0,
            LegoTool = 1,
            SuctionCup = 2,
            TwoFingerGripper = 3,
            ThreeFingerGripper = 4
        };

        Tool tool;
        Type type;
        std::string robot_name;
        int robot_dof; // degree of freedom of the robot links
        int hand_dof; // degree of freefom of end-effector
        std::vector<std::string> capabilities; // list of capabilities
    };


}