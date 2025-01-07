#pragma once
#include "Utils/Common.hpp"

namespace robot {
    struct RobotState {
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

    };


    struct RobotTrajectory {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & trajectory;
            ar & times;
            ar & cost;
        }
        int robot_id;
        std::vector<RobotState> trajectory;
        std::vector<double> times;
        std::vector<int> act_ids;
        double cost;
    };

    typedef std::vector<RobotTrajectory> MRTrajectory;

    struct Robot {
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
    };


}