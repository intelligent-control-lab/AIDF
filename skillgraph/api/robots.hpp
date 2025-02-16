#pragma once
#include "Utils/Common.hpp"
#include "data_structure.hpp"

namespace skillgraph {
    
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