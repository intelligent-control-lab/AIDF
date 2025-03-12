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

        /*
        * Constructor for Robot Class
        * @param type: type of the robot
        * @param gripperType: type of the end-effector
        * @param sensorType: type of the sensor
        * @param name: name of the robot
        */
        Robot(const std::string &type, const std::string &gripperType, const std::string &sensorType,
            int robot_id, const std::string &name, const std::vector<std::string> &capabilities);
        
        int getDOF() const;
        void set_home_state(const std::vector<double> &home_state) {
            this->home_state = home_state;
        }

        std::string type_string() const;

        std::string tool_string() const;

        std::string to_string() const;

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
        int robot_id;
        std::string robot_name;
        int robot_dof; // degree of freedom of the robot links
        int hand_dof; // degree of freefom of end-effector
        std::vector<std::string> capabilities; // list of capabilities
        std::string end_effector_link; // end effector link
        std::vector<double> home_state; // home state of the robot
    };

    // define a pointer to the robot
    typedef std::shared_ptr<Robot> RobotPtr;


}