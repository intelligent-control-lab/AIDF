/**
 * @file robots.hpp
 * @brief Defines the Robot class and related types for the skillgraph framework.
 */
#pragma once
#include "Utils/Common.hpp"
#include "data_structure.hpp"

namespace skillgraph {
    /**
     * @class Robot
     * @brief Represents a robot, including type, end-effector, and capabilities.
     */
    class Robot {
        /*
        * Robot Class containing the robot type, name, degree of freedom, end-effector type, and capabilities
        */
    public:
        /**
         * @brief Default constructor.
         */
        Robot() = default;

        /**
         * @brief Construct a Robot with specified properties.
         * @param type Type of the robot (string).
         * @param gripperType Type of the end-effector (string).
         * @param sensorType Type of the sensor (string).
         * @param robot_id Robot ID.
         * @param name Name of the robot.
         * @param capabilities List of robot capabilities.
         */
        Robot(const std::string &type, const std::string &gripperType, const std::string &sensorType,
            int robot_id, const std::string &name, const std::vector<std::string> &capabilities);
        /**
         * @brief Get the total degrees of freedom (DOF) of the robot.
         * @return Total DOF (robot + hand).
         */
        int getDOF() const;
        /**
         * @brief Set the home state for the robot.
         * @param home_state Vector of joint values for the home state.
         */
        void set_home_state(const std::vector<double> &home_state) {
            this->home_state = home_state;
        }

        /**
         * @brief Get the robot type as a string.
         * @return Robot type string.
         */
        std::string type_string() const;

        /**
         * @brief Get the tool (end-effector) type as a string.
         * @return Tool type string.
         */
        std::string tool_string() const;

        /**
         * @brief Get a string representation of the robot.
         * @return String with robot details.
         */
        std::string to_string() const;

        /**
         * @enum Type
         * @brief Supported robot types.
         */
        enum Type {
            GP4 = 0,      /**< Yaskawa GP4 robot */
            Kinova = 1,   /**< Kinova robot */
            Panda = 2     /**< Franka Emika Panda robot */
        };

        /**
         * @enum Tool
         * @brief Supported end-effector types.
         */
        enum Tool {
            NoTool = 0,           /**< No tool */
            LegoTool = 1,         /**< Lego tool */
            SuctionCup = 2,       /**< Suction cup */
            TwoFingerGripper = 3, /**< Two-finger gripper */
            ThreeFingerGripper = 4/**< Three-finger gripper */
        };

        Tool tool;                        /**< End-effector tool type */
        Type type;                        /**< Robot type */
        int robot_id;                     /**< Robot ID */
        std::string robot_name;           /**< Robot name */
        int robot_dof;                    /**< DOF of robot links */
        int hand_dof;                     /**< DOF of end-effector */
        std::vector<std::string> capabilities; /**< List of capabilities */
        std::string end_effector_link;    /**< End effector link name */
        std::vector<double> home_state;   /**< Home state joint values */
    };

    /**
     * @brief Shared pointer to Robot.
     */
    typedef std::shared_ptr<Robot> RobotPtr;


}