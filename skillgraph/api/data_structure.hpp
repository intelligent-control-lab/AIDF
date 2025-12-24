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

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

#pragma once
#include "Utils/Common.hpp"
#include "objects.hpp"

namespace skillgraph {
    struct RobotState {
        /*
        * RobotState Class containing the robot state, name, and joint values
        */

        RobotState() = default;

        int robot_id = -1;
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

        std::string to_string() const {
            std::string str = "RobotState: ";
            str += "Robot Name: " + robot_name + " ";
            str += "Joint Values: ";
            for (const auto& val : joint_values) {
                str += std::to_string(val) + " ";
            }
            str += "Hand Values: ";
            for (const auto& val : hand_values) {
                str += std::to_string(val) + " ";
            }
            str += "\n";
            return str;
        }

        
        std::unordered_map<std::string, std::any> attributes;
    };

    struct EnvState {
        EnvState() = default;
        // copy constructor
        EnvState(const EnvState &other) {
            objects.clear();
            objects.reserve(other.objects.size());
            
            for (const auto& obj : other.objects) {
                if (obj) { // Check for null pointers
                    objects.push_back(obj->clone());
                } else {
                    // Push a null pointer or create an empty object based on your needs
                    objects.push_back(nullptr);
                }
            }
        }

        // Assignment operator
        EnvState& operator=(const EnvState& other) {
            if (this != &other) {  // Self-assignment check
                objects.clear();
                objects.reserve(other.objects.size());
                
                for (const auto& obj : other.objects) {
                    if (obj) {
                        objects.push_back(obj->clone());
                    } else {
                        objects.push_back(nullptr);
                    }
                }
            }
            return *this;
        }

        std::vector<ObjPtr> objects;
    
        std::string to_string() const {
            std::string str = "EnvState: \n";
            for (const auto& obj : objects) {
                str += obj->to_string() + "\n";
            }
            return str;
        }

        bool operator==(const EnvState &other) const {
            if (objects.size() != other.objects.size()) {
                return false;
            }
            for (int i = 0; i < objects.size(); i++) {
                if (!(*objects[i] == *other.objects[i])) {
                    return false;
                }
            }
            return true;
        }
    };

    struct State {
        /*
        * State Class containing the robot state and environment state
        */
        State() = default;
        std::vector<RobotState> robot_states;
        EnvState env_state;
        int assembled_steps = 0;

        bool operator==(const State &other) const {
            if (robot_states.size() != other.robot_states.size()) {
                return false;
            }
            for (int i = 0; i < robot_states.size(); i++) {
                if (!(robot_states[i] == other.robot_states[i])) {
                    return false;
                }
            }
            return env_state == other.env_state && assembled_steps == other.assembled_steps;
        }

        std::string to_string() const {
            std::string str = "State: ";
            for (const auto& rs : robot_states) {
                str += rs.to_string();
            }
            str += env_state.to_string();
            return str;
        }
    };


    struct RobotTrajectory {
        /*
        * RobotTrajectory Class containing the robot trajectory, times, action ids, and cost
        */ 
        RobotTrajectory() = default;
        int robot_id = -1;
        std::vector<RobotState> trajectory;
        std::vector<double> times;
        std::vector<int> act_ids;
        double cost = 0.0;
    };

    typedef std::vector<RobotTrajectory> MRTrajectory;

}

// Hash specializations should be placed in the global namespace, after your type definitions
namespace std {
    
    template <>
    struct hash<skillgraph::RobotState> {
        std::size_t operator()(const skillgraph::RobotState& rs) const {
            std::size_t seed = 0;
            std_hash_combine(seed, rs.robot_id);
            std_hash_combine(seed, rs.robot_name);
            
            for (const auto& val : rs.joint_values) {
                std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(val)); 
            }
            for (const auto& val : rs.hand_values) {
                std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(val));
            }
            
            return seed;
        }
    };

    template <>
    struct hash<skillgraph::EnvState> {
        std::size_t operator()(const skillgraph::EnvState& env) const {
            std::size_t seed = 0;
            
            // Hash each object by content rather than pointer
            for (const auto& obj_ptr : env.objects) {
                if (obj_ptr) {  // Check for null pointers
                    std_hash_combine(seed, std::hash<skillgraph::Object>{}(*obj_ptr));
                } else {
                    // Handle null pointers consistently
                    std_hash_combine(seed, std::size_t(0));
                }
            }
            
            return seed;
        }
    };

    template <>
    struct hash<skillgraph::State> {
        std::size_t operator()(const skillgraph::State& state) const {
            std::size_t seed = 0;
            
            for (const auto& rs : state.robot_states) {
                std_hash_combine(seed, rs);  // Uses the hash<RobotState> specialization
            }
            
            std_hash_combine(seed, state.env_state);  // Uses the hash<EnvState> specialization
            std_hash_combine(seed, state.assembled_steps);
            
            return seed;
        }
    };
}
