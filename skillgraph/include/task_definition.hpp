#pragma once
#include "Utils/FileIO.hpp"

namespace task_definition
{
    struct Object{
        //  lego brick
        // general
        std::string brick_name;
        int height;
        int width;
        double x;
        double y;
        double z;
        double quat_x;
        double quat_y;
        double quat_z;
        double quat_w;
        double cur_x;
        double cur_y;
        double cur_z;
        int press_side;
        int press_offset;
        Eigen::Quaterniond cur_quat;
        bool in_stock;
        std::map<std::string, std::string> top_connect;
        std::map<std::string, std::string> bottom_connect;
    };

    struct State{
        // robot state and env state
        // feasibility functions rely on lego state (cannot be task-agnostic?)
        // robot state (can be task-agnostic?)
    };
}