#pragma once
#include "Utils/Common.hpp"

namespace object {
    struct Object  {
        enum State {
            Static = 0,
            Attached = 1,
            Supported = 2,
            Handover = 3,
        };
        enum Shape {
            Box = 0,
            Sphere = 1,
            Cylinder = 2,
            Mesh = 3,
        };

        Object() = default;
        Object(const std::string &name, const std::string& parent_link, State state, double x, double y, double z, double qx, double qy, double qz, double qw):
            name(name), parent_link(parent_link), state(state), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) 
            {}
        
        bool operator==(const Object& other) const {
            return name == other.name;
        }
        
        std::string name;
        // mode of the object
        State state;
        std::string parent_link;
        int robot_id = -1;

        // geometry of the object
        double x, y, z;
        double qx = 0, qy = 0, qz = 0, qw = 1.0;
        double x_attach, y_attach, z_attach;
        double qx_attach = 0, qy_attach = 0, qz_attach = 0, qw_attach = 0;

        // collision shape of the object
        Shape shape;
        double radius;
        double length; // x
        double width; // y
        double height; // z
        std::string mesh_path;
    };

    struct Requirement {
        double precision;
    };

}