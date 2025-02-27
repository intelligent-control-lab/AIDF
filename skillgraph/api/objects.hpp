#pragma once
#include "Utils/Common.hpp"

namespace skillgraph {
    struct Object  {
        /*
        * Object Class containing the object state, name, type and geometry
        */

        enum State {
            /* Object State Enum */
            Static = 0,
            Attached = 1,
            Supported = 2,
            Handover = 3,
        };
        enum Shape {
            /* Object Shape Enum */
            Box = 0,
            Sphere = 1,
            Cylinder = 2,
            Mesh = 3,
        };
        enum Type {
            /* Object Type Enum */
            LegoBrick = 0,
            MagneticBlock = 1,
        };

        Object() = default;
        Object(const std::string &name, const std::string& parent_link, State state, 
            double x, double y, double z, double qx, double qy, double qz, double qw);

        virtual bool operator==(const Object& other) const {
            return name == other.name && x == other.x && y == other.y && z == other.z &&
                qx == other.qx && qy == other.qy && qz == other.qz && qw == other.qw &&
                state == other.state && parent_link == other.parent_link;
        }

        virtual bool sameType(const Object &other) const {
            return type == other.type;
        }

        virtual std::string to_string() const {
            std::string str = "Object: ";
            str += "Name: " + name + " ";
            str += "Type: " + std::to_string(type) + " ";
            str += "State: " + std::to_string(state) + " ";
            str += "Parent Link: " + parent_link + " \n";
            str += "Position: " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " \n";
            str += "Orientation: " + std::to_string(qx) + " " + std::to_string(qy) + " " + std::to_string(qz) + " " + std::to_string(qw) + " \n";
            return str;
        }
        
        std::string name;
        Type type;
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
    typedef std::shared_ptr<Object> ObjPtr;

} // namespace skillgraph

namespace std {
    // Helper function to combine hash values - define this BEFORE using it
    template <typename T>
    inline void std_hash_combine(std::size_t& seed, const T& value) {
        seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    
// Alternative to bit_cast_alternative for pre-C++20
template<typename To, typename From>
inline To bit_cast_alternative(const From& from) {
    static_assert(sizeof(To) == sizeof(From), "Types must be same size for bit_cast_alternative");
    static_assert(std::is_trivially_copyable<From>::value, "From type must be trivially copyable");
    static_assert(std::is_trivially_copyable<To>::value, "To type must be trivially copyable");
    
    To to;
    std::memcpy(&to, &from, sizeof(To));
    return to;
}


template <>
struct hash<skillgraph::Object> {
    std::size_t operator()(const skillgraph::Object& obj) const {
        std::size_t seed = 0;
        
        // Hash string and enum members
        std_hash_combine(seed, obj.name);
        std_hash_combine(seed, static_cast<int>(obj.type));
        std_hash_combine(seed, static_cast<int>(obj.state));
        std_hash_combine(seed, obj.parent_link);
        std_hash_combine(seed, obj.robot_id);
        
        // Hash position and orientation
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.x));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.y));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.z));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qx));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qy));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qz));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qw));
        
        // Hash attachment data
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.x_attach));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.y_attach));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.z_attach));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qx_attach));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qy_attach));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qz_attach));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.qw_attach));
        
        // Hash shape data
        std_hash_combine(seed, static_cast<int>(obj.shape));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.radius));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.length));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.width));
        std_hash_combine(seed, bit_cast_alternative<std::uint64_t>(obj.height));
        std_hash_combine(seed, obj.mesh_path);
        
        return seed;
    }
};

} // namespace std