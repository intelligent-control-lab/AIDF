/**
 * @file objects.cpp
 * @brief Implements the Object class for the skillgraph framework.
 */
#include "objects.hpp"

namespace skillgraph {

/**
 * @brief Construct an Object with specified properties.
 * @param name Name of the object.
 * @param parent_link Parent link name.
 * @param state State of the object.
 * @param x X position.
 * @param y Y position.
 * @param z Z position.
 * @param qx Quaternion X.
 * @param qy Quaternion Y.
 * @param qz Quaternion Z.
 * @param qw Quaternion W.
 */
Object::Object(const std::string &name, const std::string& parent_link, State state, 
    double x, double y, double z, double qx, double qy, double qz, double qw):
    name(name), parent_link(parent_link), state(state), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) 
{
    
}

}