#include "objects.hpp"

namespace skillgraph {

Object::Object(const std::string &name, const std::string& parent_link, State state, 
    double x, double y, double z, double qx, double qy, double qz, double qw):
    name(name), parent_link(parent_link), state(state), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) 
{
    
}

}