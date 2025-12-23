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