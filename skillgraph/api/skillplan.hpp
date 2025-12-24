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

#include "skillgraph.hpp"

namespace skillgraph::skillplan {

struct ExportOptions {
    bool include_environment_spec = true;
    bool include_action_constraints = true;
    double float_tolerance = 1e-9;
};

Json::Value export_plan(const SkillGraph &skillgraph,
                        const std::vector<SkillPtr> &plan_skill_seq,
                        const State &initial_state,
                        const ExportOptions &options = ExportOptions());

bool write_json_to_file(const Json::Value &json, const std::string &path, std::string *error = nullptr);

} // namespace skillgraph::skillplan

