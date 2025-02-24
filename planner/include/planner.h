#pragma once
#include "skillgraph.hpp"

namespace planner {

class AssemblyPlanner {
public:
    AssemblyPlanner(std::shared_ptr<skillgraph::SkillGraph> skillgraph);
    
    ~AssemblyPlanner();
    
    void plan();

private:
    std::shared_ptr<skillgraph::SkillGraph> skillgraph_;
};

}