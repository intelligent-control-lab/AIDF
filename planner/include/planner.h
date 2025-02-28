#pragma once

#include "skillgraph.hpp"
#include <queue>
#include <unordered_map>

namespace planner {

// Custom comparator that only looks at the cost (first element)
struct CompareState {
    bool operator()(const std::pair<double, skillgraph::State>& a, const std::pair<double, skillgraph::State>& b) const {
        return a.first > b.first;  // For a min-heap (lowest cost first)
    }
};

class AssemblyPlanner {
public:
    AssemblyPlanner(std::shared_ptr<skillgraph::SkillGraph> skillgraph);
    
    ~AssemblyPlanner();
    
    void plan();

    bool get_path(skillgraph::State &state, std::vector<skillgraph::State> &path, 
        std::vector<skillgraph::SkillPtr> &gs_path);
        

private:
    std::shared_ptr<skillgraph::SkillGraph> skillgraph_;
    
    std::unordered_map<skillgraph::State, skillgraph::State> predecessor_;
    std::unordered_map<skillgraph::State, skillgraph::SkillPtr> predecessor_skill_;
};

}