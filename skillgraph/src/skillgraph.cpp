#include "skillgraph.hpp"

namespace skillgraph {

SkillGraph::SkillGraph(const std::string &config) {
    
};


void SkillGraph::add_atomic_skill(const std::string &skill) {

};

std::set<Skill> SkillGraph::feasible_u(const task_def::State& state)
{
    if (task_type_ == TaskType::Lego) {
        return lego_feasible_u(state);
    }
    
}

std::set<Skill> SkillGraph::lego_feasible_u(const task_def::State &state)
{
    std::set<Skill> feasible_set;
    
    return feasible_set;
}

}