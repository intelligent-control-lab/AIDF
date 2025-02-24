#include "planner.h"

namespace planner {

AssemblyPlanner::AssemblyPlanner(std::shared_ptr<skillgraph::SkillGraph> skillgraph) {
    skillgraph_ = skillgraph;
}

AssemblyPlanner::~AssemblyPlanner() {
}

void AssemblyPlanner::plan() {
    // use the skill graph's next feasible u, to do a greedy best first search
}

}
