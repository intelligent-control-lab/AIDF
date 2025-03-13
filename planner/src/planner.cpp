#include "planner.h"
#include "Utils/Logger.hpp"

namespace planner {

using namespace skillgraph;

AssemblyPlanner::AssemblyPlanner(std::shared_ptr<skillgraph::SkillGraph> skillgraph) {
    skillgraph_ = skillgraph;
}

AssemblyPlanner::~AssemblyPlanner() {
}

void AssemblyPlanner::plan() {
    // use the skill graph's next feasible u, to do a greedy best first search
    
    // get initial state
    State state = skillgraph_->get_initial_state();

    // create a priority queue, with key as the cost of the state, value as the state, just compare the cost double not the state
    std::priority_queue<std::pair<double, State>, std::vector<std::pair<double, State>>, CompareState> pq;
    pq.push(std::make_pair(0.0, state));

    // store the predecessor
    predecessor_.clear();
    predecessor_skill_.clear();

    // use A* search to find the best path
    while (!pq.empty()) {
        // get the top element
        auto top = pq.top();
        pq.pop();
        State current_state = top.second;
        double current_cost = top.first;
        //std::cout << "Current state: " << current_state.to_string() << std::endl;
        //std::cout << "Current cost: " << current_cost << std::endl;

        if (skillgraph_->at_target(current_state)) {
            // print the path
            plan_path.clear();
            plan_skill_seq.clear();
            get_path(current_state, plan_path, plan_skill_seq);
            std::cout << "Found the target state!" << std::endl;
            for (const auto &s : plan_skill_seq) {
                std::cout << s->to_string() << std::endl;
            }
            return;
        }

        // get the feasible set
        std::vector<SkillPtr> feasible_set = skillgraph_->feasible_u(current_state);

        // for each feasible skill, get the next state
        for (const auto &gs : feasible_set) { 
            State next_state;
            double cost;

            if (skillgraph_->get_next_state(current_state, gs, next_state, cost)) {
                // check if the next state is the target state
                
                // add the next state to the priority queue
                pq.push(std::make_pair(cost + current_cost, next_state));
                predecessor_[next_state] = current_state;
                predecessor_skill_[next_state] = gs;
            }
        }
    }

}

bool AssemblyPlanner::get_path(State &state, std::vector<State> &path, std::vector<SkillPtr> &gs_path) {
    path.push_back(state);
    while (predecessor_.find(state) != predecessor_.end()) {
        if (predecessor_skill_.find(state) != predecessor_skill_.end()) {
            gs_path.push_back(predecessor_skill_[state]);
        }
        else {
            log("No skill found for the state", LogLevel::ERROR);
            return false;
        }
        state = predecessor_[state];
        path.push_back(state);
    }
    std::reverse(path.begin(), path.end());
    std::reverse(gs_path.begin(), gs_path.end());
    return true;
}

}
