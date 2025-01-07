#include <memory>
#include <vector>
#include <unordered_map>
#include <string>

// Forward declarations
class State;
class Action;
class ActivityGraph;
class ADG;

// Represents a node in the skill graph
class SkillNode {
public:
    enum Type {
        STATE,      // Represents a state configuration
        ACTION,     // Represents an action/skill
        GEOMETRIC,  // Represents geometric constraints/poses
        KINEMATIC   // Represents kinematic constraints
    };

    SkillNode(Type type, const std::string& name) 
        : type_(type), name_(name) {}

    virtual bool isValid(const State& current_state) const = 0;
    virtual std::vector<std::shared_ptr<SkillNode>> getNextNodes() const = 0;

protected:
    Type type_;
    std::string name_;
    std::vector<std::weak_ptr<SkillNode>> next_nodes_;
    std::vector<std::weak_ptr<SkillNode>> prev_nodes_;
};

// The skill graph that defines the planning domain
class SkillGraph {
public:
    SkillGraph(int num_robots) : num_robots_(num_robots) {}

    // Domain definition methods
    void addState(const std::string& name, 
                 const std::vector<std::string>& preconditions,
                 const std::vector<std::string>& effects);
    
    void addAction(const std::string& name,
                  const std::vector<std::string>& preconditions,
                  const std::vector<std::string>& effects,
                  const std::vector<std::string>& geometric_constraints,
                  const std::vector<std::string>& kinematic_constraints);

    void addGeometricConstraint(const std::string& name,
                              const std::function<bool(const State&)>& validator);
    
    void addKinematicConstraint(const std::string& name,
                               const std::function<bool(const State&)>& validator);

    // Planning support methods
    std::vector<std::shared_ptr<Action>> getValidActions(const State& current_state) const;
    bool isValidTransition(const State& from_state, const Action& action, const State& to_state) const;
    bool validateState(const State& state) const;

    // Robot capability methods
    void setRobotCapabilities(int robot_id, 
                            const std::vector<std::string>& capabilities);
    
    bool hasCapability(int robot_id, const std::string& capability) const;

private:
    int num_robots_;
    std::unordered_map<std::string, std::shared_ptr<SkillNode>> nodes_;
    std::vector<std::vector<std::string>> robot_capabilities_;
};

// The main planner that uses the skill graph
class MultiRobotPlanner {
public:
    struct PlanningResult {
        bool success;
        std::shared_ptr<ActivityGraph> task_graph;
        std::shared_ptr<ADG> motion_graph;
        double planning_time;
        std::string failure_reason;
    };

    MultiRobotPlanner(std::shared_ptr<SkillGraph> skill_graph)
        : skill_graph_(std::move(skill_graph)) {}

    // Main planning methods
    PlanningResult plan(const State& initial_state,
                       const std::vector<std::string>& goals);

private:
    std::shared_ptr<SkillGraph> skill_graph_;

    // Helper methods for hierarchical planning
    bool generateTaskPlan(const State& initial_state,
                         const std::vector<std::string>& goals,
                         std::shared_ptr<ActivityGraph>& task_graph);
    
    bool generateMotionPlan(const std::shared_ptr<ActivityGraph>& task_graph,
                           std::shared_ptr<ADG>& motion_graph);

    bool validatePlanFeasibility(const std::shared_ptr<ActivityGraph>& task_graph,
                               const std::shared_ptr<ADG>& motion_graph);
};
