#pragma once
#include "robots.hpp"
#include "object.hpp"

namespace env {


class PlanInstance {
public:
    virtual void setNumberOfRobots(int num_robots);
    virtual void setRobotNames(const std::vector<std::string>& robot_names) {
        robot_names_ = robot_names;
    }
    virtual void setHandNames(const std::vector<std::string>& hand_names) {
        hand_names_ = hand_names;
    }
    virtual void setStartPose(int robot_id, const std::vector<double>& pose);
    virtual void setGoalPose(int robot_id, const std::vector<double>& pose);
    virtual bool checkCollision(const std::vector<robot::RobotState> &poses, bool self, bool debug=false) = 0;
    virtual double computeDistance(const robot::RobotState& a, const robot::RobotState &b) const = 0;
    virtual double computeDistance(const robot::RobotState& a, const robot::RobotState &b, int dim) const = 0;
    virtual bool connect(const robot::RobotState& a, const robot::RobotState& b, double col_step_size = 0.1, bool debuf=false) = 0;
    virtual bool steer(const robot::RobotState& a, const robot::RobotState& b, double max_dist,  robot::RobotState& result, double col_step_size = 0.1) = 0;
    virtual bool sample(robot::RobotState &pose) = 0;
    virtual double getVMax(int robot_id);
    virtual void setVmax(double vmax);
    virtual robot::RobotState interpolate(const robot::RobotState &a, const robot::RobotState&b, double t) const = 0;
    virtual double interpolate(const robot::RobotState &a, const robot::RobotState&b, double t, int dim) const = 0;
    virtual void addMoveableObject(const env::Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void moveObject(const env::Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void removeObject(const std::string& name) { throw std::runtime_error("Not implemented");};
    virtual void moveRobot(int robot_id, const robot::RobotState& pose) { throw std::runtime_error("Not implemented");};
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const robot::RobotState &pose) { throw std::runtime_error("Not implemented");};
    virtual void detachObjectFromRobot(const std::string& name, const robot::RobotState &pose) { throw std::runtime_error("Not implemented");};
    virtual void updateScene() = 0;
    virtual void resetScene(bool reset_sim) = 0;
    virtual void setPadding(double padding) {throw std::runtime_error("Not implemented");};
    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) { throw std::runtime_error("Not implemented");};
    virtual void printKnownObjects() const { throw std::runtime_error("Not implemented");};
    virtual int numCollisionChecks();
    // Additional methods for future functionalities can be added here
    virtual ~PlanInstance() = default;

    virtual int getNumberOfRobots() const {
        return num_robots_;
    }

    virtual std::vector<robot::RobotState> getStartPoses() const {
        return start_poses_;
    }

    virtual std::vector<robot::RobotState> getGoalPoses() const {
        return goal_poses_;
    }

    virtual robot::RobotState getStartPose(int robot_id) const {
        assert (robot_id < start_poses_.size());
        return start_poses_[robot_id];
    }

    virtual robot::RobotState getGoalPose(int robot_id) const {
        assert (robot_id < goal_poses_.size());
        return goal_poses_[robot_id];
    }

    virtual robot::RobotState initRobotState(int robot_id) const;

    virtual void setRobotDOF(int robot_id, size_t dof);

    virtual void setHandDof(int robot_id, size_t dof);

    virtual size_t getRobotDOF(int robot_id) const;

    virtual size_t getHandDOF(int robot_id) const;

    virtual bool hasObject(const std::string& name) const {
        return objects_.find(name) != objects_.end();
    }

    virtual env::Object getObject(const std::string& name) const {
        return objects_.at(name);
    }

    virtual std::vector<env::Object> getAttachedObjects(int robot_id) const {
        std::vector<env::Object> attached_objects;
        for (const auto& obj : objects_) {
            if (obj.second.robot_id == robot_id && obj.second.state == env::Object::State::Attached) {
                attached_objects.push_back(obj.second);
            }
        }
        return attached_objects;
    }

protected:
    int num_robots_;
    double v_max_ = 1.0;
    std::vector<robot::RobotState> start_poses_;
    std::vector<size_t> robot_dof_, hand_dof_;
    std::vector<robot::RobotState> goal_poses_;
    std::vector<std::string> robot_names_, hand_names_;
    std::unordered_map<std::string, env::Object> objects_;

    int num_collision_checks_ = 0;
};

}