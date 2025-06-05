#include "skills.hpp"
#include "tasks.hpp"
#include "Utils/Logger.hpp"

namespace skillgraph {

bool Skill::isAtomic(Skill::Type type) {
    return int(type) < 100;
}

bool Skill::isMeta(Skill::Type type) {
    return int(type) >= 100;
}

Skill::Type Skill::from_string(const std::string &name) {
    if (name == "Pick") return Skill::Type::Pick;
    if (name == "Place-Down") return Skill::Type::PlaceTop;
    if (name == "Place-Up") return Skill::Type::PlaceBottom;
    if (name == "Support-Bottom") return Skill::Type::SupportBottom;
    if (name == "Support-Top") return Skill::Type::SupportTop;
    if (name == "Handover") return Skill::Type::Handover;
    if (name == "Transit") return Skill::Type::Transit;
    if (name == "align") return Skill::Type::align;
    if (name == "PlaceWithSupport") return Skill::Type::PlaceWithSupport;
    if (name == "PickAndPlace") return Skill::Type::PickAndPlace;
    if (name == "PickAndPlaceWithSupport") return Skill::Type::PickAndPlaceWithSupport;
    if (name == "PickHandoverAndPlace") return Skill::Type::PickHandoverAndPlace;
    
    // otherwise, we have an unknown skill
    throw std::runtime_error("Unknown skill type: " + name);

}

void Skill::set_param(const Json::Value &param) {
    this->param = param;
}


AtomicSkill::AtomicSkill(const std::string &name) {
    this->name = name;
    this->type = from_string(name);
}

MetaSkill::MetaSkill(const std::string &name, const std::vector<AtomicSkillPtr> &atomic_skills,
        int num_robot, const std::vector<int> &robot_ids) {
    this->name = name;
    this->type = from_string(name);
    this->atomic_skills = atomic_skills;
    for (int i = 0; i < atomic_skills.size(); i++) {
        this->atomic_skills[i]->seq_within_meta = i;
    }
    this->num_robot = num_robot;
    this->robot_ids = robot_ids;
}

MetaSkill::MetaSkill(const MetaSkill &meta_skill) {
    this->name = meta_skill.name;
    this->type = meta_skill.type;
    this->atomic_skills.clear();
    // copy the individual skills too
    for (auto &skill : meta_skill.atomic_skills) {
        this->atomic_skills.push_back(std::make_shared<AtomicSkill>(*skill));
    }
    this->num_robot = meta_skill.num_robot;
    this->robot_ids = meta_skill.robot_ids;
}

bool MetaSkill::set_robot(const std::vector<RobotPtr> &robots) {
    /*
    * Use provided robots to set the robot for all atomic skills, based on robot_ids in skillgraph
    */
    if (robots.size() != num_robot) {
        log("Number of robots does not match", LogLevel::ERROR);
        return false;
    }

    for (int i = 0; i < robot_ids.size(); i++) {
        int use_robot_id = robot_ids[i];
        atomic_skills[i]->robot = robots[use_robot_id];
    }
    this->robots = robots;
    return true;
}


bool MetaSkill::set_object(ObjPtr obj) {
    
    if (obj == nullptr) {
        log("Object is null", LogLevel::ERROR);
        return false;
    }

    for (int i = 0; i < atomic_skills.size(); i++) {
        atomic_skills[i]->object = obj;
    }
    
    // add the object to the list of objects if it is not already there
    if (std::find(objects.begin(), objects.end(), obj) == objects.end()) {
        objects.push_back(obj);
    }
    return true;
}

std::vector<Skill::Type> MetaSkill::get_atomic_skill_types() {
    std::vector<Skill::Type> types;
    for (const auto &skill : atomic_skills) {
        types.push_back(skill->type);
    }
    return types;
}

bool MetaSkill::set_executor(std::shared_ptr<MetaSkillExecutor> executor) {
    this->executor = executor;
    return true;
}


SkillExecutor::SkillExecutor(Skill::Type type) {
    // To be implemented
}

MetaSkillExecutor::MetaSkillExecutor(Skill::Type type, const std::vector<AtomicSkillPtr> &atomic_skills){

}

void MetaSkillExecutor::add_atomic_executor(std::shared_ptr<SkillExecutor> atomic_executor) {
    atomic_executors.push_back(atomic_executor);
}

bool MetaSkillExecutor::execute(State &current_state) {
    for (const auto &atomic_executor : atomic_executors) {
        if (!atomic_executor->execute(current_state)) {
            log("Failed to execute atomic skill", LogLevel::ERROR);
            return false;
        }
    }
    return true;
}


void SkillExecutor::set_pre_condition(TaskParamPtr pre_condition) {
    if (pre_condition == nullptr) {
        log("Pre condition is null", LogLevel::ERROR);
        return;
    }
    // make a copy of the pre condition for the meta skill executor
    this->pre_condition = std::make_shared<TaskParam>(*pre_condition);
}

void SkillExecutor::set_post_condition(TaskParamPtr post_condition) {
    if (post_condition == nullptr) {
        log("Post condition is null", LogLevel::ERROR);
        return;
    }
    // make a copy of the post condition for the meta skill executor
    this->post_condition = std::make_shared<TaskParam>(*post_condition);
}

void SkillExecutor::set_planned_trajectory(const RobotTrajectory &planned_trajectory) {
    this->planned_trajectory_ = planned_trajectory;
}

}