#include "skills.hpp"
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
    this->num_robot = num_robot;
    this->robot_ids = robot_ids;
}
 

bool MetaSkill::set_robot(const std::vector<RobotPtr> robots, int primary_robot_id) {
    if (robots.size() != num_robot) {
        log("Number of robots does not match", LogLevel::ERROR);
        return false;
    }

    for (int i = 0; i < robot_ids.size(); i++) {
        atomic_skills[i]->robot = robots[robot_ids[i]];
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


SkillExecutor::SkillExecutor(Skill::Type type) {
    // To be implemented
}

}