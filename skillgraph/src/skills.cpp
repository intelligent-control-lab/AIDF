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
    if (name == "Translate") return Skill::Type::Translate;
    if (name == "Rotate") return Skill::Type::Rotate;
    if (name == "TranslateWithRotation") return Skill::Type::TranslateWithRotation;
    
    // otherwise, we have an unknown skill
    throw std::runtime_error("Unknown skill type: " + name);

}

void Skill::set_default_param(const Json::Value &param) {
    if (this->default_param != nullptr) {
        this->default_param->set_json_param(param);
    }
    else {
        this->default_param = std::make_shared<SkillParam>(type, param);
    }
}

bool Skill::set_executor(std::shared_ptr<SkillExecutor> executor) {
    this->executor = executor;
    if (this->executor->skill_param == nullptr) {
        this->executor->set_skill_param(this->default_param);
    }
    return true;
}

void SkillParam::set_json_param(const Json::Value &param) {
    this->param = param;
};

bool SkillParam::has(const std::string &key) const {
    return param.isMember(key);
}


Json::Value SkillParam::get(const std::string &key) const {
    if (param.isMember(key)) {
        log("Parameter " + key + ": " + param[key].asString(), LogLevel::INFO);
        return param[key];
    } else {
        throw std::runtime_error("Key " + key + " not found in TaskParam config" + param.toStyledString());
    }
}

void SkillParam::update_param(const std::string &key, const std::string &value) {
    if (param.isMember(key)) {
        param[key] = value;
    } else {
        // If the key does not exist, add it
        param[key] = value;
    }
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
    if (this->executor->skill_param == nullptr) {
        this->executor->set_skill_param(this->default_param);
    }
    return true;
}

bool MetaSkill::set_compose_type(const std::string &type) {
    if (type == "temporal") {
        compose_type = Temporal;
    } else if (type == "functional") {
        compose_type = Functional;
    } else if (type == "spatial") {
        compose_type = Spatial;
    } else {
        log("Invalid composition type: " + type, LogLevel::ERROR);
        return false;
    }
    return true;
}


SkillExecutor::SkillExecutor(Skill::Type type) {
    this->skill_type = type;
    // To be implemented
}

MetaSkillExecutor::MetaSkillExecutor(Skill::Type type, MetaSkill::ComposeType compose_type, const std::vector<AtomicSkillPtr> &atomic_skills){
    this->compose_type = compose_type;
    this->skill_type = type;
}

void MetaSkillExecutor::add_atomic_executor(std::shared_ptr<SkillExecutor> atomic_executor) {
    atomic_executors.push_back(atomic_executor);
}

bool MetaSkillExecutor::execute(State &current_state) {
    log("Executing meta skill of composition type " + 
        std::to_string(static_cast<int>(compose_type)), LogLevel::INFO);
    if (compose_type == MetaSkill::ComposeType::Temporal) {
        for (const auto &atomic_executor : atomic_executors) {
            if (!atomic_executor->execute(current_state)) {
                log("Failed to execute atomic skill", LogLevel::ERROR);
                return false;
            }
        }
    }
    else if (compose_type == MetaSkill::ComposeType::Functional) {
        
    }
    else if (compose_type == MetaSkill::ComposeType::Spatial) {
        if (atomic_executors.size() < 1) {
            log("Spatial composition requires at least one atomic executor", LogLevel::ERROR);
            return false;
        }
        return atomic_executors[0]->execute(current_state);
    }
    return true;
}


void SkillExecutor::set_pre_condition(SkillConditionPtr pre_condition) {
    if (pre_condition == nullptr) {
        log("Pre condition is null", LogLevel::ERROR);
        return;
    }
    // make a copy of the pre condition 
    this->pre_condition = std::make_shared<SkillCondition>(*pre_condition);
}

void SkillExecutor::set_post_condition(SkillConditionPtr post_condition) {
    if (post_condition == nullptr) {
        log("Post condition is null", LogLevel::ERROR);
        return;
    }
    // make a copy of the post condition
    this->post_condition = std::make_shared<SkillCondition>(*post_condition);
}


void SkillExecutor::set_task_param(std::shared_ptr<TaskParam> task_param) {
   if (task_param == nullptr) {
        log("Task parameter is null", LogLevel::ERROR);
        return;
    }
    // make a copy of the task parameter
    this->task_param = std::make_shared<TaskParam>(*task_param);
}



void SkillExecutor::set_planned_trajectory(const RobotTrajectory &planned_trajectory) {
    this->planned_trajectory_ = planned_trajectory;
}

}