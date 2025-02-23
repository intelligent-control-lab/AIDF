#include "skills.hpp"

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
    this->executor = std::make_shared<SkillExecutor>(this->type);
}

MetaSkill::MetaSkill(const std::string &name) {
    this->name = name;
    this->type = from_string(name);
    this->executor = std::make_shared<SkillExecutor>(this->type);
}

SkillExecutor::SkillExecutor(Skill::Type type) {
    // To be implemented
}

}