#pragma once
#include "algorithms.hpp"
#include "Utils/Common.hpp"

namespace skillgraph {
    
    class SkillExecutor; // forward declaration

    struct Skill {
        /*
        Class definition of Skill and Type
        */
        enum Type {
            Pick = 1,
            PlaceTop = 2,
            PlaceBottom = 3,
            SupportBottom = 4,
            SupportTop = 5,
            Handover = 6,
            Transit = 7,
            align = 8,
            PlaceWithSupport = 101,
            PickAndPlace = 102,
            PickAndPlaceWithSupport = 103,
            PickHandoverAndPlace = 104,
        };

        static bool isAtomic(Type type);
        static bool isMeta(Type type);
        static Type from_string(const std::string &type);
        void set_param(const Json::Value &param);

        Type type; // enum type
        std::string name; // name
        std::shared_ptr<SkillExecutor> executor; // skill executor
        Json::Value param; // parameters
        
        virtual std::string to_string() const {
            return "Skill: " + name;
        }
    };

    struct AtomicSkill : public Skill {
        /*
        Class definition of AtomicSkill
        */
        AtomicSkill(const std::string &name);
    };

    struct MetaSkill : public Skill {
        /*
        Class definition of MetaSkill, which contains a collection of atomic skills
        */
        MetaSkill(const std::string &name);
        
        std::vector<Skill::Type> atomic_skills;
    };

    class SkillExecutor {
        /*
        Class definition of SkillExecutor
        */
        public:
            SkillExecutor() = default;
            SkillExecutor(Skill::Type type);
            
            // chooses one or many algorithms to implement the skill 
            std::vector<skillgraph::Algorithm> implementation;

            // properties
            std::shared_ptr<skillgraph::ConditionEvaluator> pre_condition;
            std::shared_ptr<skillgraph::ConditionEvaluator> post_condition;
            Skill::Type skill_type; // corresponding skill

            // funnction implementation for executing this skill
            std::function<std::any(const std::vector<std::any>&)> perform();
    };
    
}