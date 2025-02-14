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
            pick = 1,
            place_top = 2,
            place_bottom = 3,
            support_bottom = 4,
            support_top = 5,
            handover = 6,
            transfer = 7,
            align = 8,
        };

        bool isAtomic(Type type);
        bool isMeta(Type type);

        Type type; // enum type
        std::string name; // name
        std::shared_ptr<SkillExecutor> executor; // skill executor
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
            SkillExecutor(const std::string &name);
            
            // chooses one or many algorithms to implement the skill 
            std::vector<algo::Algorithm> implementation;

            // properties
            metric::Evaluator pre_condition;
            metric::Evaluator post_condition;
            Skill::Type skill_type; // corresponding skill

            // funnction implementation for executing this skill
            std::function<std::any(const std::vector<std::any>&)> perform();
    };
    
}