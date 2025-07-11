#pragma once
#include "algorithms.hpp"
#include "Utils/Common.hpp"

namespace skillgraph {
    
    class SkillExecutor; // forward declaration
    class MetaSkillExecutor; // forward declaration
    class SkillParam; // forward declaration

    class Skill {
    public:
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
            Translate = 9,
            Rotate = 10,
            PlaceWithSupport = 101,
            PickAndPlace = 102,
            PickAndPlaceWithSupport = 103,
            PickHandoverAndPlace = 104,
            PickAndPlaceRealRobot = 105, // TODO: Chaitanya - correct this
            TranslateWithRotation = 106,
        };

        static bool isAtomic(Type type);
        static bool isMeta(Type type);
        static Type from_string(const std::string &type);
        void set_param(const Json::Value &param);

        virtual std::string to_string() const {
            return "Skill: " + name;
        }

        virtual ObjPtr get_object() {
            return nullptr;
        };

        Type type; // enum type
        std::string name; // name
        std::shared_ptr<SkillExecutor> executor; // skill executor
        std::shared_ptr<SkillParam> param; // skill parameters
        
    };
    typedef std::shared_ptr<Skill> SkillPtr;

    class SkillParam {
    public:
        /*
        Class definition of SkillParam, which contains the skill type and parameters
        */
        SkillParam(Skill::Type type, const Json::Value &param) : type(type), param(param) {}

        void set_json_param(const Json::Value &param);

        bool get(const std::string &key, Json::Value &value) const;

        void update_param(const std::string &key, const std::string &value);

        virtual std::string to_string() const {
            return "SkillParam: " + std::to_string(type) + " with parameters: " + param.toStyledString();
        }
    
        Skill::Type type; // skill type
        Json::Value param; // parameters
    };
    typedef std::shared_ptr<SkillParam> SkillParamPtr;

    class AtomicSkill : public Skill {
    public:
        /*
        Class definition of AtomicSkill
        */
        AtomicSkill(const std::string &name);
        
        virtual std::string to_string() const override {
            std::string str = "Atomic Skill: " + name;
            if (object) {
                str += " object: " + object->name;
            }
            if (robot) {
                str += " robot: " + robot->robot_name;
            }
            return str;
        }

        virtual ObjPtr get_object() override {
            return object;
        }

        ObjPtr object;
        RobotPtr robot;
        int seq_within_meta = -1;
    };
    typedef std::shared_ptr<AtomicSkill> AtomicSkillPtr;

    class MetaSkill : public Skill {
    public:
        enum ComposeType {
            Temporal = 1, // sequential execution
            Functional = 2, // parallel execution
            Spatial = 3 // spatial execution
        };
        /*
        Class definition of MetaSkill, which contains a collection of atomic skills
        */
        MetaSkill(const std::string &name, const std::vector<AtomicSkillPtr> &atomic_skills,
                int num_robot, const std::vector<int> &robot_ids);

        // copy constructor
        MetaSkill(const MetaSkill &meta_skill);

        // Set the robot for all atomic skills according to the id of the primary robot
        bool set_robot(const std::vector<RobotPtr> &robots);
        // Set the object for all atomic skills
        bool set_object(ObjPtr obj);
        // Set executor;
        bool set_executor(std::shared_ptr<MetaSkillExecutor> executor);

        //set the composition type
        bool set_compose_type(const std::string &type);

        std::vector<Skill::Type> get_atomic_skill_types();

        virtual std::string to_string() const override {
            std::string str = "Meta Skill: " + name;
            for (const auto &robot : robots) {
                str += " robot: " + robot->robot_name;
            }
            for (const auto &obj : objects) {
                str += " object: " + obj->name;
            }
            return str;
        }

        virtual ObjPtr get_object() override {
            return (objects.size() > 0) ? objects[0] : nullptr;
        }

        int num_robot;
        std::vector<int> robot_ids;
        std::vector<AtomicSkillPtr> atomic_skills;
        std::vector<RobotPtr> robots;
        std::vector<ObjPtr> objects;
        ComposeType compose_type = Temporal; // default composition type
    };
    typedef std::shared_ptr<MetaSkill> MetaSkillPtr;

    // forward declaration
    class TaskParam;
    typedef std::shared_ptr<TaskParam> TaskParamPtr;
   
   
    class SkillExecutor {
        /*
        Class definition of SkillExecutor
        */
        public:
            SkillExecutor() = default;
            SkillExecutor(Skill::Type type);
            
            // chooses one or many algorithms to implement the skill 
            std::vector<AlgorithmPtr> implementation;

            virtual bool execute(State &current_state) = 0;

            // set the start state for the executor
            // virtual bool setStartState(const State &start_state) =0;

            // set the same pre conditions for meta skill and all atomic skills
            void set_pre_condition(TaskParamPtr pre_condition);

            // set the same post conditions for meta skill and all atomic skills
            void set_post_condition(TaskParamPtr post_condition);

            virtual void set_planned_trajectory(const RobotTrajectory &planned_trajectory);

            // properties
            TaskParamPtr pre_condition;
            TaskParamPtr post_condition;
            Skill::Type skill_type; // corresponding skill
            RobotTrajectory planned_trajectory_;
    };
    typedef std::shared_ptr<SkillExecutor> SkillExecutorPtr;

    class MetaSkillExecutor : public SkillExecutor {
        /*
        Class definition of MetaSkillExecutor, made of a sequence of atomic executors
        */
        public:
            MetaSkillExecutor() = default;
            MetaSkillExecutor(Skill::Type type, MetaSkill::ComposeType compose_type, const std::vector<AtomicSkillPtr> &atomic_skills);
            
            virtual bool execute(State &current_state) override;

            void add_atomic_executor(std::shared_ptr<SkillExecutor> atomic_executor);

            std::vector<SkillExecutorPtr> atomic_executors;
        


            // virtual bool setStartState(const State &start_state) override{
            //     controller_->setStartState(start_state);
            //     return true;
            // }
            


        protected:
            MetaSkill::ComposeType compose_type;
    };
    typedef std::shared_ptr<MetaSkillExecutor> MetaSkillExecutorPtr;
    
}