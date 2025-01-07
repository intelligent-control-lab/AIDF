#pragma once
#include "robots.hpp"
#include "object.hpp"
#include "Utils/FileIO.hpp"

namespace task_def
{
    // forward decalre
    class Activity;
    typedef std::shared_ptr<Activity> ActPtr;

    class ObjectNode {
    /* contains misc information about objects in a task graph */
    public:
        ObjectNode() = default;
        ObjectNode(const object::Object &obj, int id) : obj(obj), obj_node_id(id) {}

        std::string name() const {
            return obj.name;
        }

        object::Object obj;
        int obj_node_id;
        std::string next_attach_link;
        bool vanish = false; // vanish before attach or after detach
        bool handover = false; // used as a handover node
        ActPtr prev_detach;
        ActPtr next_attach;
    };
    typedef std::shared_ptr<ObjectNode> ObjNodePtr;

    struct EnvState {
        std::vector<object::Object> objects;
    };

    class SetCollisionNode {
    public:
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & obj_name;
            ar & link_name;
            ar & allow;
        }

        SetCollisionNode() = default;
        SetCollisionNode(const std::string &obj_name, const std::string &link_name, bool allow) 
            : obj_name(obj_name), link_name(link_name), allow(allow) {}
        std::string obj_name;
        std::string link_name;
        bool allow;
    };

    class Activity {
    public:
        enum Type {
            home = 0,
            pick_tilt_up = 1, // lego
            pick_up = 2, // lego
            pick_down = 3, // lego
            pick_twist = 4, // lego
            pick_twist_up = 5, // lego
            drop_tilt_up = 7, // lego
            drop_up = 8, // lego
            drop_down = 9, // lego
            drop_twist = 10, // lego
            drop_twist_up = 11, // lego
            support = 13, // lego
            support_pre = 14, // lego
            pick = 15,
            drop = 16,
            open_gripper = 17,
            close_gripper = 18,
            home_receive = 19, // lego
            receive = 20, // lego
            home_handover = 21, // lego
            handover_up = 22, // lego
            handover_down = 23, // lego
            handover_twist = 24, // lego
            handover_twist_up = 25, // lego
            place_tilt_down_pre = 26, // lego
            place_tilt_down = 27, // lego
            place_down = 28, // lego
            place_up = 29, // lego
            place_twist = 30, // lego
            place_twist_down = 31, // lego
            receive_place = 32, // lego
            press_up = 33, // lego
            press_down = 34, // lego
        };

        static const std::map<Type, std::string> enumStringMap;

        Activity() = default;
        Activity(int robot_id, Type type) : robot_id(robot_id), type(type) {}
        void add_type2_dep(std::shared_ptr<Activity> type2_dep) {
            this->type2_prev.push_back(type2_dep);
        }
        void add_type2_next(std::shared_ptr<Activity> type2_next) {
            this->type2_next.push_back(type2_next);
        }

        std::string type_string() const {
            return enumStringMap.at(type);
        }

        bool is_skippable() const {
            return type == Type::home || type == Type::home_handover;
        }

        int robot_id;
        int act_id;
        Type type;
        std::vector<std::shared_ptr<Activity>> type2_prev;
        std::vector<std::shared_ptr<Activity>> type2_next;
        std::shared_ptr<Activity> type1_prev;
        std::shared_ptr<Activity> type1_next;
        robot::RobotState start_pose;
        robot::RobotState end_pose;
        std::vector<ObjNodePtr> obj_detached;
        std::vector<ObjNodePtr> obj_attached;
        std::vector<SetCollisionNode> collision_nodes;
    };


    class ActivityGraph {
    public:
        ActivityGraph() = default;
        ActivityGraph(int num_robots);

        ActivityGraph(const ActivityGraph &other, int first_n_tasks);
        
        ActPtr add_act(int robot_id, Activity::Type type);

        ActPtr add_act(int robot_id, Activity::Type type, ActPtr type2_dep);
    
        /* add a static object to the scene (no attached parent)*/
        ObjNodePtr add_obj(const object::Object &obj);

        /* set the object node to be attached to a robot at the onset of selected activity */
        void attach_obj(ObjNodePtr obj, const std::string &link_name, ActPtr act);

        /* set the object node to be detached from a robot at the onset of selected activity */
        void detach_obj(ObjNodePtr obj, ActPtr act);

        void add_type2_dep(ActPtr act, ActPtr dep);

        /* enable or disable collision checking between the object_node and the robot at the onset of selected activity */
        void set_collision(const std::string &obj_name, const std::string &link_name, ActPtr act, bool allow);

        bool saveGraphToFile(const std::string &filename) const;

        ActPtr get(int robot_id, int act_id);
        std::shared_ptr<const Activity> get(int robot_id, int act_id) const;
        ActPtr get_last_act(int robot_id);
        ActPtr get_last_act(int robot_id, Activity::Type type);
        ObjNodePtr get_last_obj(const std::string &obj_name);

        int num_activities(int robot_id) const {
                    return activities_[robot_id].size();
        }

        int num_robots() const {
            return num_robots_;
        }
        
        std::vector<ObjNodePtr> get_obj_nodes() const {
            return obj_nodes_;
        }

        std::vector<ObjNodePtr> get_start_obj_nodes() const;
        std::vector<ObjNodePtr> get_end_obj_nodes() const;

        bool bfs(ActPtr act_i, std::vector<std::vector<bool>> &visited, bool forward) const;

        std::vector<ObjNodePtr> find_indep_obj(ActPtr act) const;

        void remove_act(int robot_id, int act_id);

    protected:
        int num_robots_ = 0;
        std::vector<std::vector<ActPtr>> activities_;
        std::vector<ObjNodePtr> obj_nodes_;
        
    };  

    class AssemblySeq {
    public:
        AssemblySeq() = default;
        virtual std::vector<ObjNodePtr> get_sequence();
        int num_tasks() {return num_tasks_;}
    
    protected:
        int num_tasks_;
        std::vector<ObjNodePtr> obj_seq_;
    };

    struct State {
        // robot state and env state
        // feasibility functions rely on lego state (cannot be task-agnostic?)
        // robot state (can be task-agnostic?)
        EnvState env_state;
        robot::RobotState robot_state;
        AssemblySeq assembly_state;
        int cur_step = 0;
    };


    class LegoAssemblySeq : public AssemblySeq {
    public:
        LegoAssemblySeq(lego_manipulation::lego::Lego::Ptr lego_ptr,
                        const std::string &task_json);
    
    private:
        lego_manipulation::lego::Lego::Ptr lego_ptr_;
        Json::Value task_json_;
        std::vector<ObjNodePtr> lego_seq_;
    };

}