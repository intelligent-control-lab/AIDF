/*
**********************************************************************************************************************
Ontology and Skill Graph for Autonomous Multi-Robot Assembly
AI Data Foundry (AIDF) Project

Copyright (c) 2025
Carnegie Mellon University
ARM Institute – Advanced Robotics for Manufacturing

Authors:
    Philip Huang philiphuang@cmu.edu
    Peiqi Yu peiqiy@andrew.cmu.edu
    Chaitanya Chawla cchawla@cs.cmu.edu

Non-Commercial Research License:
Permission is hereby granted to use, copy, modify, and distribute this Software for non-commercial research and
educational purposes only, provided that the above copyright notice and this permission notice appear in all
copies or substantial portions of the Software.

Commercial use of this Software, in whole or in part, requires explicit written permission from Carnegie Mellon
University and the ARM Institute.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
**********************************************************************************************************************
*/

#include "skillplan.hpp"

#include "Utils/Logger.hpp"
#include "lego/lego_objects.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>

namespace skillgraph::skillplan {

namespace {

std::string nowUtcIso8601() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &tt);
#else
    gmtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%FT%TZ");
    return oss.str();
}

bool nearlyEqual(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

std::string objectStateToString(Object::State state) {
    switch (state) {
        case Object::State::Static:
            return "Static";
        case Object::State::Attached:
            return "Attached";
        case Object::State::Supported:
            return "Supported";
        case Object::State::Handover:
            return "Handover";
        default:
            return "Unknown";
    }
}

std::string objectShapeToString(Object::Shape shape) {
    switch (shape) {
        case Object::Shape::Box:
            return "Box";
        case Object::Shape::Sphere:
            return "Sphere";
        case Object::Shape::Cylinder:
            return "Cylinder";
        case Object::Shape::Mesh:
            return "Mesh";
        default:
            return "Unknown";
    }
}

std::string objectTypeToString(Object::Type type) {
    switch (type) {
        case Object::Type::Unknown:
            return "Unknown";
        case Object::Type::LegoBrick:
            return "LegoBrick";
        case Object::Type::MagneticBlock:
            return "MagneticBlock";
        default:
            return "Unknown";
    }
}

Json::Value vecToJson(const std::vector<double> &v) {
    Json::Value arr(Json::arrayValue);
    for (const auto &x : v) {
        arr.append(x);
    }
    return arr;
}

Json::Value poseJson(double x, double y, double z, double qx, double qy, double qz, double qw) {
    Json::Value pose;
    pose["position"]["x"] = x;
    pose["position"]["y"] = y;
    pose["position"]["z"] = z;
    pose["orientation"]["x"] = qx;
    pose["orientation"]["y"] = qy;
    pose["orientation"]["z"] = qz;
    pose["orientation"]["w"] = qw;
    return pose;
}

Json::Value robotStateToJson(const RobotState &rs, size_t joint_limit, size_t hand_limit) {
    Json::Value out;
    out["robot_id"] = rs.robot_id;
    out["robot_name"] = rs.robot_name;
    if (joint_limit == 0 || joint_limit >= rs.joint_values.size()) {
        out["joint_positions"] = vecToJson(rs.joint_values);
    } else {
        out["joint_positions"] = vecToJson(std::vector<double>(rs.joint_values.begin(), rs.joint_values.begin() + joint_limit));
    }
    if (!rs.hand_values.empty()) {
        if (hand_limit == 0 || hand_limit >= rs.hand_values.size()) {
            out["hand_positions"] = vecToJson(rs.hand_values);
        } else {
            out["hand_positions"] = vecToJson(std::vector<double>(rs.hand_values.begin(), rs.hand_values.begin() + hand_limit));
        }
    }
    return out;
}

Json::Value objectToJson(const Object &obj) {
    Json::Value out;
    out["name"] = obj.name;
    out["type"] = objectTypeToString(obj.type);
    out["state"] = objectStateToString(obj.state);
    out["parent_link"] = obj.parent_link;
    out["robot_id"] = obj.robot_id;
    out["shape"] = objectShapeToString(obj.shape);
    out["pose"] = poseJson(obj.x, obj.y, obj.z, obj.qx, obj.qy, obj.qz, obj.qw);

    if (obj.state == Object::State::Attached) {
        out["attach_pose"] =
            poseJson(obj.x_attach, obj.y_attach, obj.z_attach, obj.qx_attach, obj.qy_attach, obj.qz_attach, obj.qw_attach);
    }

    Json::Value dims;
    if (obj.shape == Object::Shape::Box) {
        dims["x"] = obj.length;
        dims["y"] = obj.width;
        dims["z"] = obj.height;
    } else if (obj.shape == Object::Shape::Cylinder) {
        dims["height"] = obj.length;
        dims["radius"] = obj.radius;
    } else if (obj.shape == Object::Shape::Sphere) {
        dims["radius"] = obj.radius;
    }
    if (!dims.isNull() && !dims.empty()) {
        out["dimensions"] = dims;
    }
    if (!obj.mesh_path.empty()) {
        out["mesh_path"] = obj.mesh_path;
    }

    // Lego-specific extras when available
    if (const auto *lego = dynamic_cast<const LegoBrick *>(&obj)) {
        Json::Value lego_json;
        lego_json["brick_id"] = lego->brick_id;
        lego_json["in_storage"] = lego->in_storage;
        lego_json["fixed"] = lego->fixed;
        out["lego"] = lego_json;
    }

    return out;
}

Json::Value objectPtrToJson(const ObjPtr &obj) {
    if (!obj) {
        return Json::Value();
    }
    return objectToJson(*obj);
}

Json::Value trajectoryToJson(const RobotTrajectory &traj,
                             int fallback_robot_id,
                             const std::string &fallback_robot_name,
                             size_t joint_limit,
                             size_t hand_limit) {
    Json::Value out;
    const int robot_id = (traj.robot_id >= 0) ? traj.robot_id : fallback_robot_id;
    out["robot_id"] = robot_id;
    if (!traj.trajectory.empty() && !traj.trajectory.front().robot_name.empty()) {
        out["robot_name"] = traj.trajectory.front().robot_name;
    } else if (!fallback_robot_name.empty()) {
        out["robot_name"] = fallback_robot_name;
    }

    Json::Value points(Json::arrayValue);
    const size_t n = traj.trajectory.size();
    for (size_t i = 0; i < n; ++i) {
        const RobotState &rs = traj.trajectory[i];
        Json::Value p;
        if (i < traj.times.size()) {
            p["t"] = traj.times[i];
        }
        if (joint_limit == 0 || joint_limit >= rs.joint_values.size()) {
            p["joint_positions"] = vecToJson(rs.joint_values);
        } else {
            p["joint_positions"] = vecToJson(std::vector<double>(rs.joint_values.begin(), rs.joint_values.begin() + joint_limit));
        }
        if (!rs.hand_values.empty()) {
            if (hand_limit == 0 || hand_limit >= rs.hand_values.size()) {
                p["hand_positions"] = vecToJson(rs.hand_values);
            } else {
                p["hand_positions"] = vecToJson(std::vector<double>(rs.hand_values.begin(), rs.hand_values.begin() + hand_limit));
            }
        }
        if (i < traj.act_ids.size()) {
            p["act_id"] = traj.act_ids[i];
        }
        points.append(p);
    }
    out["points"] = points;

    if (!traj.times.empty()) {
        out["times"] = vecToJson(traj.times);
    }
    if (!traj.act_ids.empty()) {
        Json::Value act_ids(Json::arrayValue);
        for (const auto &id : traj.act_ids) {
            act_ids.append(id);
        }
        out["act_ids"] = act_ids;
    }
    if (!nearlyEqual(traj.cost, 0.0, 0.0)) {
        out["cost"] = traj.cost;
    }
    return out;
}

std::string sanitizeIdComponent(const std::string &in) {
    std::string out;
    out.reserve(in.size());
    for (const char c : in) {
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
            out.push_back(c);
        } else {
            out.push_back('_');
        }
    }
    while (!out.empty() && out.front() == '_') {
        out.erase(out.begin());
    }
    while (!out.empty() && out.back() == '_') {
        out.pop_back();
    }
    if (out.empty()) {
        return "x";
    }
    return out;
}

bool objectsEquivalent(const Object &a, const Object &b, double tol) {
    if (a.name != b.name) {
        return false;
    }
    if (a.type != b.type || a.state != b.state || a.parent_link != b.parent_link || a.robot_id != b.robot_id) {
        return false;
    }
    if (a.shape != b.shape) {
        return false;
    }
    if (!nearlyEqual(a.x, b.x, tol) || !nearlyEqual(a.y, b.y, tol) || !nearlyEqual(a.z, b.z, tol) ||
        !nearlyEqual(a.qx, b.qx, tol) || !nearlyEqual(a.qy, b.qy, tol) || !nearlyEqual(a.qz, b.qz, tol) ||
        !nearlyEqual(a.qw, b.qw, tol)) {
        return false;
    }

    if (!nearlyEqual(a.x_attach, b.x_attach, tol) || !nearlyEqual(a.y_attach, b.y_attach, tol) ||
        !nearlyEqual(a.z_attach, b.z_attach, tol) || !nearlyEqual(a.qx_attach, b.qx_attach, tol) ||
        !nearlyEqual(a.qy_attach, b.qy_attach, tol) || !nearlyEqual(a.qz_attach, b.qz_attach, tol) ||
        !nearlyEqual(a.qw_attach, b.qw_attach, tol)) {
        return false;
    }

    if (!nearlyEqual(a.radius, b.radius, tol) || !nearlyEqual(a.length, b.length, tol) || !nearlyEqual(a.width, b.width, tol) ||
        !nearlyEqual(a.height, b.height, tol) || a.mesh_path != b.mesh_path) {
        return false;
    }

    const auto *lego_a = dynamic_cast<const LegoBrick *>(&a);
    const auto *lego_b = dynamic_cast<const LegoBrick *>(&b);
    if ((lego_a != nullptr) != (lego_b != nullptr)) {
        return false;
    }
    if (lego_a && lego_b) {
        if (lego_a->brick_id != lego_b->brick_id || lego_a->in_storage != lego_b->in_storage || lego_a->fixed != lego_b->fixed) {
            return false;
        }
    }

    return true;
}

struct SceneDiff {
    std::vector<ObjPtr> added;
    std::vector<std::string> removed;
    std::vector<ObjPtr> updated;
    Json::Value attachments = Json::Value(Json::arrayValue);
    Json::Value collisions_allow = Json::Value(Json::arrayValue);
    Json::Value collisions_disallow = Json::Value(Json::arrayValue);
};

SceneDiff diffScene(const EnvState &prev_env, const EnvState &next_env, double tol) {
    SceneDiff diff;

    std::map<std::string, ObjPtr> prev_by_name;
    std::map<std::string, ObjPtr> next_by_name;

    for (const auto &obj : prev_env.objects) {
        if (obj) {
            prev_by_name[obj->name] = obj;
        }
    }
    for (const auto &obj : next_env.objects) {
        if (obj) {
            next_by_name[obj->name] = obj;
        }
    }

    std::set<std::string> names;
    for (const auto &[name, _] : prev_by_name) {
        names.insert(name);
    }
    for (const auto &[name, _] : next_by_name) {
        names.insert(name);
    }

    for (const auto &name : names) {
        const auto prev_it = prev_by_name.find(name);
        const auto next_it = next_by_name.find(name);

        if (prev_it == prev_by_name.end() && next_it != next_by_name.end()) {
            diff.added.push_back(next_it->second);
            continue;
        }
        if (prev_it != prev_by_name.end() && next_it == next_by_name.end()) {
            diff.removed.push_back(name);
            continue;
        }

        const ObjPtr &prev_obj = prev_it->second;
        const ObjPtr &next_obj = next_it->second;
        if (!prev_obj || !next_obj) {
            continue;
        }

        // Attachment transitions (only for Attached state; other states are captured via object updates).
        if (prev_obj->state != Object::State::Attached && next_obj->state == Object::State::Attached) {
            Json::Value evt;
            evt["action"] = "attach";
            evt["object"] = next_obj->name;
            evt["robot_id"] = next_obj->robot_id;
            evt["link"] = next_obj->parent_link;
            evt["relative_pose"] = poseJson(next_obj->x_attach,
                                            next_obj->y_attach,
                                            next_obj->z_attach,
                                            next_obj->qx_attach,
                                            next_obj->qy_attach,
                                            next_obj->qz_attach,
                                            next_obj->qw_attach);
            diff.attachments.append(evt);

            Json::Value allow;
            allow["object"] = next_obj->name;
            allow["link"] = next_obj->parent_link;
            diff.collisions_allow.append(allow);
        } else if (prev_obj->state == Object::State::Attached && next_obj->state != Object::State::Attached) {
            Json::Value evt;
            evt["action"] = "detach";
            evt["object"] = prev_obj->name;
            evt["robot_id"] = prev_obj->robot_id;
            evt["link"] = prev_obj->parent_link;
            evt["world_pose"] = poseJson(next_obj->x, next_obj->y, next_obj->z, next_obj->qx, next_obj->qy, next_obj->qz, next_obj->qw);
            diff.attachments.append(evt);

            Json::Value disallow;
            disallow["object"] = prev_obj->name;
            disallow["link"] = prev_obj->parent_link;
            diff.collisions_disallow.append(disallow);
        }

        if (!objectsEquivalent(*prev_obj, *next_obj, tol)) {
            diff.updated.push_back(next_obj);
        }
    }

    return diff;
}

Json::Value minimalObjectRef(const ObjPtr &obj) {
    Json::Value out;
    if (!obj) {
        return out;
    }
    out["name"] = obj->name;
    out["type"] = objectTypeToString(obj->type);
    if (const auto *lego = dynamic_cast<const LegoBrick *>(obj.get())) {
        out["brick_id"] = lego->brick_id;
    }
    return out;
}

} // namespace

Json::Value export_plan(const SkillGraph &skillgraph,
                        const std::vector<SkillPtr> &plan_skill_seq,
                        const State &initial_state,
                        const ExportOptions &options) {
    Json::Value root;
    root["schema"] = "aidf.skillplan";
    root["schema_version"] = 1;

    root["source"]["generator"] = "plan_lego";
    root["source"]["created_utc"] = nowUtcIso8601();
    root["source"]["skillgraph_config"] = skillgraph.get_config_filename();

    const auto env = skillgraph.get_environment();
    if (env) {
        root["environment"]["name"] = env->name;
        root["environment"]["type"] = (env->type == Environment::Type::Lego) ? "Lego" : "NIST";
        if (options.include_environment_spec) {
            root["environment"]["spec"] = env->spec_;
        }
        if (env->spec_.isMember("backend")) {
            root["environment"]["backend"] = env->spec_["backend"];
        }
    }

    // Robots (from skillgraph definitions, stable ordering by robot_id)
    {
        Json::Value robots_json(Json::arrayValue);
        std::vector<std::string> robot_names = skillgraph.get_robot_names();

        struct RobEntry {
            int id;
            Json::Value json;
        };
        std::vector<RobEntry> entries;
        entries.reserve(robot_names.size());

        for (const auto &name : robot_names) {
            RobotPtr robot = skillgraph.get_robot(name);
            Json::Value r;
            r["id"] = robot->robot_id;
            r["name"] = robot->robot_name;
            r["type"] = robot->type_string();
            r["tool"] = robot->tool_string();
            r["dof"] = robot->robot_dof;
            r["hand_dof"] = robot->hand_dof;
            r["end_effector_link"] = robot->end_effector_link;
            if (!robot->home_state.empty()) {
                r["home_joint_positions"] = vecToJson(robot->home_state);
            }
            Json::Value caps(Json::arrayValue);
            for (const auto &cap : robot->capabilities) {
                caps.append(cap);
            }
            r["capabilities"] = caps;

            entries.push_back({robot->robot_id, r});
        }

        std::sort(entries.begin(), entries.end(), [](const RobEntry &a, const RobEntry &b) { return a.id < b.id; });
        for (const auto &entry : entries) {
            robots_json.append(entry.json);
        }

        root["robots"] = robots_json;
    }

    std::map<int, size_t> joint_dof_by_robot_id;
    std::map<int, size_t> hand_dof_by_robot_id;
    for (const auto &name : skillgraph.get_robot_names()) {
        RobotPtr robot = skillgraph.get_robot(name);
        joint_dof_by_robot_id[robot->robot_id] = static_cast<size_t>(robot->robot_dof);
        hand_dof_by_robot_id[robot->robot_id] = static_cast<size_t>(robot->hand_dof);
    }

    // Initial scene snapshot
    {
        Json::Value init;

        Json::Value rs_arr(Json::arrayValue);
        std::vector<RobotState> rs_sorted = initial_state.robot_states;
        std::sort(rs_sorted.begin(), rs_sorted.end(), [](const RobotState &a, const RobotState &b) { return a.robot_id < b.robot_id; });
        for (const auto &rs : rs_sorted) {
            const size_t joint_limit = joint_dof_by_robot_id.count(rs.robot_id) ? joint_dof_by_robot_id.at(rs.robot_id) : 0;
            const size_t hand_limit = hand_dof_by_robot_id.count(rs.robot_id) ? hand_dof_by_robot_id.at(rs.robot_id) : 0;
            rs_arr.append(robotStateToJson(rs, joint_limit, hand_limit));
        }
        init["robots"] = rs_arr;

        Json::Value obj_arr(Json::arrayValue);
        std::vector<ObjPtr> objs = initial_state.env_state.objects;
        std::sort(objs.begin(), objs.end(), [](const ObjPtr &a, const ObjPtr &b) {
            if (!a) {
                return true;
            }
            if (!b) {
                return false;
            }
            return a->name < b->name;
        });
        for (const auto &obj : objs) {
            if (obj) {
                obj_arr.append(objectPtrToJson(obj));
            }
        }
        init["objects"] = obj_arr;

        root["initial_scene"] = init;
    }

    Json::Value actions(Json::arrayValue);

    const EnvState *prev_env = &initial_state.env_state;
    std::string prev_action_id;
    int global_action_idx = 0;

    for (size_t meta_idx = 0; meta_idx < plan_skill_seq.size(); ++meta_idx) {
        const SkillPtr &skill = plan_skill_seq[meta_idx];
        const auto meta = std::dynamic_pointer_cast<MetaSkill>(skill);
        if (!meta) {
            throw std::runtime_error("SkillPlan export expects meta skills; got non-meta skill at index " + std::to_string(meta_idx));
        }

        for (size_t atomic_idx = 0; atomic_idx < meta->atomic_skills.size(); ++atomic_idx) {
            const AtomicSkillPtr &atomic = meta->atomic_skills[atomic_idx];
            if (!atomic) {
                throw std::runtime_error("Null atomic skill at meta index " + std::to_string(meta_idx) + ", atomic index " +
                                         std::to_string(atomic_idx));
            }
            if (!atomic->robot) {
                throw std::runtime_error("Atomic skill " + atomic->name + " has null robot (meta: " + meta->name + ")");
            }
            if (!atomic->executor) {
                throw std::runtime_error("Atomic skill " + atomic->name + " has null executor (meta: " + meta->name + ")");
            }

            const SkillExecutorPtr exec = atomic->executor;
            if (!exec->post_condition) {
                throw std::runtime_error("Atomic executor missing post_condition for " + atomic->name + " (meta: " + meta->name + ")");
            }

            const State &target_state = exec->post_condition->target_state;
            const int robot_id = atomic->robot->robot_id;
            if (robot_id < 0 || static_cast<size_t>(robot_id) >= target_state.robot_states.size()) {
                throw std::runtime_error("Target state missing robot state for robot_id " + std::to_string(robot_id) + " (skill " +
                                         atomic->name + ")");
            }

            const RobotState &goal_rs = target_state.robot_states[robot_id];
            const RobotTrajectory &traj = exec->planned_trajectory_;

            const bool has_goal = !goal_rs.joint_values.empty() || !goal_rs.hand_values.empty();
            const bool has_traj = !traj.trajectory.empty();
            if (!has_goal && !has_traj) {
                throw std::runtime_error("SkillPlan export requires at least a joint goal or trajectory; none available for skill " +
                                         atomic->name + " (meta: " + meta->name + ")");
            }

            Json::Value action;

            std::ostringstream id_ss;
            id_ss << "a" << std::setw(4) << std::setfill('0') << global_action_idx;
            const std::string action_id = id_ss.str();
            action["id"] = action_id;
            action["task_index"] = static_cast<int>(meta_idx) + 1;

            action["meta_skill"]["name"] = meta->name;
            action["meta_skill"]["index"] = static_cast<int>(meta_idx);
            action["skill"]["name"] = atomic->name;
            action["skill"]["index_in_meta"] = static_cast<int>(atomic_idx);

            action["robot"]["id"] = robot_id;
            action["robot"]["name"] = atomic->robot->robot_name;
            if (!atomic->robot->end_effector_link.empty()) {
                action["robot"]["end_effector_link"] = atomic->robot->end_effector_link;
            }

            if (atomic->object) {
                action["object"] = minimalObjectRef(atomic->object);
            }

            // Dependencies (M1: sequential)
            Json::Value deps(Json::arrayValue);
            if (!prev_action_id.empty()) {
                deps.append(prev_action_id);
            }
            action["depends_on"] = deps;

            // Goal (joint-space; EE pose optional in future milestones)
            Json::Value goal;
            const size_t joint_limit = joint_dof_by_robot_id.count(robot_id) ? joint_dof_by_robot_id.at(robot_id) : 0;
            const size_t hand_limit = hand_dof_by_robot_id.count(robot_id) ? hand_dof_by_robot_id.at(robot_id) : 0;
            if (!goal_rs.joint_values.empty()) {
                if (joint_limit == 0 || joint_limit >= goal_rs.joint_values.size()) {
                    goal["joint_positions"] = vecToJson(goal_rs.joint_values);
                } else {
                    goal["joint_positions"] =
                        vecToJson(std::vector<double>(goal_rs.joint_values.begin(), goal_rs.joint_values.begin() + joint_limit));
                }
            }
            if (!goal_rs.hand_values.empty()) {
                if (hand_limit == 0 || hand_limit >= goal_rs.hand_values.size()) {
                    goal["hand_positions"] = vecToJson(goal_rs.hand_values);
                } else {
                    goal["hand_positions"] =
                        vecToJson(std::vector<double>(goal_rs.hand_values.begin(), goal_rs.hand_values.begin() + hand_limit));
                }
            }
            action["goal"] = goal;

            if (has_traj) {
                action["trajectory"] = trajectoryToJson(traj, robot_id, atomic->robot->robot_name, joint_limit, hand_limit);
            } else {
                action["trajectory"] = Json::Value();
            }

            if (options.include_action_constraints) {
                action["constraints"] = exec->post_condition->constraints_json;
            }

            // Scene updates (diff env state across atomic steps)
            SceneDiff scene_diff = diffScene(*prev_env, target_state.env_state, options.float_tolerance);
            Json::Value scene_updates;
            scene_updates["attachments"] = scene_diff.attachments;
            scene_updates["collisions_allow"] = scene_diff.collisions_allow;
            scene_updates["collisions_disallow"] = scene_diff.collisions_disallow;

            Json::Value objects;
            Json::Value add_arr(Json::arrayValue);
            for (const auto &obj : scene_diff.added) {
                add_arr.append(objectPtrToJson(obj));
            }
            Json::Value rm_arr(Json::arrayValue);
            for (const auto &name : scene_diff.removed) {
                rm_arr.append(name);
            }
            Json::Value upd_arr(Json::arrayValue);
            for (const auto &obj : scene_diff.updated) {
                upd_arr.append(objectPtrToJson(obj));
            }
            objects["add"] = add_arr;
            objects["remove"] = rm_arr;
            objects["update"] = upd_arr;
            scene_updates["objects"] = objects;
            action["scene_updates"] = scene_updates;

            // Optional human-friendly stable key (useful for debugging/grep)
            std::ostringstream key_ss;
            key_ss << "t" << std::setw(3) << std::setfill('0') << (static_cast<int>(meta_idx) + 1) << "_";
            key_ss << sanitizeIdComponent(meta->name) << "_";
            key_ss << "s" << std::setw(2) << std::setfill('0') << static_cast<int>(atomic_idx) << "_";
            key_ss << sanitizeIdComponent(atomic->name) << "_r" << robot_id;
            action["key"] = key_ss.str();

            actions.append(action);
            prev_env = &target_state.env_state;
            prev_action_id = action_id;
            ++global_action_idx;
        }
    }

    root["actions"] = actions;

    root["summary"]["num_meta_skills"] = static_cast<int>(plan_skill_seq.size());
    root["summary"]["num_actions"] = global_action_idx;
    root["summary"]["num_robots"] = static_cast<int>(skillgraph.get_robot_names().size());

    // Keep a small pointer to original task config (complimentary to skillgraph.json)
    const Json::Value &root_cfg = skillgraph.get_root_config();
    if (root_cfg.isMember("tasks")) {
        root["tasks"] = root_cfg["tasks"];
    }

    return root;
}

bool write_json_to_file(const Json::Value &json, const std::string &path, std::string *error) {
    try {
        std::filesystem::path out_path(path);
        if (out_path.has_parent_path()) {
            std::filesystem::create_directories(out_path.parent_path());
        }

        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        builder["commentStyle"] = "None";
        builder["enableYAMLCompatibility"] = false;

        std::ofstream out(path);
        if (!out.is_open()) {
            if (error) {
                *error = "Failed to open output file: " + path;
            }
            return false;
        }
        out << Json::writeString(builder, json);
        out.close();
        return true;
    } catch (const std::exception &e) {
        if (error) {
            *error = e.what();
        }
        return false;
    }
}

} // namespace skillgraph::skillplan
