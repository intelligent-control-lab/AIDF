#include "lego/lego_algorithms.hpp"
#include "tasks.hpp"
#include "Utils/Math.hpp"
#include "Utils/Logger.hpp"
#include "adg.h"


namespace skillgraph {

LegoGraspGenerator::LegoGraspGenerator(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
    std::shared_ptr<skillgraph::PlanInstance> instance,
    const LegoPolicyCfg &config, RobotPtr robot, ObjPtr object) 
    : lego_ptr_(lego_ptr), instance_(instance), config_(config), robot_(robot), object_(object) {
}

bool LegoGraspGenerator::generate(const Json::Value &constraint, Skill::Type type, int skill_seq,
         State &goal_state) {
    
    RobotState &robot_goal_state = goal_state.robot_states[robot_->robot_id];
    EnvState &env_state = goal_state.env_state;
    robot_goal_state = instance_->initRobotState(robot_->robot_id);
    
    // mandatory fields
    int brick_id = constraint["brick_id"].asInt();
    int brick_x = constraint["x"].asInt();
    int brick_y = constraint["y"].asInt();
    int brick_z = constraint["z"].asInt();
    int ori = constraint["ori"].asInt();
    int manip_type = constraint["manip_type"].asInt();
    bool sup_req = false;
    
    // optional fields
    int press_x, press_y, press_z, press_ori, press_side, press_offset;
    int support_x, support_y, support_z, support_ori;
    int attack_dir = -1;
    if (constraint.isMember("press_side")) {
        press_side = constraint["press_side"].asInt();
        press_offset = constraint["press_offset"].asInt();
    }
    if (constraint.isMember("press_x")) {
        press_x = constraint["press_x"].asInt();
        press_y = constraint["press_y"].asInt();
        press_z = constraint["press_z"].asInt();
        press_ori = constraint["press_ori"].asInt();
    }
    if (constraint.isMember("support_x")) {
        support_x = constraint["support_x"].asInt();
        support_y = constraint["support_y"].asInt();
        support_z = constraint["support_z"].asInt();
        support_ori = constraint["support_ori"].asInt();
        if (support_x != -1) {
            sup_req = true;
        }
    }
    if (constraint.isMember("attack_dir")) {
        attack_dir = constraint["attack_dir"].asInt();
    }

    // end-effector pose
    Eigen::MatrixXd home_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
    std::vector<double> home_state = robot_->home_state;
    home_q << home_state[0], home_state[1], home_state[2], home_state[3], home_state[4], home_state[5];
    home_q = home_q * 180.0 / M_PI;

    Eigen::Matrix4d cart_T = Eigen::Matrix4d::Identity();
    std::string brick_name = object_->name;
    bool reachable = true;

    Eigen::MatrixXd pick_offset = Eigen::MatrixXd::Zero(6, 1);
    pick_offset << -0.005, 0.005, -0.005,  // place brick offset
                   -0.005, 0.005, -0.0028; // grab brick offset

    Eigen::MatrixXd twist_R_pick = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd  twist_R_place = Eigen::MatrixXd::Identity(3, 3);
    twist_R_pick << cos(config_.twist_rad), 0, sin(config_.twist_rad), 
                0, 1, 0, 
                -sin(config_.twist_rad), 0, cos(config_.twist_rad);
    twist_R_place << cos(-config_.twist_rad), 0, sin(-config_.twist_rad), 
                0, 1, 0, 
                -sin(-config_.twist_rad), 0, cos(-config_.twist_rad);
    
    lego_manipulation::math::VectorJd goal_q = Eigen::MatrixXd::Zero(lego_ptr_->robot_dof_1(), 1);
    lego_manipulation::math::VectorJd seed_q = Eigen::MatrixXd::Zero(lego_ptr_->robot_dof_1(), 1);

    if (type == Skill::Type::Transit) {
        if (skill_seq == 0) {
            // goto pick_pre_pose
            lego_ptr_->brick_pose_in_stock(brick_name, press_side, press_offset, cart_T);
            
            Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
            //offset_T.col(3) << pick_offset(3), pick_offset(4), pick_offset(5) - abs(pick_offset(5)), 1;
            //offset_T = cart_T * offset_T;

            calculateIKforLego(cart_T, home_q, robot_->robot_id, 0, false, goal_q, robot_goal_state, reachable);
            // find the env state objects with the same name as current object
            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->state = Object::State::Attached;
                    break;
                }
            }
        }
        if (skill_seq == 2 && !sup_req) {
            // goto place_pre_pose
            lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z, press_ori, press_side, cart_T);

            Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
            //offset_T.col(3) << pick_offset(0), pick_offset(1) * attack_dir, pick_offset(2) - abs(pick_offset(2)), 1;
            //offset_T = cart_T * offset_T;

            calculateIKforLego(cart_T, home_q, robot_->robot_id, 1, false, goal_q, robot_goal_state, reachable);
            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->state = Object::State::Static;
                    break;
                }
            }
        }
        if (skill_seq == 4 && !sup_req) {
            // go to home pose
            robot_goal_state.joint_values = robot_->home_state;
        }
    }
    else if (type == Skill::Type::Pick) {
        lego_ptr_->brick_pose_in_stock(brick_name, press_side, press_offset, cart_T);
        calculateIKforLego(cart_T, home_q, robot_->robot_id, 0, false, goal_q, robot_goal_state, reachable);
        if (!reachable) {
            return false;
        }
        
        // twist the block
        Eigen::Matrix4d twist_T = Eigen::MatrixXd::Identity(4, 4);
        twist_T.block(0, 0, 3, 3) << twist_R_pick;

        if (robot_->robot_id == 0) {
            cart_T = lego_manipulation::math::FK(goal_q, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(goal_q, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        cart_T = cart_T * twist_T;
        seed_q = goal_q;
        calculateIKforLego(cart_T, seed_q, robot_->robot_id, 2, false, goal_q, robot_goal_state, reachable);

        if (!reachable) {
            return false;
        }

        // twist up the block
        if (robot_->robot_id == 0) {
            cart_T = lego_manipulation::math::FK(goal_q, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(goal_q, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        cart_T(2, 3) = cart_T(2, 3) + 0.015;
        seed_q = goal_q;
        calculateIKforLego(cart_T, seed_q, robot_->robot_id, 1, true, goal_q, robot_goal_state, reachable);

    }
    else if (type == Skill::Type::PlaceTop) {
        lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z, press_ori, press_side, cart_T);
        calculateIKforLego(cart_T, home_q, robot_->robot_id, 1, false, goal_q, robot_goal_state, reachable);
        if (!reachable) {
            return false;
        }

        // twist the block
        Eigen::Matrix4d twist_T = Eigen::MatrixXd::Identity(4, 4);
        twist_T.block(0, 0, 3, 3) << twist_R_place;
        if (robot_->robot_id == 0) {
            cart_T = lego_manipulation::math::FK(goal_q, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(goal_q, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        cart_T = cart_T * twist_T;
        cart_T(2, 3) = cart_T(2, 3) + 0.015;
        seed_q = goal_q;
        calculateIKforLego(cart_T, seed_q, robot_->robot_id, 1, true, goal_q, robot_goal_state, reachable);
    }
    else if (type == Skill::Type::PlaceBottom) {

    }
    else if (type == Skill::Type::SupportBottom) {

    }
    else if (type == Skill::Type::SupportTop) {

    }
    else if (type == Skill::Type::Handover) {

    }
    else if (type == Skill::Type::PlaceWithSupport || type == Skill::Type::PickAndPlace 
        || type == Skill::Type::PickAndPlaceWithSupport || type == Skill::Type::PickHandoverAndPlace) {
            throw std::runtime_error("Unsupported meta skill " + std::to_string(int(type)));
    }
    else {
        throw std::runtime_error("Unknown skill type: " + std::to_string(int(type)));
    }

    return true;
}


void LegoGraspGenerator::calculateIKforLego(const Eigen::MatrixXd& T, const Eigen::MatrixXd & home_q,
        int robot_id, int fk_type, bool check_collision, lego_manipulation::math::VectorJd &joint_q, 
        RobotState &robot_state, bool &reachable) {
    if (!reachable) {
        return;
    }

    //auto tic = std::chrono::high_resolution_clock::now();

    if (robot_id == 0) {
        if (fk_type == 0) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);
        }
        else if (fk_type == 1) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_assemble_inv_r1(), 0, IK_status_);
        }
        else if (fk_type == 2) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_disassemble_inv_r1(), 0, IK_status_);
        }
        else if (fk_type == 3) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_inv_r1(), 0, IK_status_);
        }
        else if (fk_type == 4) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_assemble_inv_r1(), 0, IK_status_);
        }
    }
    else {
        if (fk_type == 0) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_inv_r2(), 0, IK_status_);
        }
        else if (fk_type == 1) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_assemble_inv_r2(), 0, IK_status_);
        }
        else if (fk_type == 2) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_disassemble_inv_r2(), 0, IK_status_);
        }
        else if (fk_type == 3) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_inv_r2(), 0, IK_status_);
        }
        else if (fk_type == 4) {
            joint_q = lego_ptr_->IK(home_q, T, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_assemble_inv_r2(), 0, IK_status_);
        }
    }

    reachable = IK_status_;
    robot_state = instance_->initRobotState(robot_id);
    for (int i = 0; i < 6; i++) {
        robot_state.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
    }

    // if (check_collision && reachable) {
    //     bool hasCollision = instance_->checkCollision({robot_state}, false);
    //     reachable &= !hasCollision;
    //     std::cout << "Collision: " << hasCollision << std::endl;
    // }
    // auto toc = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count() / 1000.0;
    // ik_reachability_time_ += duration;

}

}