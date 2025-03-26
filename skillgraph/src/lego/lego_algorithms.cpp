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

bool LegoGraspGenerator::calculateHandoverPoses(int robot_id, std::vector<RobotState> &handover_goal, Eigen::MatrixXd &receive_q) {
    // calculate handover pose for the support arm robot id
    lego_manipulation::math::VectorJd r_transfer_up_goal, r_transfer_down_goal, r_transfer_twist_goal, r_transfer_twist_up_goal;
    RobotState transfer_up_state, transfer_down_state, transfer_twist_state, transfer_twist_up_state;
    Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);

    log("Calculating handover poses for robot " + std::to_string(robot_id+1), LogLevel::DEBUG);

    if (robot_id == 0) {
        // the other robot is receiving, the hadover robot needs to place at the receiving robot
        cart_T = lego_manipulation::math::FK(receive_q, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), false);
    }
    else {
        cart_T = lego_manipulation::math::FK(receive_q, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), false);
    }

    Eigen::MatrixXd y_s90 = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd z_180 = Eigen::MatrixXd::Identity(4, 4);
    y_s90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    cart_T = cart_T * y_s90 * z_180;
    
    Eigen::Matrix4d up_T = cart_T;
    up_T(2, 3) = up_T(2, 3) + 0.015;

    bool reachable = true;

    // transfer up
    // first a base home handover q
    auto home_handover_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1); 
    home_handover_q.col(0) << 0, 0, 0, 0, -90, 0; 
    Eigen::Matrix4d home_handover_T = lego_manipulation::math::FK(home_handover_q, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
    home_handover_T.col(3) << 0.2, 0, 0.5, 1;
    home_handover_T = lego_ptr_->world_base_frame() * home_handover_T; 
    home_handover_q = lego_ptr_->IK(home_handover_q, home_handover_T, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_ee_inv_r1(), 0, IK_status_);
    
    calculateIKforLego(up_T, home_handover_q, robot_id, 0, true, r_transfer_up_goal, transfer_up_state, reachable);

    if (reachable) {
        // transfer down
        calculateIKforLego(cart_T, r_transfer_up_goal, robot_id, 0, true, r_transfer_down_goal, transfer_down_state, reachable);
    }
    if (reachable) {
        // transfer twist
        if (robot_id == 0) {
            cart_T = lego_manipulation::math::FK(r_transfer_down_goal, lego_ptr_->robot_DH_tool_handover_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(r_transfer_down_goal, lego_ptr_->robot_DH_tool_handover_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::MatrixXd twist_R_handover = Eigen::MatrixXd::Identity(3, 3);
        twist_R_handover << cos(-config_.twist_rad_handover), 0, sin(-config_.twist_rad_handover), 
                0, 1, 0, 
                -sin(-config_.twist_rad_handover), 0, cos(-config_.twist_rad_handover);
        twist_T.block(0, 0, 3, 3) << twist_R_handover;
        cart_T = cart_T * twist_T;
        calculateIKforLego(cart_T, r_transfer_down_goal, robot_id, 1, true, r_transfer_twist_goal, transfer_twist_state, reachable);
    }
    if (reachable) {
        // transfer twist up
        if (robot_id == 0) {
            cart_T = lego_manipulation::math::FK(r_transfer_twist_goal, lego_ptr_->robot_DH_tool_handover_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(r_transfer_twist_goal, lego_ptr_->robot_DH_tool_handover_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        Eigen::Matrix4d up_T = cart_T;
        up_T(2, 3) = up_T(2, 3) + 0.015;
        calculateIKforLego(up_T, r_transfer_twist_goal, robot_id, 1, true, r_transfer_twist_up_goal, transfer_twist_up_state, reachable);
    }
    handover_goal = {transfer_up_state, transfer_down_state, transfer_twist_state, transfer_twist_up_state};

    log("Handover poses for robot " + std::to_string(robot_id+1) + " is " + std::to_string(reachable), LogLevel::DEBUG);
    return reachable;
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
    int manip_type = constraint["manipulate_type"].asInt();
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

    // local helper variable to determine the skill type
    std::string meta_skill_type = "pickplace";
    if (manip_type == 1) {
        meta_skill_type = "pickhandoverplace";
    }
    else if (sup_req) {
        meta_skill_type = "pickplacewithsupport";
    }

    // end-effector pose
    Eigen::MatrixXd home_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
    std::vector<double> home_state = robot_->home_state;
    home_q << home_state[0], home_state[1], home_state[2], home_state[3], home_state[4], home_state[5];
    home_q = home_q * 180.0 / M_PI;

    Eigen::MatrixXd receive_q = Eigen::MatrixXd(lego_ptr_->robot_dof_1(), 1);
    receive_q << 0, 0, 0, 0, 0, 180;

    Eigen::MatrixXd y_s90 = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd z_180 = Eigen::MatrixXd::Identity(4, 4);
    y_s90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
    z_180 << -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4d receive_T = lego_manipulation::math::FK(receive_q, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), false);
    receive_T.col(3) << 0.45, 0, 0.35, 1;
    receive_T = lego_ptr_->world_base_frame() * receive_T; 
    receive_q = lego_ptr_->IK(receive_q, receive_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_);
    if (!IK_status_) {
        return false;
    }


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
            if (!reachable) {
                return false;
            }
            // find the env state objects with the same name as current object
            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->parent_link = robot_->end_effector_link;
                    obj->robot_id = robot_->robot_id;
                    obj->state = Object::State::Attached;
                    instance_->computeRelativeTransform(*obj, robot_goal_state);
                    std::dynamic_pointer_cast<LegoBrick>(obj)->in_storage = false;
                    break;
                }
            }
        }
        if ((meta_skill_type == "pickplace" && skill_seq == 2) 
            || (meta_skill_type == "pickplacewithsupport" && skill_seq == 5)
            || (meta_skill_type == "pickhandoverplace" && skill_seq == 7)) {
            // goto place_pre_pose
            lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z, press_ori, press_side, cart_T);

            Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
            //offset_T.col(3) << pick_offset(0), pick_offset(1) * attack_dir, pick_offset(2) - abs(pick_offset(2)), 1;
            //offset_T = cart_T * offset_T;

            calculateIKforLego(cart_T, home_q, robot_->robot_id, 0, false, goal_q, robot_goal_state, reachable);
            if (!reachable) {
                return false;
            }
            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->state = Object::State::Static;
                    obj->parent_link = "world";
                    Eigen::Matrix4d brick_pose_mtx;
                    lego_ptr_->calc_bric_asssemble_pose(brick_name, brick_x, brick_y, brick_z, ori, brick_pose_mtx);
                    obj->x = brick_pose_mtx(0, 3);
                    obj->y = brick_pose_mtx(1, 3);
                    obj->z = brick_pose_mtx(2, 3) - obj->height/2;
                    // update orientation
                    Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
                    obj->qx = quat.x();
                    obj->qy = quat.y();
                    obj->qz = quat.z();
                    obj->qw = quat.w();
                    break;
                }
            }
        }
        if (meta_skill_type == "pickplacewithsupport" && skill_seq == 3) {
            // go to support_pre pose
            Eigen::Matrix4d sup_T = Eigen::MatrixXd::Identity(4, 4);

            lego_ptr_->support_pose(support_x, support_y, support_z, support_ori, sup_T);
            sup_T(2, 3) = sup_T(2, 3) + config_.pre_support_z_offset;
            calculateIKforLego(sup_T, home_q, robot_->robot_id, 0, true, goal_q, robot_goal_state, reachable);
            if (!reachable) {
                return false;
            }
        }
        if (meta_skill_type == "pickhandoverplace" && skill_seq == 3) {
            // goto receive pose
            for (int i = 0; i < 6; i++) {
                robot_goal_state.joint_values[i] = receive_q(i) * M_PI / 180.0;
            }
        }
        if (meta_skill_type == "pickhandoverplace" && skill_seq == 4) {
            // goto transfer_up pose
            std::vector<RobotState> handover_goals;
            calculateHandoverPoses(robot_->robot_id, handover_goals, receive_q);
            robot_goal_state = handover_goals[0];

            // find the env state objects with the same name as current object
            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->parent_link = robot_->end_effector_link;
                    obj->robot_id = robot_->robot_id;
                    obj->state = Object::State::Attached;
                    instance_->computeRelativeTransform(*obj, robot_goal_state);
                    break;
                }
            }
        }
        if (meta_skill_type == "pickhandoverplace" && skill_seq == 7) {
            // go to pre place up pose
            lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z+2, press_ori, press_side, cart_T);
            cart_T = cart_T * y_s90 * z_180;

            lego_manipulation::math::VectorJd r_place_up, r_twist, r_twist_down;
            // place up
            calculateIKforLego(cart_T, home_q, robot_->robot_id, 3, true, r_place_up, robot_goal_state, reachable);
            if (!reachable) {
                return false;
            }
            // find the env state objects with the same name as current object
            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->parent_link = robot_->end_effector_link;
                    break;
                }
            }
        }
        if (meta_skill_type == "pickhandoverplace" && skill_seq == 8) {
            // go to support_top_pre pose
            Eigen::Matrix4d press_T = Eigen::MatrixXd::Identity(4, 4);
            Eigen::Matrix4d press_up_T = press_T;
            
            // press up
            if(support_ori == 0){
                lego_ptr_->assemble_pose_from_top(support_x+1, support_y, support_z+1, 0, support_ori+1, press_T);
                press_T(0, 3) = press_T(0, 3) + 0.002;
            }
            else if(support_ori == 1){
                lego_ptr_->assemble_pose_from_top(support_x, support_y-1, support_z+1, 1, support_ori+1, press_T);
                press_T(1, 3) = press_T(1, 3) - 0.002;
            }
            else if (support_ori == 2) {
                lego_ptr_->assemble_pose_from_top(support_x, support_y+1, support_z+1, 1, support_ori+1, press_T);
                press_T(1, 3) = press_T(1, 3) + 0.002;
            }
            else if (support_ori == 3) {
                lego_ptr_->assemble_pose_from_top(support_x-1, support_y, support_z+1, 0, support_ori+1, press_T);
                press_T(0, 3) = press_T(0, 3) - 0.002;
            }
            else {
                log("Invalid support orientation " + std::to_string(support_ori), LogLevel::ERROR);
                return false;
            }
            press_T(2, 3) = press_T(2, 3) - lego_ptr_->brick_height() + lego_ptr_->lever_wall_height() + lego_ptr_->knob_height();
            press_up_T = press_T;
            press_up_T(2, 3) = press_up_T(2, 3) + 0.005;
    
    
            calculateIKforLego(press_up_T, home_q, robot_->robot_id, 0, true, goal_q, robot_goal_state, reachable);
            if (!reachable) {
                return false;
            }

            for (auto &obj : env_state.objects) {
                // update the state of the object
                if (obj->name == brick_name) {
                    obj->state = Object::State::Static;
                    obj->parent_link = "world";
                    Eigen::Matrix4d brick_pose_mtx;
                    lego_ptr_->calc_bric_asssemble_pose(brick_name, brick_x, brick_y, brick_z, ori, brick_pose_mtx);
                    obj->x = brick_pose_mtx(0, 3);
                    obj->y = brick_pose_mtx(1, 3);
                    obj->z = brick_pose_mtx(2, 3) - obj->height/2;
                    // update orientation
                    Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
                    obj->qx = quat.x();
                    obj->qy = quat.y();
                    obj->qz = quat.z();
                    obj->qw = quat.w();
                    break;
                }
            }
        }


        if ((meta_skill_type == "pickplace" && skill_seq == 4)
            || (meta_skill_type == "pickplacewithsupport" && skill_seq == 2)
            || (meta_skill_type == "pickplacewithsupport" && skill_seq == 7)
            || (meta_skill_type == "pickplacewithsupport" && skill_seq == 8)
            || (meta_skill_type == "pickhandoverplace" && skill_seq == 2)
            || (meta_skill_type == "pickhandoverplace" && skill_seq == 5)
            || (meta_skill_type == "pickhandoverplace" && skill_seq == 6)
            || (meta_skill_type == "pickhandoverplace" && skill_seq == 11)
            || (meta_skill_type == "pickhandoverplace" && skill_seq == 12)){
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
        if (!reachable) {
            return false;
        }
    }
    else if (type == Skill::Type::PlaceTop) {
        lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z, press_ori, press_side, cart_T);
        calculateIKforLego(cart_T, home_q, robot_->robot_id, 0, false, goal_q, robot_goal_state, reachable);
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
        if (!reachable) {
            return false;
        }
    }
    else if (type == Skill::Type::PlaceBottom) {
        lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z+2, press_ori, press_side, cart_T);
        cart_T = cart_T * y_s90 * z_180;

        lego_manipulation::math::VectorJd r_place_up, r_twist, r_twist_down;
        // place up
        calculateIKforLego(cart_T, home_q, robot_->robot_id, 3, true, r_place_up, robot_goal_state, reachable);
        if (!reachable) {
            return false;
        }

        // place twist
        Eigen::MatrixXd twist_T = Eigen::MatrixXd::Identity(4, 4);
        twist_T.block(0, 0, 3, 3) << twist_R_pick;

        if (robot_->robot_id == 0) {
            cart_T = lego_manipulation::math::FK(r_place_up, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), false);
        }
        else {
            cart_T = lego_manipulation::math::FK(r_place_up, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        }
        cart_T = cart_T * twist_T;
        calculateIKforLego(cart_T, r_place_up, robot_->robot_id, 4, true, r_twist, robot_goal_state, reachable);
        if (!reachable) {
            return false;
        }
        
    }
    else if (type == Skill::Type::SupportBottom) {
        Eigen::Matrix4d sup_T = Eigen::MatrixXd::Identity(4, 4);
        lego_ptr_->support_pose(support_x, support_y, support_z, support_ori, sup_T);

        calculateIKforLego(sup_T, home_q, robot_->robot_id, 0, true, goal_q, robot_goal_state, reachable);
        if (!reachable) {
            return false;
        }
    }
    else if (type == Skill::Type::SupportTop) {
        Eigen::Matrix4d press_T = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Matrix4d press_up_T = press_T;
        
        // press up
        if(support_ori == 0){
            lego_ptr_->assemble_pose_from_top(support_x+1, support_y, support_z+1, 0, support_ori+1, press_T);
            press_T(0, 3) = press_T(0, 3) + 0.002;
        }
        else if(support_ori == 1){
            lego_ptr_->assemble_pose_from_top(support_x, support_y-1, support_z+1, 1, support_ori+1, press_T);
            press_T(1, 3) = press_T(1, 3) - 0.002;
        }
        else if (support_ori == 2) {
            lego_ptr_->assemble_pose_from_top(support_x, support_y+1, support_z+1, 1, support_ori+1, press_T);
            press_T(1, 3) = press_T(1, 3) + 0.002;
        }
        else if (support_ori == 3) {
            lego_ptr_->assemble_pose_from_top(support_x-1, support_y, support_z+1, 0, support_ori+1, press_T);
            press_T(0, 3) = press_T(0, 3) - 0.002;
        }
        else {
            log("Invalid support orientation " + std::to_string(support_ori), LogLevel::ERROR);
            return false;
        }
        press_T(2, 3) = press_T(2, 3) - lego_ptr_->brick_height() + lego_ptr_->lever_wall_height() + lego_ptr_->knob_height();
        press_up_T = press_T;
        press_up_T(2, 3) = press_up_T(2, 3) + 0.005;


        calculateIKforLego(press_T, home_q, robot_->robot_id, 0, true, goal_q, robot_goal_state, reachable);
        if (!reachable) {
            return false;
        }
    }
    else if (type == Skill::Type::Handover) {
        std::vector<RobotState> handover_goals;
        calculateHandoverPoses(robot_->robot_id, handover_goals, receive_q);
        robot_goal_state = handover_goals[2];
    }
    else if (type == Skill::Type::PlaceWithSupport || type == Skill::Type::PickAndPlace 
        || type == Skill::Type::PickAndPlaceWithSupport || type == Skill::Type::PickHandoverAndPlace) {
            throw std::runtime_error("Unsupported meta skill " + std::to_string(int(type)));
    }
    else {
        throw std::runtime_error("Unknown skill type: " + std::to_string(int(type)));
    }


    // find the env state objects with the same name as current object
    for (auto &obj : env_state.objects) {
        // update the state of the object
        if (obj->name == brick_name && obj->state == Object::State::Attached
                && obj->robot_id == robot_goal_state.robot_id) {
            instance_->computeWorldTransform(*obj, robot_goal_state);
            break;
        }
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