#include "lego/lego_algorithms.hpp"
#include "tasks.hpp"
#include "Utils/Math.hpp"
#include "Utils/Logger.hpp"
#include "adg.h"


namespace skillgraph {

/**
 * @brief Construct a LegoGraspGenerator for Lego manipulation tasks.
 * @param lego_ptr Shared pointer to Lego object.
 * @param instance Shared pointer to PlanInstance.
 * @param config Lego policy configuration.
 * @param robot Robot pointer.
 * @param object Object pointer.
 */
LegoGraspGenerator::LegoGraspGenerator(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
    std::shared_ptr<skillgraph::PlanInstance> instance,
    const LegoPolicyCfg &config, RobotPtr robot, ObjPtr object) 
    : lego_ptr_(lego_ptr), instance_(instance), config_(config), robot_(robot), object_(object) {
}

/**
 * @brief Calculate handover poses for the support arm robot.
 *
 * This method computes the handover pose for the specified robot and updates the goal and receive_q.
 * @param robot_id The ID of the robot.
 * @param handover_goal Output vector of handover goal states.
 * @param receive_q Output matrix for receiving joint configuration.
 * @return True if calculation is successful.
 */
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
    log("generate function",LogLevel::INFO);
    log("Generating grasp pose for skill " + std::to_string(skill_seq) + " robot " 
        + std::to_string(robot_->robot_id), LogLevel::INFO);
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

            


        // instance_->updateScene();
        // bool hasCollision = instance_->checkCollision({robot_goal_state}, false);
        // if (hasCollision) {
        //     log("Collision detected in generate grasp pose for skill " + std::to_string(skill_seq) + " robot " 
        //         + std::to_string(robot_->robot_id), LogLevel::WARN);
        //     return false;
        // }
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
    
        // instance_->updateScene();
        
        // bool hasCollision = instance_->checkCollision({robot_state}, false);
        // reachable &= !hasCollision;
        // if (hasCollision) {
        //     log("Collision detected in calculateIKforLego", LogLevel::WARN);
        // }
    

}
















































LegoPlan::LegoPlan(std::shared_ptr<lego_manipulation::lego::Lego> lego_ptr,
                    std::shared_ptr<skillgraph::PlanInstance> instance,
                    const LegoPolicyCfg &config,
                    RobotPtr robot, ObjPtr object) 
    : lego_ptr_(lego_ptr), instance_(instance), config_(config), robot_(robot), object_(object) {
}

bool LegoPlan::plan_skill(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, Skill::Type type, skillgraph::RobotTrajectory &traj) {
    if (type == Skill::Type::Pick) {
        return plan_pick(current_state, task_param, traj);
    }
    else if (type == Skill::Type::PlaceTop) {
        return plan_placedown(current_state, task_param, traj);
    }
    else if (type == Skill::Type::PlaceBottom) {
        return plan_placeup(current_state, task_param, traj);
    }
    else if (type == Skill::Type::SupportBottom) {
        return plan_support(current_state, task_param, traj);
    }
    else if (type == Skill::Type::SupportTop) {
        return plan_pressdown(current_state, task_param, traj);
    }
    else if (type == Skill::Type::Handover) {
        return plan_handover(current_state, task_param, traj);
    }
    else if(type == Skill::Type::Transit || type == Skill::Type::align || type == Skill::Type::Translate || type == Skill::Type::Rotate) {
        return true;
    }
    else {
        throw std::runtime_error("Unsupported skill type: " + std::to_string(int(type)));
        // return true; // For now, we return true for unsupported skills
    }
}

// Add this private helper method to LegoPlan class in lego_algorithms.cpp
void LegoPlan::calculateIKforLegoPlan(const Eigen::MatrixXd& T_target_pose, const Eigen::MatrixXd & seed_q_deg, 
                                     int robot_id, int fk_type_for_ik, bool check_collision, 
                                     lego_manipulation::math::VectorJd &joint_q_deg_out, 
                                     RobotState &robot_state_rad_out, bool &IK_status_out) {
    IK_status_out = false; // Assume failure initially
    joint_q_deg_out = seed_q_deg; // Default to seed if IK fails early

    lego_manipulation::math::VectorJd temp_joint_q_deg;

    if (robot_id == 0) { // Assuming robot 0 is r1
        if (fk_type_for_ik == 0) { // Standard Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_inv_r1(), 0, IK_status_out);
        } else if (fk_type_for_ik == 1) { // Assemble Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_assemble_inv_r1(), 0, IK_status_out);
        } else if (fk_type_for_ik == 2) { // Disassemble Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_disassemble_inv_r1(), 0, IK_status_out);
        } else if (fk_type_for_ik == 3) { // Alt Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_inv_r1(), 0, IK_status_out);
        } else if (fk_type_for_ik == 4) { // Alt Assemble Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), 
                                    lego_ptr_->robot_base_inv_r1(), lego_ptr_->robot_tool_alt_assemble_inv_r1(), 0, IK_status_out);
        } else {
            log("Unknown fk_type_for_ik: " + std::to_string(fk_type_for_ik) + " for robot " + std::to_string(robot_id), LogLevel::ERROR);
            IK_status_out = false;
            return;
        }
    } else { // Assuming robot 1 is r2
        if (fk_type_for_ik == 0) { // Standard Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_inv_r2(), 0, IK_status_out);
        } else if (fk_type_for_ik == 1) { // Assemble Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_assemble_inv_r2(), 0, IK_status_out);
        } else if (fk_type_for_ik == 2) { // Disassemble Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_disassemble_inv_r2(), 0, IK_status_out);
        } else if (fk_type_for_ik == 3) { // Alt Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_inv_r2(), 0, IK_status_out);
        } else if (fk_type_for_ik == 4) { // Alt Assemble Tool
            temp_joint_q_deg = lego_ptr_->IK(seed_q_deg, T_target_pose, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), 
                                    lego_ptr_->robot_base_inv_r2(), lego_ptr_->robot_tool_alt_assemble_inv_r2(), 0, IK_status_out);
        } else {
            log("Unknown fk_type_for_ik: " + std::to_string(fk_type_for_ik) + " for robot " + std::to_string(robot_id), LogLevel::ERROR);
            IK_status_out = false;
            return;
        }
    }



    if (IK_status_out) {
        joint_q_deg_out = temp_joint_q_deg;
        robot_state_rad_out = instance_->initRobotState(robot_id); // Ensure it's properly sized
        for (int i = 0; i < joint_q_deg_out.rows(); ++i) {
            robot_state_rad_out.joint_values[i] = joint_q_deg_out(i, 0) * M_PI / 180.0;
        }

        if (check_collision) {
            //check start pose collision




            //check target pose collision
            bool has_collision = instance_->checkCollision({robot_state_rad_out}, false);
            if (has_collision) {
                IK_status_out = false;
                // log collision
            }
        }
    }
}

void LegoPlan::interpolate_segment(const RobotState& start_pose_rad, const RobotState& end_pose_rad, 
                                   skillgraph::RobotTrajectory &traj) {
    // Basic linear interpolation.
    // Timestamps are calculated based on dt and velocity.



    
        bool start_collision = instance_->checkCollision({start_pose_rad}, false);
        bool end_collision = instance_->checkCollision({end_pose_rad}, false);

        if (start_collision || end_collision) {
            if (start_collision)
                log("Start pose is in collision!", LogLevel::ERROR);
            if (end_collision)
                log("End pose is in collision!", LogLevel::ERROR);
            return; // Skip interpolation due to collision
        }
    

    double total_duration = 0.0;
    if (!traj.times.empty()) {
        total_duration = traj.times.back();
    }
    
    // Calculate path length and segment duration
    double max_joint_diff = 0.0;
    if (start_pose_rad.joint_values.size() != end_pose_rad.joint_values.size()) {
        // Log error: joint_values size mismatch
        return;
    }

    for (size_t i = 0; i < start_pose_rad.joint_values.size(); ++i) {
        max_joint_diff = std::max(max_joint_diff, std::abs(end_pose_rad.joint_values[i] - start_pose_rad.joint_values[i]));
    }

    double segment_duration = 0.0;
    if (config_.velocity > 1e-6 && max_joint_diff > 1e-6) { // Max L1 joint velocity
        segment_duration = max_joint_diff / config_.velocity;
    } else if (max_joint_diff <= 1e-6) { // Start and end are the same
        segment_duration = 0.0; 
    } else { // Default duration if velocity is zero or not meaningful
        segment_duration = 1.0; // Default 1 second
    }

    int num_steps = 0;
    if (config_.dt > 1e-6 && segment_duration > 1e-6) {
        num_steps = static_cast<int>(std::ceil(segment_duration / config_.dt));
    }
    num_steps = std::max(1, num_steps); // Ensure at least one step (the end point)

    // If it's the first segment and traj is empty, add the start_pose_rad
    // Assumes plan_pick adds the absolute start state initially.
    // This function appends from the last point in traj to end_pose_rad.

    for (int i = 1; i <= num_steps; ++i) {
        double ratio = static_cast<double>(i) / num_steps;
        RobotState wp_state;
        wp_state.robot_id = start_pose_rad.robot_id; // Or end_pose_rad.robot_id, should be consistent
        wp_state.robot_name = start_pose_rad.robot_name;
        wp_state.joint_values.resize(start_pose_rad.joint_values.size());
        for (size_t j = 0; j < start_pose_rad.joint_values.size(); ++j) {
            wp_state.joint_values[j] = start_pose_rad.joint_values[j] + ratio * (end_pose_rad.joint_values[j] - start_pose_rad.joint_values[j]);
        }
        // Assuming hand_values remain constant for this segment or are handled by the caller of plan_pick
        wp_state.hand_values = end_pose_rad.hand_values; // Or start_pose_rad.hand_values

        traj.trajectory.push_back(wp_state);
        traj.times.push_back(total_duration + ratio * segment_duration);
        // act_ids can be populated if needed, e.g., with a default or specific action ID for pick segments
    }
}


bool LegoPlan::plan_pick(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj)
{
    traj.trajectory.clear();
    traj.times.clear();
    traj.act_ids.clear(); // Also clear action IDs

    // 1. Initialization
    int robot_id = robot_->robot_id;
    if (robot_id < 0 || robot_id >= instance_->getNumberOfRobots()) {
        log("Invalid robot_id in plan_pick: " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }
    
    int robot_dof;
    if (robot_id == 0) {
        robot_dof = lego_ptr_->robot_dof_1();
    } else if (robot_id == 1) {
        robot_dof = lego_ptr_->robot_dof_2();
    } else {
        log("Invalid robot_id in plan_pick: " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }


    const Json::Value& constraints = task_param.constraints_json;
    if (!constraints.isMember("brick_id") || !constraints.isMember("press_side") || !constraints.isMember("press_offset")) {
        log("Missing constraints (brick_id, press_side, or press_offset) in plan_pick.", LogLevel::ERROR);
        return false;
    }
    // if (!constraints.isMember("brick_x") || !constraints.isMember("brick_y") || !constraints.isMember("brick_z") || !constraints.isMember("brick_ori")) {
    //     log("Missing constraints (brick_x, brick_y, brick_z, or brick_ori) in plan_pick.", LogLevel::ERROR);
    //     return false;
    // }
     if (!constraints.isMember("x") || !constraints.isMember("y") || !constraints.isMember("z") || !constraints.isMember("ori")) {
        log("Missing constraints (brick_x, brick_y, brick_z, or brick_ori) in plan_pick.", LogLevel::ERROR);
        return false;
    }
    int brick_id_val = constraints["brick_id"].asInt();
    int press_side = constraints["press_side"].asInt();
    int press_offset = constraints["press_offset"].asInt();
    // int brick_x = constraints["brick_x"].asInt();
    // int brick_y = constraints["brick_y"].asInt();
    // int brick_z = constraints["brick_z"].asInt();
    // int brick_ori = constraints["brick_ori"].asInt();



    int brick_x = constraints["x"].asInt();
    int brick_y = constraints["y"].asInt();
    int brick_z = constraints["z"].asInt();
    int brick_ori = constraints["ori"].asInt();
    
    std::string brick_name = object_->name;
    if (brick_name.empty()){
        log("Could not find brick name for ID: " + std::to_string(brick_id_val), LogLevel::ERROR);
        return false;
    }


    lego_manipulation::math::VectorJd home_q_deg = Eigen::MatrixXd(robot_dof, 1);
    // Ensure home_q_deg matches the DOF
    if (robot_dof == 6) { // Example for 6 DOF
        home_q_deg << 0, 0, 0, 0, -90, 0; 
    } else {
        log("Unsupported DOF for home_q_deg in plan_pick: " + std::to_string(robot_dof), LogLevel::ERROR);
        return false;
    }


    Eigen::Vector3d grab_offsets(-0.005, 0.005, -0.0028); 

    Eigen::Matrix3d twist_R_pick_mat = Eigen::Matrix3d::Identity();
    double twist_angle_rad = config_.twist_rad;
    twist_R_pick_mat << cos(twist_angle_rad), 0, sin(twist_angle_rad),
                        0, 1, 0,
                        -sin(twist_angle_rad), 0, cos(twist_angle_rad);

    bool overall_reachable = true;
    RobotState current_robot_state_rad = current_state.robot_states[robot_id];
    
    // Add initial state to trajectory
    traj.trajectory.push_back(current_robot_state_rad);
    traj.robot_id = robot_id;
    traj.times.push_back(0.0);


    lego_manipulation::math::VectorJd current_seed_q_deg(robot_dof, 1);
    if (current_robot_state_rad.joint_values.size() != static_cast<size_t>(current_seed_q_deg.rows())) {
        log(std::to_string(current_robot_state_rad.joint_values.size()) + " != " + std::to_string(current_seed_q_deg.rows()) + 
            " in plan_pick: Mismatch in joint values size and robot_dof.", LogLevel::ERROR);
        // log("Mismatch in joint values size and robot_dof.", LogLevel::ERROR);
        return false;
    }
    for(int i=0; i < current_seed_q_deg.rows(); ++i) {
        current_seed_q_deg(i) = current_robot_state_rad.joint_values[i] * 180.0 / M_PI;
    }
    
    // Use home_q_deg as the initial seed for the very first IK for robustness,
    // but the trajectory starts from current_robot_state_rad.
    // current_seed_q_deg = home_q_deg; // Keep seeding from current state after initial point.

    RobotState prev_interpolated_state_rad = current_robot_state_rad;
    lego_manipulation::math::VectorJd q_out_deg(robot_dof,1);
    RobotState state_out_rad = instance_->initRobotState(robot_id); // Ensures correct robot_id and name
    bool ik_status;

    Eigen::MatrixXd cart_T_grab_base = Eigen::Matrix4d::Identity();
    // Ensure calc_brick_grab_pose is compatible with your lego_ptr version
    
    lego_ptr_->calc_brick_grab_pose(brick_name, 1, 0, brick_x, brick_y, brick_z, brick_ori, press_side, press_offset,
                        cart_T_grab_base);

    // --- Pick Tilt Up ---
    // Eigen::Matrix4d pick_tilt_up_transform = Eigen::Matrix4d::Identity();
    // pick_tilt_up_transform(0,3) = grab_offsets(0); 
    // pick_tilt_up_transform(1,3) = grab_offsets(1); 
    // pick_tilt_up_transform(2,3) = grab_offsets(2) - std::abs(grab_offsets(2)); 
    // Eigen::Matrix4d target_T_pick_tilt_up = cart_T_grab_base * pick_tilt_up_transform;

    // calculateIKforLegoPlan(target_T_pick_tilt_up, home_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status); // Seed with home for first critical pose
    // if (!ik_status) { overall_reachable = false; log("IK failed for Pick Tilt Up", LogLevel::WARN); return overall_reachable; }
    // interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    // prev_interpolated_state_rad = state_out_rad;
    // current_seed_q_deg = q_out_deg;
    std::vector<RobotState> check_states= {state_out_rad};;
    if (instance_->checkCollision(check_states, false, false)) {
        log("Pick Up state is in collision", LogLevel::WARN);
        return false;
    }
    // --- Pick Up ---
    double pick_up_lift_z = 0.01; 
    Eigen::Matrix4d target_T_pick_up = cart_T_grab_base;
    target_T_pick_up(2,3) += pick_up_lift_z; 

    calculateIKforLegoPlan(target_T_pick_up, current_seed_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status);
    if (!ik_status) { overall_reachable = false; log("IK failed for Pick Up", LogLevel::WARN); return overall_reachable; }
    
    

    
    
    
    
    
    
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


    


        // add collision check here
    std::vector<RobotState> check_vec = {state_out_rad};
    if (instance_->checkCollision(check_vec, false, false)) {
        log("Pick Up state is in collision", LogLevel::WARN);
        return false;
    }




    // --- Pick (at grab pose) ---
    Eigen::Matrix4d target_T_pick = cart_T_grab_base;
    calculateIKforLegoPlan(target_T_pick, current_seed_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status);
    if (!ik_status) { overall_reachable = false; log("IK failed for Pick (at grab pose)", LogLevel::WARN); return overall_reachable; }
   
   
   
   
   
   
   
   
   
   
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;
// add collision check here
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected after Pick (grab pose)", LogLevel::WARN);
        return false;
    }









    // --- Pick Twist ---
    Eigen::Matrix4d fk_cart_T_for_twist;
    if (robot_id == 0) {
        fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), false);
    } else {
        fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), false);
    }
    Eigen::Matrix4d twist_transform_mat = Eigen::Matrix4d::Identity();
    twist_transform_mat.block<3,3>(0,0) = twist_R_pick_mat;
    Eigen::Matrix4d target_T_pick_twist = fk_cart_T_for_twist * twist_transform_mat;

    calculateIKforLegoPlan(target_T_pick_twist, current_seed_q_deg, robot_id, 2, true, q_out_deg, state_out_rad, ik_status); // fk_type 2 for disassemble tool
    if (!ik_status) { overall_reachable = false; log("IK failed for Pick Twist", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected after Pick Twist", LogLevel::WARN);
        return false;
    }



    // --- Pick Twist Up ---
    Eigen::Matrix4d fk_cart_T_twisted;
     if (robot_id == 0) {
        fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    } else {
        fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
    }
    Eigen::Matrix4d target_T_pick_twist_up = fk_cart_T_twisted;
    target_T_pick_twist_up(2,3) += 0.015; // Z up by 1.5cm

    calculateIKforLegoPlan(target_T_pick_twist_up, current_seed_q_deg, robot_id, 1, true, q_out_deg, state_out_rad, ik_status); // fk_type 1 for assemble tool
    if (!ik_status) { overall_reachable = false; log("IK failed for Pick Twist Up", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    // prev_interpolated_state_rad = state_out_rad; // Not strictly needed for the last segment

//add collision check here
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected after Pick Twist Up", LogLevel::WARN);
        return false;
    }




    return overall_reachable;
}


bool LegoPlan::plan_placedown(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj) {
    traj.trajectory.clear();
    traj.times.clear();
    traj.act_ids.clear();

    int robot_id = robot_->robot_id;
    traj.robot_id = robot_id;
    int robot_dof = (robot_id == 0) ? lego_ptr_->robot_dof_1() : lego_ptr_->robot_dof_2();

    const Json::Value& constraints = task_param.constraints_json;
    // Mandatory fields for placedown
    if (!constraints.isMember("brick_x") || !constraints.isMember("brick_y") || !constraints.isMember("brick_z") ||
        !constraints.isMember("brick_ori") || !constraints.isMember("press_side") || !constraints.isMember("press_offset")) {
        log("Missing constraints for plan_placedown (brick_x,y,z,ori, press_side, press_offset).", LogLevel::ERROR);
        return false;
    }
    int brick_x = constraints["brick_x"].asInt();
    int brick_y = constraints["brick_y"].asInt();
    int brick_z = constraints["brick_z"].asInt();
    int brick_ori = constraints["brick_ori"].asInt();
    int press_side = constraints["press_side"].asInt();
    int press_offset_val = constraints["press_offset"].asInt();
    int attack_dir = constraints.isMember("attack_dir") ? constraints["attack_dir"].asInt() : 1;
    std::string brick_name = object_->name;

    lego_manipulation::math::VectorJd home_q_deg = Eigen::MatrixXd(robot_dof, 1);
    if (robot_dof == 6) home_q_deg << 0,0,0,0,-90,0;
    else { log("Unsupported DOF for home_q_deg in plan_placedown", LogLevel::ERROR); return false; }

    RobotState current_robot_state_rad = current_state.robot_states[robot_id];
    traj.trajectory.push_back(current_robot_state_rad);
    traj.times.push_back(0.0);

    lego_manipulation::math::VectorJd current_seed_q_deg(robot_dof, 1);
    for(int i=0; i < robot_dof; ++i) current_seed_q_deg(i) = current_robot_state_rad.joint_values[i] * 180.0 / M_PI;
    
    RobotState prev_interpolated_state_rad = current_robot_state_rad;
    lego_manipulation::math::VectorJd q_out_deg(robot_dof,1);
    RobotState state_out_rad = instance_->initRobotState(robot_id);
    bool ik_status;
    bool overall_reachable = true;

    // Offsets from TaskAssignment::calculateDropPoses (based on pick_offset member)
    // pick_offset << -0.005, 0.005, -0.005,  // place brick offset (idx 0,1,2)
    //                -0.005, 0.005, -0.0028, // grab brick offset (idx 3,4,5)
    double place_x_offset = -0.005;
    double place_y_offset = 0.005;
    double place_z_offset = -0.005;
    double grab_z_offset = -0.0028; // Used for abs(grab_z_offset)

    Eigen::MatrixXd cart_T_base_drop = Eigen::Matrix4d::Identity();
    lego_ptr_->calc_brick_grab_pose(brick_name, 1, 0, brick_x, brick_y, brick_z, brick_ori, press_side, press_offset_val, cart_T_base_drop);

    // Sequence: r_offset_goal, r_drop_up_goal, r_drop_goal, r_drop_twist_goal, r_drop_twist_up_goal
    std::vector<RobotState> check_states;
        check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Offset Goal", LogLevel::WARN);
        return false;
    }

    // 1. Offset Goal (Approach to Drop Up)
    Eigen::Matrix4d target_T_offset = cart_T_base_drop;
    Eigen::Matrix4d offset_transform = Eigen::Matrix4d::Identity();
    offset_transform(0,3) = place_x_offset;
    offset_transform(1,3) = place_y_offset * attack_dir;
    offset_transform(2,3) = place_z_offset - std::abs(grab_z_offset); // Approach from above and side
    target_T_offset = cart_T_base_drop * offset_transform;
    
    calculateIKforLegoPlan(target_T_offset, home_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status); // Seed with home
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceDown Offset Goal", LogLevel::WARN); return overall_reachable; }
    
    
    
    

    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;




    // add collision check here
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Offset Goal", LogLevel::WARN);
        return false;
    }


    // 2. Drop Up Goal
    Eigen::Matrix4d target_T_drop_up = cart_T_base_drop;
    Eigen::Matrix4d up_transform = Eigen::Matrix4d::Identity();
    // This Z should be an actual "up" from the contact point.
    // TaskAssignment used pick_offset(2) which is place_z_offset (-0.005). This is still down.
    // Let's use a positive lift from cart_T_base_drop.
    double drop_up_lift_z = 0.02; // Lift 2cm above contact
    up_transform(2,3) = drop_up_lift_z; 
    target_T_drop_up = cart_T_base_drop * up_transform;

    calculateIKforLegoPlan(target_T_drop_up, current_seed_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status);
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceDown Drop Up Goal", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


    // add collision check here
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Drop Up Goal", LogLevel::WARN);
        return false;
    }


    // 3. Drop Goal (Contact Pose)
    Eigen::Matrix4d target_T_drop = cart_T_base_drop;
    calculateIKforLegoPlan(target_T_drop, current_seed_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status);
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceDown Drop Goal", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;
// add collision check here
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Drop Goal", LogLevel::WARN);
        return false;
    }




    // 4. Drop Twist Goal
    Eigen::Matrix3d twist_R_place_mat = Eigen::Matrix3d::Identity();
    double place_twist_angle_rad = -config_.twist_rad; // Opposite to pick twist
    twist_R_place_mat << cos(place_twist_angle_rad), 0, sin(place_twist_angle_rad),
                         0, 1, 0,
                         -sin(place_twist_angle_rad), 0, cos(place_twist_angle_rad);
    
    Eigen::Matrix4d fk_cart_T_for_twist;
    // FK with assemble tool (fk_type 1) as per TaskAssignment
    if (robot_id == 0) fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    else fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
    
    Eigen::Matrix4d twist_transform_mat = Eigen::Matrix4d::Identity();
    twist_transform_mat.block<3,3>(0,0) = twist_R_place_mat;
    Eigen::Matrix4d target_T_drop_twist = fk_cart_T_for_twist * twist_transform_mat;

    calculateIKforLegoPlan(target_T_drop_twist, current_seed_q_deg, robot_id, 1, true, q_out_deg, state_out_rad, ik_status); // fk_type 1 for assemble tool
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceDown Drop Twist Goal", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Drop Twist Goal", LogLevel::WARN);
        return false;
    }


    // 5. Drop Twist Up Goal
    Eigen::Matrix4d fk_cart_T_twisted;
    // FK with assemble tool (fk_type 1)
    if (robot_id == 0) fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    else fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
    
    Eigen::Matrix4d target_T_drop_twist_up = fk_cart_T_twisted;
    target_T_drop_twist_up(2,3) += 0.015; // Z up by 1.5cm

    calculateIKforLegoPlan(target_T_drop_twist_up, current_seed_q_deg, robot_id, 1, true, q_out_deg, state_out_rad, ik_status); // fk_type 1 for assemble tool
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceDown Drop Twist Up Goal", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);


    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Drop Twist Up Goal", LogLevel::WARN);
        return false;
    }

    return overall_reachable;
}

bool LegoPlan::plan_placeup(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj) {
    traj.trajectory.clear();
    traj.times.clear();
    traj.act_ids.clear();

    int robot_id = robot_->robot_id;
    traj.robot_id = robot_id;
    int robot_dof = (robot_id == 0) ? lego_ptr_->robot_dof_1() : lego_ptr_->robot_dof_2();

    const Json::Value& constraints = task_param.constraints_json;
    // Mandatory fields for placeup
    if (!constraints.isMember("press_x") || !constraints.isMember("press_y") || !constraints.isMember("press_z") ||
        !constraints.isMember("press_ori") || !constraints.isMember("press_side") ) {
        log("Missing constraints for plan_placeup (press_x,y,z,ori, press_side).", LogLevel::ERROR);
        return false;
    }
    int press_x = constraints["press_x"].asInt();
    int press_y = constraints["press_y"].asInt();
    int press_z = constraints["press_z"].asInt(); // This is the target Z of the brick
    int press_ori = constraints["press_ori"].asInt();
    int press_side = constraints["press_side"].asInt(); // Used by assemble_pose_from_top
    int attack_dir = constraints.isMember("attack_dir") ? constraints["attack_dir"].asInt() : 1;
    // brick_name from object_->name if needed, but assemble_pose_from_top doesn't take it.

    lego_manipulation::math::VectorJd home_q_deg = Eigen::MatrixXd(robot_dof, 1); // Seed for first IK
    // Use a specific home for placeup, e.g., home_receive_q from TaskAssignment
    if (robot_dof == 6) home_q_deg << 0,0,0,0,0,180; // Example: receive-like home
    else if (robot_dof == 7) home_q_deg << 0,0,0,0,0,0,0; // Adjust
    else { log("Unsupported DOF for home_q_deg in plan_placeup", LogLevel::ERROR); return false; }


    RobotState current_robot_state_rad = current_state.robot_states[robot_id];
    traj.trajectory.push_back(current_robot_state_rad);
    traj.times.push_back(0.0);

    lego_manipulation::math::VectorJd current_seed_q_deg(robot_dof, 1);
    for(int i=0; i < robot_dof; ++i) current_seed_q_deg(i) = current_robot_state_rad.joint_values[i] * 180.0 / M_PI;
    
    RobotState prev_interpolated_state_rad = current_robot_state_rad;
    lego_manipulation::math::VectorJd q_out_deg(robot_dof,1);
    RobotState state_out_rad = instance_->initRobotState(robot_id);
    bool ik_status;
    bool overall_reachable = true;

    Eigen::Matrix4d y_s90 = Eigen::Matrix4d::Identity();
    y_s90 << 0,0,1,0, 0,1,0,0, -1,0,0,0, 0,0,0,1;
    Eigen::Matrix4d z_180 = Eigen::Matrix4d::Identity();
    z_180 << -1,0,0,0, 0,-1,0,0, 0,0,1,0, 0,0,0,1;

    // Offsets from TaskAssignment::calculatePlacePoses (based on pick_offset member)
    // pick_offset(6) = -0.0028 (place up offset)
    // pick_offset(4) =  0.005 (grab y offset)
    // pick_offset(3) = -0.005 (grab x offset)
    double place_up_val = -0.0028;
    double grab_y_off = 0.005;
    double grab_x_off = -0.005;

    Eigen::Matrix4d cart_T_base_placeup = Eigen::Matrix4d::Identity();
    // press_z+2 is used in TaskAssignment, meaning the brick is placed 2 units higher than its final Z.
    lego_ptr_->assemble_pose_from_top(press_x, press_y, press_z + 2, press_ori, press_side, cart_T_base_placeup);
    cart_T_base_placeup = cart_T_base_placeup * y_s90 * z_180; // Transform for bottom placement

    // Sequence: r_place_tilt_down_pre, r_place_tilt_down, r_place_down, r_place_up, r_twist, r_twist_down
    std::vector<RobotState> check_states;
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Tilt Down Pre", LogLevel::WARN);
        return false;
    }
    // 1. Place Tilt Down Pre
    Eigen::Matrix4d target_T_tilt_down_pre = cart_T_base_placeup;
    Eigen::Matrix4d pre_transform = Eigen::Matrix4d::Identity();
    // Original: -(place_up_val - abs(place_up_val)) -> 0 since place_up_val is negative
    pre_transform(0,3) = 0.0; 
    pre_transform(1,3) = attack_dir * (-grab_y_off);
    pre_transform(2,3) = grab_x_off - 0.02; // Further away in Z
    target_T_tilt_down_pre = cart_T_base_placeup * pre_transform;

    calculateIKforLegoPlan(target_T_tilt_down_pre, home_q_deg, robot_id, 3, true, q_out_deg, state_out_rad, ik_status); // fk_type 3 (alt tool), seed with home
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceUp Tilt Down Pre", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;
    //add collision check here
    
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Tilt Down Pre", LogLevel::WARN);
        return false;
    }




    // 2. Place Tilt Down
    Eigen::Matrix4d target_T_tilt_down = cart_T_base_placeup;
    Eigen::Matrix4d tilt_transform = Eigen::Matrix4d::Identity();
    tilt_transform(0,3) = 0.0;
    tilt_transform(1,3) = attack_dir * (-grab_y_off);
    tilt_transform(2,3) = grab_x_off;
    target_T_tilt_down = cart_T_base_placeup * tilt_transform;
    
    calculateIKforLegoPlan(target_T_tilt_down, current_seed_q_deg, robot_id, 3, true, q_out_deg, state_out_rad, ik_status); // fk_type 3
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceUp Tilt Down", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;

    //add collision check here
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Tilt Down", LogLevel::WARN);
        return false;
    }





    // 3. Place Down (Contact)
    Eigen::Matrix4d target_T_place_down = cart_T_base_placeup;
    Eigen::Matrix4d contact_transform = Eigen::Matrix4d::Identity();
    contact_transform(0,3) = -place_up_val; // This makes X positive (0.0028)
    target_T_place_down = cart_T_base_placeup * contact_transform;

    calculateIKforLegoPlan(target_T_place_down, current_seed_q_deg, robot_id, 3, true, q_out_deg, state_out_rad, ik_status); // fk_type 3
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceUp Place Down", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Place Down", LogLevel::WARN);
        return false;
    }




    
    // 4. Place Up (Retract after contact to base orientation for twist)
    Eigen::Matrix4d target_T_place_up = cart_T_base_placeup; // This is the pose before specific contact offsets
    calculateIKforLegoPlan(target_T_place_up, current_seed_q_deg, robot_id, 3, true, q_out_deg, state_out_rad, ik_status); // fk_type 3
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceUp Place Up", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Place Up", LogLevel::WARN);
        return false;
    }



    // 5. Place Twist
    Eigen::Matrix3d twist_R_pick_mat = Eigen::Matrix3d::Identity(); // Same as pick twist
    twist_R_pick_mat << cos(config_.twist_rad),0,sin(config_.twist_rad), 0,1,0, -sin(config_.twist_rad),0,cos(config_.twist_rad);

    Eigen::Matrix4d fk_cart_T_for_twist;
    // FK with alt_assemble_tool (fk_type 4)
    if (robot_id == 0) fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    else fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), false);
    
    Eigen::Matrix4d twist_transform_mat = Eigen::Matrix4d::Identity();
    twist_transform_mat.block<3,3>(0,0) = twist_R_pick_mat;
    Eigen::Matrix4d target_T_twist = fk_cart_T_for_twist * twist_transform_mat;

    calculateIKforLegoPlan(target_T_twist, current_seed_q_deg, robot_id, 4, true, q_out_deg, state_out_rad, ik_status); // fk_type 4
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceUp Twist", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Twist", LogLevel::WARN);
        return false;
    }


    // 6. Place Twist Down
    Eigen::Matrix4d fk_cart_T_twisted;
    // FK with alt_assemble_tool (fk_type 4)
    if (robot_id == 0) fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_alt_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    else fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_alt_assemble_r2(), lego_ptr_->robot_base_r2(), false);

    Eigen::Matrix4d twist_down_transform = Eigen::Matrix4d::Identity();
    twist_down_transform(0,3) = 0.015; // X forward
    twist_down_transform(2,3) = -0.015; // Z down
    Eigen::Matrix4d target_T_twist_down = fk_cart_T_twisted * twist_down_transform;
    
    calculateIKforLegoPlan(target_T_twist_down, current_seed_q_deg, robot_id, 4, true, q_out_deg, state_out_rad, ik_status); // fk_type 4
    if (!ik_status) { overall_reachable = false; log("IK failed for PlaceUp Twist Down", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PlaceUp Twist Down", LogLevel::WARN);
        return false;
    }




    return overall_reachable;
}





bool LegoPlan::plan_support(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj) {
    traj.trajectory.clear();
    traj.times.clear();
    traj.act_ids.clear();

    int robot_id = robot_->robot_id;
    traj.robot_id = robot_id;
    int robot_dof = (robot_id == 0) ? lego_ptr_->robot_dof_1() : lego_ptr_->robot_dof_2();

    const Json::Value& constraints = task_param.constraints_json;
    if (!constraints.isMember("support_x") || !constraints.isMember("support_y") || 
        !constraints.isMember("support_z") || !constraints.isMember("support_ori")) {
        log("Missing constraints for plan_support (support_x,y,z,ori).", LogLevel::ERROR);
        return false;
    }
    int support_x = constraints["support_x"].asInt();
    int support_y = constraints["support_y"].asInt();
    int support_z = constraints["support_z"].asInt();
    int support_ori = constraints["support_ori"].asInt();

    lego_manipulation::math::VectorJd home_q_deg = Eigen::MatrixXd(robot_dof, 1);
    if (robot_dof == 6) home_q_deg << 0,0,0,0,-90,0;
    else { log("Unsupported DOF for home_q_deg in plan_support", LogLevel::ERROR); return false; }

    RobotState current_robot_state_rad = current_state.robot_states[robot_id];
    traj.trajectory.push_back(current_robot_state_rad);
    traj.times.push_back(0.0);

    lego_manipulation::math::VectorJd current_seed_q_deg(robot_dof, 1);
    for(int i=0; i < robot_dof; ++i) current_seed_q_deg(i) = current_robot_state_rad.joint_values[i] * 180.0 / M_PI;
    
    RobotState prev_interpolated_state_rad = current_robot_state_rad;
    lego_manipulation::math::VectorJd q_out_deg(robot_dof,1);
    RobotState state_out_rad = instance_->initRobotState(robot_id);
    bool ik_status;
    bool overall_reachable = true;

    Eigen::Matrix4d sup_T = Eigen::Matrix4d::Identity();
    lego_ptr_->support_pose(support_x, support_y, support_z, support_ori, sup_T);


    std::vector<RobotState> check_states={state_out_rad};
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Support Pre-Pose", LogLevel::WARN);
        return false;
    }
    // 1. Pre-Support Pose
    Eigen::Matrix4d pre_sup_T = sup_T;
    // config_.pre_support_z_offset is -0.001. TaskAssignment uses sup_down_offset = -0.001 for pre-support.
    // This means pre-support is slightly lower. If it's an approach from above, it should be positive.
    // Let's assume pre_support_z_offset is a clearance, so it should be positive if approaching from above.
    // Or, if it's an offset along the tool's Z, then its sign depends on tool orientation.
    // LegoGraspGenerator uses: sup_T(2, 3) = sup_T(2, 3) + config_.pre_support_z_offset; for pre-pose.
    // If config_.pre_support_z_offset is positive, it means higher.
    // Let's make it an approach from a bit higher.
    double pre_support_clearance = 0.02; // 2cm above
    pre_sup_T(2,3) += pre_support_clearance; 

    calculateIKforLegoPlan(pre_sup_T, home_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status); // Seed with home
    if (!ik_status) { overall_reachable = false; log("IK failed for Support Pre-Pose", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Support Pre-Pose", LogLevel::WARN);
        return false;
    }

    // 2. Support Pose (Contact)
    calculateIKforLegoPlan(sup_T, current_seed_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status);
    if (!ik_status) { overall_reachable = false; log("IK failed for Support Pose", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Support Pose", LogLevel::WARN);
        return false;
    }

    return overall_reachable;
}











bool LegoPlan::plan_pressdown(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj) {
    traj.trajectory.clear();
    traj.times.clear();
    traj.act_ids.clear();

    int robot_id = robot_->robot_id;
    traj.robot_id = robot_id;
    int robot_dof = (robot_id == 0) ? lego_ptr_->robot_dof_1() : lego_ptr_->robot_dof_2();

    const Json::Value& constraints = task_param.constraints_json;
    // In LegoGraspGenerator, these are support_x,y,z,ori for the brick being pressed ON.
    if (!constraints.isMember("support_x") || !constraints.isMember("support_y") || 
        !constraints.isMember("support_z") || !constraints.isMember("support_ori")) {
        log("Missing constraints for plan_pressdown (support_x,y,z,ori of the target brick).", LogLevel::ERROR);
        return false;
    }
    int target_brick_x = constraints["support_x"].asInt();
    int target_brick_y = constraints["support_y"].asInt();
    int target_brick_z = constraints["support_z"].asInt();
    int target_brick_ori_type = constraints["support_ori"].asInt(); // This is the side of the target brick to press on.

    lego_manipulation::math::VectorJd home_q_deg = Eigen::MatrixXd(robot_dof, 1);
    if (robot_dof == 6) home_q_deg << 0,0,0,0,-90,0;
    else { log("Unsupported DOF for home_q_deg in plan_pressdown", LogLevel::ERROR); return false; }

    RobotState current_robot_state_rad = current_state.robot_states[robot_id];
    traj.trajectory.push_back(current_robot_state_rad);
    traj.times.push_back(0.0);

    lego_manipulation::math::VectorJd current_seed_q_deg(robot_dof, 1);
    for(int i=0; i < robot_dof; ++i) current_seed_q_deg(i) = current_robot_state_rad.joint_values[i] * 180.0 / M_PI;
    
    RobotState prev_interpolated_state_rad = current_robot_state_rad;
    lego_manipulation::math::VectorJd q_out_deg(robot_dof,1);
    RobotState state_out_rad = instance_->initRobotState(robot_id);
    bool ik_status;
    bool overall_reachable = true;

    Eigen::Matrix4d press_T = Eigen::Matrix4d::Identity();
    // Logic from LegoGraspGenerator::generate Skill::Type::SupportTop
    // The 'support_ori' in LegoGraspGenerator seems to indicate which face of the target brick the press comes from.
    // And assemble_pose_from_top is called with target_brick_ori+1 for its 'press_side' argument.
    // This is complex and specific to how lego_ptr_->assemble_pose_from_top interprets these.
    // Let's replicate the logic carefully.
    int press_target_x = target_brick_x;
    int press_target_y = target_brick_y;
    int press_target_z = target_brick_z + 1; // Pressing on top of the target brick
    int press_target_ori_for_assemble = 0; // Default orientation for assemble_pose_from_top
    int press_side_for_assemble = target_brick_ori_type + 1;

    if(target_brick_ori_type == 0){ // Pressing from +X side of target brick
        press_target_x = target_brick_x + 1; 
        press_target_ori_for_assemble = 0; // Brick being pressed is horizontal
        lego_ptr_->assemble_pose_from_top(press_target_x, press_target_y, press_target_z, press_target_ori_for_assemble, press_side_for_assemble, press_T);
        press_T(0, 3) += 0.002; // Small offset
    } else if(target_brick_ori_type == 1){ // Pressing from -Y side
        press_target_y = target_brick_y - 1;
        press_target_ori_for_assemble = 1; // Brick being pressed is vertical
        lego_ptr_->assemble_pose_from_top(press_target_x, press_target_y, press_target_z, press_target_ori_for_assemble, press_side_for_assemble, press_T);
        press_T(1, 3) -= 0.002;
    } else if (target_brick_ori_type == 2) { // Pressing from +Y side
        press_target_y = target_brick_y + 1;
        press_target_ori_for_assemble = 1;
        lego_ptr_->assemble_pose_from_top(press_target_x, press_target_y, press_target_z, press_target_ori_for_assemble, press_side_for_assemble, press_T);
        press_T(1, 3) += 0.002;
    } else if (target_brick_ori_type == 3) { // Pressing from -X side
        press_target_x = target_brick_x - 1;
        press_target_ori_for_assemble = 0;
        lego_ptr_->assemble_pose_from_top(press_target_x, press_target_y, press_target_z, press_target_ori_for_assemble, press_side_for_assemble, press_T);
        press_T(0, 3) -= 0.002;
    } else {
        log("Invalid target_brick_ori_type for plan_pressdown: " + std::to_string(target_brick_ori_type), LogLevel::ERROR);
        return false;
    }
    // Adjust Z for pressing tool
    press_T(2, 3) = press_T(2, 3) - lego_ptr_->brick_height() + lego_ptr_->lever_wall_height() + lego_ptr_->knob_height();



     std::vector<RobotState> check_states;
     check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PressDown Up-Pose", LogLevel::WARN);
        return false;
    }
    // 1. Press Up Pose
    Eigen::Matrix4d press_up_T = press_T;
    press_up_T(2, 3) += 0.005; // Lifted by 0.5cm

    calculateIKforLegoPlan(press_up_T, home_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status); // Seed with home
    if (!ik_status) { overall_reachable = false; log("IK failed for PressDown Up-Pose", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


   
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PressDown Up-Pose", LogLevel::WARN);
        return false;
    }




    // 2. Press Down Pose (Contact)
    calculateIKforLegoPlan(press_T, current_seed_q_deg, robot_id, 0, true, q_out_deg, state_out_rad, ik_status);
    if (!ik_status) { overall_reachable = false; log("IK failed for PressDown Contact-Pose", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    
    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at PressDown Contact-Pose", LogLevel::WARN);
        return false;
    }





    return overall_reachable;
}


























bool LegoPlan::plan_handover(const skillgraph::State &current_state, const skillgraph::TaskParam &task_param, skillgraph::RobotTrajectory &traj) {
    traj.trajectory.clear();
    traj.times.clear();
    traj.act_ids.clear();

    int robot_id_handovering = robot_->robot_id;
    traj.robot_id = robot_id_handovering;
    int robot_dof = (robot_id_handovering == 0) ? lego_ptr_->robot_dof_1() : lego_ptr_->robot_dof_2();
    int robot_id_receiving = (robot_id_handovering == 0) ? 1 : 0;

    const Json::Value& constraints = task_param.constraints_json;
    if (!constraints.isMember("receive_q_joint_values")) {
        log("Missing constraint 'receive_q_joint_values' for plan_handover.", LogLevel::ERROR);
        return false;
    }
    Json::Value receive_q_json = constraints["receive_q"];
    if (!receive_q_json.isArray() || receive_q_json.size() != robot_dof) {
        log("Invalid 'receive_q' format or size.", LogLevel::ERROR);
        return false;
    }
    Eigen::MatrixXd receive_q_other_robot_deg = Eigen::MatrixXd(robot_dof, 1);
    for (size_t i = 0; i < receive_q_json.size(); ++i) {
        receive_q_other_robot_deg(i,0) = receive_q_json[static_cast<Json::ArrayIndex>(i)].asDouble(); // Assuming these are in degrees
    }

    // Home pose for the handovering arm (e.g. home_handover_q from TaskAssignment)
    lego_manipulation::math::VectorJd seed_q_deg_handover_arm = Eigen::MatrixXd(robot_dof, 1);
    if (robot_dof == 6) seed_q_deg_handover_arm << 0,0,0,0,-90,0;
    else { log("Unsupported DOF for home_q_deg in plan_handover", LogLevel::ERROR); return false; }


    RobotState current_robot_state_rad = current_state.robot_states[robot_id_handovering];
    traj.trajectory.push_back(current_robot_state_rad);
    traj.times.push_back(0.0);

    lego_manipulation::math::VectorJd current_seed_q_deg(robot_dof, 1);
    for(int i=0; i < robot_dof; ++i) current_seed_q_deg(i) = current_robot_state_rad.joint_values[i] * 180.0 / M_PI;
    
    RobotState prev_interpolated_state_rad = current_robot_state_rad;
    lego_manipulation::math::VectorJd q_out_deg(robot_dof,1);
    RobotState state_out_rad = instance_->initRobotState(robot_id_handovering);
    bool ik_status;
    bool overall_reachable = true;

    Eigen::Matrix4d y_s90 = Eigen::Matrix4d::Identity();
    y_s90 << 0,0,1,0, 0,1,0,0, -1,0,0,0, 0,0,0,1;
    Eigen::Matrix4d z_180 = Eigen::Matrix4d::Identity();
    z_180 << -1,0,0,0, 0,-1,0,0, 0,0,1,0, 0,0,0,1;

    Eigen::Matrix4d cart_T_handover_base = Eigen::Matrix4d::Identity();
    if (robot_id_receiving == 0) { // Receiving robot is R1, handovering is R2, FK uses R1's DH_alt
        cart_T_handover_base = lego_manipulation::math::FK(receive_q_other_robot_deg, lego_ptr_->robot_DH_tool_alt_r1(), lego_ptr_->robot_base_r1(), false);
    } else { // Receiving robot is R2, handovering is R1, FK uses R2's DH_alt
        cart_T_handover_base = lego_manipulation::math::FK(receive_q_other_robot_deg, lego_ptr_->robot_DH_tool_alt_r2(), lego_ptr_->robot_base_r2(), false);
    }
    cart_T_handover_base = cart_T_handover_base * y_s90 * z_180; // Transform to handover orientation

    // Sequence: transfer_up, transfer_down, transfer_twist, transfer_twist_up


        std::vector<RobotState> check_states;
        check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Handover Transfer Up", LogLevel::WARN);
        return false;
    }
    // 1. Transfer Up
    Eigen::Matrix4d target_T_transfer_up = cart_T_handover_base;
    target_T_transfer_up(2,3) += 0.015; // Lift 1.5cm

    // FK type 0 for IK relative to the transformed cart_T_handover_base (which used alt tool for FK)
    calculateIKforLegoPlan(target_T_transfer_up, seed_q_deg_handover_arm, robot_id_handovering, 0, true, q_out_deg, state_out_rad, ik_status); // Seed with home-like pose
    if (!ik_status) { overall_reachable = false; log("IK failed for Handover Transfer Up", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;


    

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Handover Transfer Up", LogLevel::WARN);
        return false;
    }

    // 2. Transfer Down (Contact)
    Eigen::Matrix4d target_T_transfer_down = cart_T_handover_base;
    calculateIKforLegoPlan(target_T_transfer_down, current_seed_q_deg, robot_id_handovering, 0, true, q_out_deg, state_out_rad, ik_status); // fk_type 0
    if (!ik_status) { overall_reachable = false; log("IK failed for Handover Transfer Down", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;



    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Handover Transfer Down", LogLevel::WARN);
        return false;
    }

    // 3. Transfer Twist
    Eigen::Matrix3d twist_R_handover_mat = Eigen::Matrix3d::Identity();
    double handover_twist_angle = -config_.twist_rad_handover; // As in TaskAssignment
    twist_R_handover_mat << cos(handover_twist_angle),0,sin(handover_twist_angle), 0,1,0, -sin(handover_twist_angle),0,cos(handover_twist_angle);

    Eigen::Matrix4d fk_cart_T_for_twist;
    // FK with handover_assemble tool (maps to fk_type 1: assemble tool)
    if (robot_id_handovering == 0) fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false); // Assuming assemble_r1 is handover_assemble_r1
    else fk_cart_T_for_twist = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false); // Assuming assemble_r2 is handover_assemble_r2
    
    Eigen::Matrix4d twist_transform_mat = Eigen::Matrix4d::Identity();
    twist_transform_mat.block<3,3>(0,0) = twist_R_handover_mat;
    Eigen::Matrix4d target_T_transfer_twist = fk_cart_T_for_twist * twist_transform_mat;
    
    calculateIKforLegoPlan(target_T_transfer_twist, current_seed_q_deg, robot_id_handovering, 1, true, q_out_deg, state_out_rad, ik_status); // fk_type 1 (assemble tool)
    if (!ik_status) { overall_reachable = false; log("IK failed for Handover Transfer Twist", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);
    prev_interpolated_state_rad = state_out_rad;
    current_seed_q_deg = q_out_deg;

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Handover Transfer Twist", LogLevel::WARN);
        return false;
    }




    // 4. Transfer Twist Up
    Eigen::Matrix4d fk_cart_T_twisted;
    // FK with handover_assemble tool (fk_type 1)
    if (robot_id_handovering == 0) fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r1(), lego_ptr_->robot_base_r1(), false);
    else fk_cart_T_twisted = lego_manipulation::math::FK(current_seed_q_deg, lego_ptr_->robot_DH_tool_assemble_r2(), lego_ptr_->robot_base_r2(), false);
        
    Eigen::Matrix4d target_T_transfer_twist_up = fk_cart_T_twisted;
    target_T_transfer_twist_up(2,3) += 0.015; // Z up by 1.5cm

    calculateIKforLegoPlan(target_T_transfer_twist_up, current_seed_q_deg, robot_id_handovering, 1, true, q_out_deg, state_out_rad, ik_status); // fk_type 1
    if (!ik_status) { overall_reachable = false; log("IK failed for Handover Transfer Twist Up", LogLevel::WARN); return overall_reachable; }
    interpolate_segment(prev_interpolated_state_rad, state_out_rad, traj);

    check_states = { state_out_rad };
    if (instance_->checkCollision(check_states, false)) {
        log("Collision detected at Handover Transfer Twist Up", LogLevel::WARN);
        return false;
    }


    return overall_reachable;
}

}