#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "gazebo_msgs/SetModelState.h"

namespace lego_manipulation
{
namespace lego
{
/**
 * @brief Data structure representing a Lego brick (legacy version).
 */
struct lego_brick{
    std::string brick_name; ///< Name of the brick
    int height; ///< Height of the brick
    int width; ///< Width of the brick
    double x; ///< X position
    double y; ///< Y position
    double z; ///< Z position
    double quat_x; ///< Quaternion X
    double quat_y; ///< Quaternion Y
    double quat_z; ///< Quaternion Z
    double quat_w; ///< Quaternion W
    double cur_x; ///< Current X position
    double cur_y; ///< Current Y position
    double cur_z; ///< Current Z position
    int press_side; ///< Side to press
    Eigen::Quaterniond cur_quat; ///< Current orientation as quaternion
    bool in_stock; ///< Whether the brick is in stock
    std::map<std::string, std::string> top_connect; ///< Top connections
    std::map<std::string, std::string> bottom_connect; ///< Bottom connections
};

/**
 * @brief Data structure representing a Lego plate (legacy version).
 */
struct lego_plate{
    int height; ///< Height of the plate
    int width; ///< Width of the plate
    double x; ///< X position
    double y; ///< Y position
    double z; ///< Z position
    double roll; ///< Roll angle
    double pitch; ///< Pitch angle
    double yaw; ///< Yaw angle
    Eigen::Quaterniond quat; ///< Orientation as quaternion
    Eigen::Matrix4d pose; ///< Pose matrix
};

/**
 * @brief Legacy class for Lego manipulation and environment setup.
 */
class Lego_old
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Lego_old> Ptr; ///< Shared pointer type
        typedef std::shared_ptr<Lego_old const> ConstPtr; ///< Const shared pointer type

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        std::map<std::string, lego_brick> brick_map_; ///< Map of bricks
        ros::ServiceClient client_; ///< ROS service client
        gazebo_msgs::SetModelState setmodelstate_; ///< Model state message
        double brick_height_m_ = 0.0096; ///< Brick height in meters
        double brick_len_offset_ = 0.0002; ///< Brick length offset
        double P_len_ = 0.008; ///< Parameter length
        double EPS_ = 0.00001; ///< Epsilon value for calculations
        double knob_height_ = 0.0017; ///< Knob height
        lego_plate assemble_plate_; ///< Assembly plate
        lego_plate storage_plate_; ///< Storage plate
        int robot_dof_ = 6; ///< Robot degrees of freedom

        Eigen::MatrixXd DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d ee_inv_, tool_inv_, tool_assemble_inv_, tool_disassemble_inv_;
        Eigen::MatrixXd DH_tool_;
        Eigen::MatrixXd DH_tool_assemble_;
        Eigen::MatrixXd DH_tool_disassemble_;
        Eigen::MatrixXd base_frame_;
        Eigen::Matrix4d T_base_inv_;

        void update_all_top_bricks(const std::string& brick_name, const Eigen::Matrix4d& dT);
        void update(const std::string& brick_name, const Eigen::Matrix4d& T);
        void calc_brick_loc(const lego_brick& brick, const lego_plate& plate, const int& orientation,
                            const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z, 
                            Eigen::Matrix4d& out_pose);
        bool is_top_connect(const lego_brick& b1, const lego_brick& b2);
        bool is_bottom_connect(const lego_brick& b1, const lego_brick& b2);
        bool bricks_overlap(const lego_brick& b1, const lego_brick& b2);
        void get_brick_corners(const lego_brick& b1, double& lx, double& ly, double& rx, double& ry);
        void brick_dimension_from_name(const std::string& b_name, int& height, int& width, const Json::Value& lego_lib);


    public:
        Lego_old();
        ~Lego_old(){}
                
        // Operations
        void setup(const std::string& env_setup_fname, const std::string& lego_lib_fname, const bool& assemble, const Json::Value& task_json, 
                   const std::string& DH_fname, const std::string& DH_tool_fname, const std::string& DH_tool_disassemble_fname, const std::string& DH_tool_assemble_fname, 
                   const std::string& base_fname, const bool& use_config_file, const ros::ServiceClient& cli);
        void set_robot_base(const std::string& base_fname);
        void set_DH(const std::string& DH_fname);
        void set_DH_tool(const std::string& DH_tool_fname);
        void set_DH_tool_assemble(const std::string& DH_tool_assemble_fname);
        void set_DH_tool_disassemble(const std::string& DH_tool_disassemble_fname);
        void print_manipulation_property();
        void set_assemble_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
        void set_storage_plate_pose(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);

        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const std::string& brick_name);
        std::string get_brick_name_by_id(const int& id, const int& seq_id);
        void update_brick_connection();
        void calc_brick_grab_pose(const std::string& name, const bool& assemble_pose, const bool& take_brick,
                                  const int& brick_assemble_x, const int& brick_assemble_y, const int& brick_assemble_z, 
                                  const int& orientation, const int& press_side, Eigen::MatrixXd& T);
        int brick_instock(const std::string& name) {return brick_map_[name].in_stock;};
        int robot_dof() {return robot_dof_;};
        Eigen::MatrixXd robot_DH() {return DH_;};
        Eigen::MatrixXd robot_DH_tool() {return DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble() {return DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble() {return DH_tool_disassemble_;};
        Eigen::MatrixXd robot_base() {return base_frame_;};
        Eigen::Matrix4d robot_base_inv() {return T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv() {return ee_inv_;};
        Eigen::Matrix4d robot_tool_inv() {return tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv() {return tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv() {return tool_disassemble_inv_;};
        bool robot_is_static(math::VectorJd robot_qd, math::VectorJd robot_qdd);
        bool robot_reached_goal(math::VectorJd robot_q, math::VectorJd goal);
        double assemble_plate_x() {return assemble_plate_.x;};
        double assemble_plate_y() {return assemble_plate_.y;};
        double assemble_plate_z() {return assemble_plate_.z;};
        double assemble_plate_roll() {return assemble_plate_.roll;};
        double assemble_plate_pitch() {return assemble_plate_.pitch;};
        double assemble_plate_yaw() {return assemble_plate_.yaw;};
        double storage_plate_x() {return storage_plate_.x;};
        double storage_plate_y() {return storage_plate_.y;};
        double storage_plate_z() {return storage_plate_.z;};
        double storage_plate_roll() {return storage_plate_.roll;};
        double storage_plate_pitch() {return storage_plate_.pitch;};
        double storage_plate_yaw() {return storage_plate_.yaw;};
};
}
}