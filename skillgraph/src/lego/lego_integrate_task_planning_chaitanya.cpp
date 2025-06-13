#include "Utils/FileIO.hpp"
#include "Utils/Math.hpp"
#include "RobotControl.hpp"
// #include "lego/Lego.hpp"
#include "lego/Lego_old.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/WrenchStamped.h>
#include <fstream>
#include <unistd.h>

// HEADER FILES FOR SERVICE INTERFACE WITH YK ROBOT
// #include "geometry_msgs/Pose.h"
// #include "gp4_lego/SetPose.h"
#ifdef HAVE_YK_TASKS
#include "yk_msgs/SetPose.h"
#endif

// HEADER FILES FOR API INTERFACE WITH YK ROBOT
// #include "yk_api/yk_interface.h"

using namespace std::chrono;


#define ROBOT_DOF 6
#define PI 3.14159265358979323846

lego_manipulation::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);

lego_manipulation::math::VectorJd assembly_plate_pose_val = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd kit_plate_pose_val = Eigen::MatrixXd::Zero(6, 1);
lego_manipulation::math::VectorJd fts_val = Eigen::MatrixXd::Zero(6, 1);
bool allok = true;

float tool_offset_x = 0.0;
float tool_offset_y = 0.0;
float tool_offset_theta = 0.0;
float tool_offset_x_record = 0.0;
float tool_offset_y_record = 0.0;
float tool_offset_theta_record = 0.0;
float place_offset_x_record = 0.0;
float place_offset_y_record = 0.0;
float place_offset_theta_record = 0.0;

// cv::Mat image_lego = cv::Mat::zeros(480, 640, CV_8UC3);

void make_a_move_client(ros::NodeHandle nh, geometry_msgs::Pose pose)
{
#ifdef HAVE_YK_TASKS
    ROS_INFO_STREAM("Calling SetPose service");
    geometry_msgs::Pose requestPose = pose;
    // requestPose.position.x = .210;      //.362019;
    // requestPose.position.y = -0.133;    //0;
    // requestPose.position.z = .30;       //0.60;
    // requestPose.orientation.x = 0;  //1.0;  //0.71;
    // requestPose.orientation.y = -1; //-0.023;   //0;
    // requestPose.orientation.z = 0;  //-0.028;   //0.71;
    // requestPose.orientation.w = 0;  //0.02; //0;

    ros::ServiceClient client = nh.serviceClient<yk_msgs::SetPose>("yk_set_pose");
    yk_msgs::SetPose srv;

    srv.request.pose = requestPose;
    srv.request.base_frame = "base_link";
    if(client.call(srv))
    {
        ROS_INFO_STREAM("Pose: "<<srv.response.pose);
    }
#endif
}



void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

void assemblyPlateStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    assembly_plate_pose_val << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}

void kitPlateStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    kit_plate_pose_val << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}

void ftsCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    fts_val << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

void allokCallback(const std_msgs::Bool::ConstPtr &msg)
{
    allok = 1;//msg->data;
}

void toolOffsetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 3) // Ensure the message contains at least two elements
    {
        tool_offset_x = msg->data[0];
        tool_offset_y = msg->data[1];
        tool_offset_theta = msg->data[2];
        // ROS_INFO("Received tool offset: x = %.2f, y = %.2f", tool_offset_x, tool_offset_y);
    }
    else
    {
        ROS_WARN("Received tool offset message with insufficient data.");
    }
}

// void visFrameCallback(const sensor_msgs::Image::ConstPtr& msg)
// {
//     // sensor_msgs::Image vis_frame = *msg;
//     cv_bridge::CvImagePtr cv_ptr;
//     try {
//       cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
//       image_lego = cv_ptr->image;
//     }
//     catch (cv_bridge::Exception& e) {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

int main(int argc, char **argv)
{
    try
    {
        // A. ROS INITIALIZATION
        //*****************************************************************************************
        ros::init(argc, argv, "hongyi_task_planning_node");
        ros::NodeHandle nh;
        ros::NodeHandle private_node_handle("~");
        std::string base_frame;
        std::string config_fname, root_pwd;
        float x_home, y_home;
        unsigned int second = 1000000;
        usleep(2 * second);

        // get params from launch file
        private_node_handle.param<std::string>("base_frame", base_frame, "world");
        ROS_INFO_STREAM("namespace of task_planning nh = " << nh.getNamespace());
        private_node_handle.getParam("config_fname", config_fname);
        private_node_handle.getParam("root_pwd", root_pwd);
        private_node_handle.param<float>("x_home", x_home, 0.3);
        private_node_handle.param<float>("y_home", y_home, 0.0);
        ros::ServiceClient set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();
        ros::Rate loop_rate(150);

        // B. READ CONFIG FILE
        //*****************************************************************************************
        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        std::string DH_fname = root_pwd + config["DH_fname"].asString();
        std::string DH_tool_fname = root_pwd + config["DH_tool_fname"].asString();
        std::string DH_tool_assemble_fname = root_pwd + config["DH_tool_assemble_fname"].asString();
        std::string DH_tool_disassemble_fname = root_pwd + config["DH_tool_disassemble_fname"].asString();
        std::string robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        std::string gazebo_env_setup_fname = root_pwd + config["env_setup_fname"].asString();
        std::string task_fname = root_pwd + config["task_graph_fname"].asString();
        std::string lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();
        bool infinite_tasks = config["Infinite_tasks"].asBool();
        int task_type_val = config["Start_with_assemble"].asInt64();
        bool use_robot = config["Use_robot"].asBool();
        bool use_ik = config["IK"]["Use_IK"].asBool();
        
        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        // C. GP4_LEGO INITIALIZATION
        //*****************************************************************************************
        lego_manipulation::lego::Lego_old::Ptr lego_ptr = std::make_shared<lego_manipulation::lego::Lego_old>();
        lego_ptr->setup(gazebo_env_setup_fname, lego_lib_fname, task_type_val, task_json, 
                               DH_fname, DH_tool_fname, DH_tool_disassemble_fname, DH_tool_assemble_fname, 
                               robot_base_fname, 1, set_state_client);
        assembly_plate_pose_val << lego_ptr->assemble_plate_x(), lego_ptr->assemble_plate_y(), lego_ptr->assemble_plate_z(), 
                                   lego_ptr->assemble_plate_roll(), lego_ptr->assemble_plate_pitch(), lego_ptr->assemble_plate_yaw();
        kit_plate_pose_val << lego_ptr->storage_plate_x(), lego_ptr->storage_plate_y(), lego_ptr->storage_plate_z(),
                              lego_ptr->storage_plate_roll(), lego_ptr->storage_plate_pitch(), lego_ptr->storage_plate_yaw();

        lego_manipulation::robot::RobotControl::Ptr robotcontrol = std::make_shared<lego_manipulation::robot::RobotControl>();
        robotcontrol->Setup(DH_fname, robot_base_fname);
        robotcontrol->set_DH_tool(DH_tool_fname);
        robotcontrol->set_DH_tool_assemble(DH_tool_assemble_fname);
        robotcontrol->set_DH_tool_disassemble(DH_tool_disassemble_fname);
        robotcontrol->print_robot_property();
        Eigen::Matrix<double, 3, 1> cart_goal = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix3d rot_goal = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd yaw_correction_R(3, 3);
        Eigen::MatrixXd yaw_noise_R(4, 4); //artificial noise for testing yaw correction
        Eigen::MatrixXd twist_R(3, 3);
        Eigen::MatrixXd twist_T(4, 4);
        Eigen::MatrixXd cart_T(4, 4);
        Eigen::Matrix4d cam_to_center = Eigen::MatrixXd::Identity(4, 4);//going to grab position on place to drop the block
        cam_to_center.col(3) << 0.00702, -0.0235, -0.01, 1;

        int twist_deg = 14;
        double incremental_deg = twist_deg;
        int twist_num = twist_deg / incremental_deg;

        int num_tasks = task_json.size();
        int twist_idx = 0;
        double ik_step = 10e-3;
        int max_iter = 10e6;
        int grab_brick;
        std_msgs::Float32MultiArray goal_msg;
        std_msgs::Int64 exec_msg;
        Eigen::Matrix4d cart_T_goal;

        int task_idx;

        if (task_type_val)
        {
            task_idx = 1;
        }
        else
        {
            task_idx = num_tasks;
        }

        // D. HOME POSE
        //*****************************************************************************************
        Eigen::MatrixXd home_q(ROBOT_DOF, 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0; // Home
        Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q, robotcontrol->robot_DH(), robotcontrol->robot_base(), false);
        home_T.col(3) << x_home, y_home, 0.5, 1; // Home X, Y, Z in base frame of the Flange (tool0) ??TODO??
        home_q = lego_manipulation::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                    robotcontrol->robot_DH(), robotcontrol->robot_base(), 0, max_iter, ik_step);

        // E. INITIALIZE ROS INTERFACES
        //*****************************************************************************************
        /** TODO: USE SERVICE INTERFACE TO SEND GOAL TO ROBOT
         *  Requires service server node, i.e., yk_tasks/yk_tasks, to be running
         */
#ifdef HAVE_YK_TASKS
        ros::ServiceClient client = nh.serviceClient<yk_msgs::SetPose>("yk_set_pose");
        yk_msgs::SetPose srv;
#endif

        /** TODO: USE YK_API TO SEND GOAL TO ROBOT
         *  Requires CMakeLists.txt to be modified to include yk_api
         */
        // MFI::YK_Interface yk_interface(namespace);

        // VIRTUAL CONTROLLER INTERFACE ??TODO??
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("sim/gp4_lego_bringup/robot_goal", ROBOT_DOF);
        ros::Subscriber robot_state_sub = nh.subscribe("sim/gp4_lego_bringup/robot_state", ROBOT_DOF * 3, robotStateCallback);
        
        // Communication with other integrated modules
        ros::Publisher exec_status_pub = nh.advertise<std_msgs::Int64>("execution_status", 1); // 1: executing 0: idle
        ros::Subscriber assembly_plate_state_sub = nh.subscribe("assembly_plate_state", 6, assemblyPlateStateCallback); // X, Y, Yaw
        ros::Subscriber kit_plate_state_sub = nh.subscribe("kit_plate_state", 6, kitPlateStateCallback); // X, Y, Yaw
        ros::Subscriber fts_sub = nh.subscribe("/fts", 1, ftsCallback);
        ros::Subscriber allok_sub = nh.subscribe("gp4_allok", 1, allokCallback); // 1: all ok, 0: not all ok
        ros::Subscriber tool_offset_sub = nh.subscribe("/tool_offset", 1, toolOffsetCallback);
        // ros::Subscriber vis_frame_sub = nh.subscribe("/vis_frame", 1, visFrameCallback);

        // F. MAIN LOOP
        //*****************************************************************************************
        lego_manipulation::math::VectorJd cur_goal = home_q;
        std::string brick_name;
        bool move_on_to_next = true;

        // Attack angle
        lego_manipulation::math::VectorJd pick_offset = Eigen::MatrixXd::Zero(6, 1);
        pick_offset << -0.005, 0.005, -0.005,  // place brick offset
                       -0.005, 0.005, -0.0163; // grab brick offset
        geometry_msgs::Pose goal_pose;
        int mode = 0; // 0:home 1:pick tilt up 2:pick_up 3:pick_down, 4:pick_twist 5:pick_twist_up 6:home
                      // 7:drop tilt up 8:drop_up 9:drop_down 10:drop_twist 11:drop_twist_up 12:done
        int executing = 0;
        int task_count = 0;
        while (ros::ok)
        {
            // F.1. DETERMINE AND CALCULATE GOAL
            //*************************************************************************************
            
            // // new task coming
            // if (mode > 6 && task_count >= 2){
            //     ROS_INFO_STREAM("Task Execution Done!");
            //     ros::shutdown();
            //     return 0;
            // }
            if(!executing && allok)
            {
                ROS_INFO_STREAM("\n\nReset environment!");
                executing = 1;
                std::ifstream task_file(task_fname, std::ifstream::binary);
                task_file >> task_json;
                lego_ptr->set_assemble_plate_pose(assembly_plate_pose_val[0], assembly_plate_pose_val[1], assembly_plate_pose_val[2], assembly_plate_pose_val[3], assembly_plate_pose_val[4], assembly_plate_pose_val[5]);
                lego_ptr->set_storage_plate_pose(kit_plate_pose_val[0], kit_plate_pose_val[1], kit_plate_pose_val[2], kit_plate_pose_val[3], kit_plate_pose_val[4], kit_plate_pose_val[5]);
                lego_ptr->setup(gazebo_env_setup_fname, lego_lib_fname, task_type_val, task_json, 
                                       DH_fname, DH_tool_fname, DH_tool_disassemble_fname, DH_tool_assemble_fname, 
                                       robot_base_fname, 0, set_state_client);
                                       
                num_tasks = task_json.size();
                if (task_type_val)
                {
                    task_idx = 1;
                    ROS_INFO_STREAM("Assembling!");
                }
                else
                {
                    task_idx = num_tasks;
                    ROS_INFO_STREAM("Disassembling!");
                }
                ROS_INFO_STREAM("Num steps: " << num_tasks);
                cur_goal = home_q;
                move_on_to_next = true;
                // if (task_count == 0)
                // {
                //     mode = 6;
                // }
                // else
                // {
                //     mode = 0;
                // }
                mode = 0; // 0:home 1:pick tilt up 2:pick_up 3:pick_down, 4:pick_twist 5:pick_twist_up 6:home
                      // 7:drop tilt up 8:drop_up 9:drop_down 10:drop_twist 11:drop_twist_up 12:done
                task_count += 1;
            }
            if(executing && allok)
            {
                robotcontrol->set_robot_q(robot_q);
                robotcontrol->set_robot_qd(robot_qd);
                robotcontrol->set_robot_qdd(robot_qdd);
                if (mode >= 4 && mode <= 9)
                {
                        lego_ptr->update_bricks(robot_q, robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), false, brick_name);
                }
                if ((use_robot && move_on_to_next) || (!use_robot && robotcontrol->reached_goal(cur_goal) && robotcontrol->is_static()))
                {
                    if (mode == 9)
                    {
                        lego_ptr->update_brick_connection();
                    }
                    if (mode == 12)
                    {
                        if (task_idx >= num_tasks && task_type_val)
                        {
                            executing = 0;
                        }
                        else if (task_idx <= 1 && !task_type_val)
                        {
                            executing = 0;
                        }
                        mode = 0;
                        if (task_type_val)
                        {
                            task_idx++;
                        }
                        else
                        {
                            task_idx--;
                        }
                        if (task_idx > num_tasks && !infinite_tasks)
                        {
                            break;
                        }
                    }
                    else if (mode == 4 || mode == 10)
                    {
                        twist_idx++;
                        if (twist_idx == twist_num)
                        {
                            mode++;
                            twist_idx = 0;
                        }
                    }
                    //custom mode changes for lego_scope: 0 -> 14 -> 15 -> 16 ->1 -> 2 -> 3 -> 13 -> 4
                    else if (mode == 0){
                        std::cout<<"going to 14";
                        mode = 14;
                    }
                    // else if(mode == 14){
                    //     std::cout<<"going to 15";
                    //     mode = 15;
                    // }
                    else if(mode == 16){
                        std::cout<<"going to 1";
                        mode = 1;
                    }
                    else if(mode == 3){
                        std::cout<<"going to 13";
                        mode = 13;
                    }
                    else if (mode == 13){
                        mode = 4;
                    }
                    else
                    {
                        mode++;
                    }
                    ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx);

                    if (mode == 0 || mode == 6 || mode == 12)
                    {
                        cur_goal = home_q;
                    }
                    else if (mode == 14) // check and record the position of placing the block
                    {
                        auto cur_graph_node = task_json[std::to_string(task_idx)];
                        brick_name = lego_ptr->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
                        grab_brick = 0;//going to drop position first
                        lego_ptr->calc_brick_grab_pose(brick_name, task_type_val, grab_brick,
                                                            cur_graph_node["x"].asInt(),
                                                            cur_graph_node["y"].asInt(),
                                                            cur_graph_node["z"].asInt(),
                                                            cur_graph_node["ori"].asInt(),
                                                            cur_graph_node["press_side"].asInt(), cart_T);
                        Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);//going to grab position on place to drop the block
                        up_T.col(3) << 0.0, 0, pick_offset(5)+0.0096, 1;//0.0096 is the height of the block, XY are place holder
                        up_T = cart_T * up_T;
                        std::cout<<"\n\n\n up_t before "<<up_T<<std::endl;
                        up_T = up_T * cam_to_center;
                        std::cout<<"\n\n\n after "<<up_T<<std::endl;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 15) //record wanted placing offset
                    {
                        std::cout<<"waiting for tool camera read"<<std::endl;
                        usleep(2 * second);
                        Eigen::Matrix4d comp_T = Eigen::MatrixXd::Identity(4, 4);
                        Eigen::Vector3d tool_offset;
                        tool_offset << tool_offset_x, tool_offset_y, 0;
                        tool_offset = tool_offset; //apply rotation to tool offset
                        place_offset_x_record = tool_offset(0);
                        place_offset_y_record = tool_offset(1);
                        place_offset_theta_record = tool_offset_theta;
                        std::cout<<"\n\n\n stored placing off set"<<" x: "<<place_offset_x_record << "y: " << place_offset_y_record<<"theta: " << tool_offset_theta<<"\n\n"<<std::endl;
                        //no change in cur_goal
                    }
                    else if (mode == 16){//go home first after measuring offset
                        Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                        up_T.col(3) << 0, 0, pick_offset(5), 1;//0.0096 is the height of the block
                        up_T = cart_T * up_T;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 1)
                    {
                        auto cur_graph_node = task_json[std::to_string(task_idx)];
                        brick_name = lego_ptr->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
                        grab_brick = 1;
                        lego_ptr->calc_brick_grab_pose(brick_name, task_type_val, grab_brick,
                                                            cur_graph_node["x"].asInt(),
                                                            cur_graph_node["y"].asInt(),
                                                            cur_graph_node["z"].asInt(),
                                                            cur_graph_node["ori"].asInt(),
                                                            cur_graph_node["press_side"].asInt(), cart_T);
                        Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                        offset_T.col(3) << pick_offset(3), pick_offset(4), pick_offset(5) - abs(pick_offset(5)), 1;
                        offset_T = cart_T * offset_T;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, offset_T.block(0, 3, 3, 1), offset_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                        if (!task_type_val && lego_ptr->brick_instock(brick_name))
                        {
                            cur_goal = home_q;
                            mode = 12;
                        }
                    }
                    else if (mode == 2)
                    {
                        Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                        up_T.col(3) << 0, 0, pick_offset(5), 1;
                        up_T = cart_T * up_T;
                        up_T = up_T * cam_to_center;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 3) //now only lineup
                    {
                        std::cout<<"waiting for tool camera read"<<std::endl;
                        usleep(7 * second);
                        Eigen::Matrix4d comp_T = Eigen::MatrixXd::Identity(4, 4);
                        Eigen::Vector3d tool_offset;
                        tool_offset << tool_offset_x, tool_offset_y, 0;
                        tool_offset = tool_offset;
                        tool_offset_x_record = tool_offset(0);
                        tool_offset_y_record = tool_offset(1);

                        comp_T.col(3) << tool_offset_x_record, tool_offset_y_record, pick_offset(5), 1;
                        
                        tool_offset_theta_record = tool_offset_theta;
                        if (abs(tool_offset_x) < 0.005 && abs(tool_offset_y) < 0.005)
                        {
                            comp_T = cart_T * comp_T;
                        }
                        else{
                            std::cout<<"\n\n OFFSET TOO LARGE \n\n";
                            comp_T = cart_T;
                        }
                        
                        
                        std::cout<<"adjusting with offset x: "<<tool_offset_x << "y: " << tool_offset_y<<std::endl;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, comp_T.block(0, 3, 3, 1), comp_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                        


                    }
                    else if (mode == 13) //now pick down
                    {
                        Eigen::Matrix4d comp_T = Eigen::MatrixXd::Identity(4, 4);
                        comp_T.col(3) << tool_offset_x_record, tool_offset_y_record, 0, 1;
                        // yaw_correction_R << cos(tool_offset_theta_record), -sin(tool_offset_theta_record),0,
                        //             sin(tool_offset_theta_record), cos(tool_offset_theta_record), 0,
                        //             0, 0, 1;
                        // comp_T.block(0, 0, 3, 3) << yaw_correction_R;
                        if (abs(tool_offset_x_record) < 0.005 && abs(tool_offset_y_record) < 0.005)
                        {
                            comp_T = cart_T * comp_T;
                        }
                        else{
                            std::cout<<"\n\n OFFSET TOO LARGE \n\n";
                            comp_T = cart_T;
                        }
                        
                        
                        std::cout<<"picking with offset x: "<<tool_offset_x_record << "y: " << tool_offset_y_record<<std::endl;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, comp_T.block(0, 3, 3, 1), comp_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 4)
                    {
                        double twist_rad = (incremental_deg + 2) / 180.0 * PI;
                        twist_R << cos(twist_rad), 0, sin(twist_rad),
                            0, 1, 0,
                            -sin(twist_rad), 0, cos(twist_rad);
                        twist_T = Eigen::MatrixXd::Identity(4, 4);
                        twist_T.block(0, 0, 3, 3) << twist_R;
                        cart_T = lego_manipulation::math::FK(cur_goal, robotcontrol->robot_DH_tool_disassemble(), robotcontrol->robot_base(), false);
                        cart_T = cart_T * twist_T;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool_disassemble(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 5 || mode == 11)
                    {
                        cart_T = lego_manipulation::math::FK(cur_goal, robotcontrol->robot_DH_tool_assemble(), robotcontrol->robot_base(), false);
                        cart_T(2, 3) = cart_T(2, 3) + 0.015;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool_assemble(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 7)
                    {
                        auto cur_graph_node = task_json[std::to_string(task_idx)];
                        brick_name = lego_ptr->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
                        grab_brick = 0;
                        lego_ptr->calc_brick_grab_pose(brick_name, task_type_val, 0,
                                                            cur_graph_node["x"].asInt(),
                                                            cur_graph_node["y"].asInt(),
                                                            cur_graph_node["z"].asInt(),
                                                            cur_graph_node["ori"].asInt(),
                                                            cur_graph_node["press_side"].asInt(), cart_T);
                        Eigen::Matrix4d offset_T = Eigen::MatrixXd::Identity(4, 4);
                        offset_T.col(3) << pick_offset(0), pick_offset(1), pick_offset(2) - abs(pick_offset(2)), 1;
                        offset_T = cart_T * offset_T;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, offset_T.block(0, 3, 3, 1), offset_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 8)
                    {
                        //modifying cart_T directily, offset will be retained until next block calculation
                        Eigen::Matrix4d comp_T = Eigen::MatrixXd::Identity(4, 4);
                        comp_T.col(3) << place_offset_x_record, place_offset_y_record, 0, 1;
                        // yaw_correction_R << cos(place_offset_theta_record), -sin(place_offset_theta_record),0,
                        //             sin(place_offset_theta_record), cos(place_offset_theta_record), 0,
                        //             0, 0, 1;
                        // comp_T.block(0, 0, 3, 3) << yaw_correction_R;
                        if (abs(place_offset_x_record) < 0.005 && abs(place_offset_y_record) < 0.005)
                        {
                            //cart_T = cart_T * comp_T;
                            comp_T = cart_T * comp_T;//Try only do offset at this step so robot push block over when twist
                        }
                        else{
                            std::cout<<"\n\n OFFSET TOO LARGE \n\n";
                            comp_T = cart_T;
                        }

                        Eigen::Matrix4d up_T = Eigen::MatrixXd::Identity(4, 4);
                        up_T.col(3) << 0, 0, pick_offset(2), 1;
                        up_T = comp_T * up_T;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, up_T.block(0, 3, 3, 1), up_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 9)
                    {
                        //cart_T is already modified from mode 8 if offset is valid

                        //modify comp_T again
                        Eigen::Matrix4d comp_T = Eigen::MatrixXd::Identity(4, 4);
                        comp_T.col(3) << place_offset_x_record * 0.6, place_offset_y_record * 0.6, 0, 1;
                        // yaw_correction_R << cos(place_offset_theta_record), -sin(place_offset_theta_record),0,
                        //             sin(place_offset_theta_record), cos(place_offset_theta_record), 0,
                        //             0, 0, 1;
                        // comp_T.block(0, 0, 3, 3) << yaw_correction_R;
                        if (abs(place_offset_x_record) < 0.005 && abs(place_offset_y_record) < 0.005 )
                        {
                            //cart_T = cart_T * comp_T;
                            comp_T = cart_T * comp_T;//Try only do offset at this step so robot push block over when twist
                        }
                        else{
                            std::cout<<"\n\n OFFSET TOO LARGE \n\n";
                            comp_T = cart_T;
                        }

                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, comp_T.block(0, 3, 3, 1), comp_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                    else if (mode == 10)
                    {
                        double twist_rad = incremental_deg / 180.0 * PI;
                        twist_R << cos(-twist_rad), 0, sin(-twist_rad),
                            0, 1, 0,
                            -sin(-twist_rad), 0, cos(-twist_rad);
                        twist_T = Eigen::MatrixXd::Identity(4, 4);
                        twist_T.block(0, 0, 3, 3) << twist_R;
                        cart_T = lego_manipulation::math::FK(cur_goal, robotcontrol->robot_DH_tool_assemble(), robotcontrol->robot_base(), false);
                        cart_T = cart_T * twist_T;
                        if(use_ik)
                        {
                            cur_goal = lego_manipulation::math::IK(cur_goal, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3),
                                                        robotcontrol->robot_DH_tool_assemble(), robotcontrol->robot_base(), 0, max_iter, ik_step);
                        }
                    }
                }

                cart_T_goal = lego_manipulation::math::FK(cur_goal, robotcontrol->robot_DH(), robotcontrol->robot_base(), false);
                Eigen::Matrix3d goal_rot = cart_T_goal.block(0, 0, 3, 3);
                Eigen::Quaterniond quat(goal_rot);
                
                goal_msg.data.clear();
                for (int j = 0; j < ROBOT_DOF; j++)
                {
                    goal_msg.data.push_back(cur_goal(j));
                }
                goal_pub.publish(goal_msg);

                // F.2.2. Robot command
                if (use_robot)
                {
                    
                    
                    auto motion_start = high_resolution_clock::now();
                    auto motion_end = high_resolution_clock::now();
                    // auto motion_start, motion_end;

                    if (move_on_to_next)
                    {
                        goal_pose.position.x = cart_T_goal(0, 3);
                        goal_pose.position.y = cart_T_goal(1, 3);
                        goal_pose.position.z = cart_T_goal(2, 3);
                        goal_pose.orientation.x = quat.x();
                        goal_pose.orientation.y = quat.y();
                        goal_pose.orientation.z = quat.z();
                        goal_pose.orientation.w = quat.w();
                        move_on_to_next = false;
                        ROS_INFO_STREAM("Sending pose: " << goal_pose);
                        motion_start = high_resolution_clock::now();
                    }

#ifdef HAVE_YK_TASKS
                    srv.request.base_frame = "base_link";
                    srv.request.pose = goal_pose;
                    srv.request.max_velocity_scaling_factor = 0.5;
                    srv.request.max_acceleration_scaling_factor = 0.5;
                    if (client.call(srv))
                    {
                        move_on_to_next = true;
                        ROS_INFO_STREAM("Pose Set to: " << srv.response.pose);
                        motion_end = high_resolution_clock::now();
                        auto duration = duration_cast<microseconds>(motion_end - motion_start);
                        ROS_INFO_STREAM("Motion Execution time: " << duration.count() / 1000000.0 << " s");
                    }
#endif
                }
            }
            
            if(!allok)
            {
                ROS_INFO_STREAM("Aborting Task Execution!");
                executing = 0;
                exec_msg.data = executing;
                exec_status_pub.publish(exec_msg);
            } else {
                exec_msg.data = executing;
                exec_status_pub.publish(exec_msg);
            }
        }
        std::cout<<"detected offset of last pick x: "<<tool_offset_x_record << "y: " << tool_offset_y_record <<"theta: " << tool_offset_theta_record <<std::endl;
        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}



