//
// Created by shuoy on 10/19/21.
//
//通过不同的控制模块来实现机器人的运动控制
// 头文件引用
#include <iostream>
#include <string>
#include <chrono>

// ROS相关
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// osqp-eigen库，求解二次规划（QP）问题
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

// 自定义参数、状态、工具等文件
#include "A1Params.h"
#include "A1CtrlStates.h"
#include "utils/Utils.h"
#include "ConvexMpc.h"
#include "utils/filter.hpp"

// A1RobotControl类定义
class A1RobotControl {
public:
    A1RobotControl();  // 默认构造函数

    A1RobotControl(ros::NodeHandle &_nh);  // 构造函数，带有ROS节点句柄

    // 更新控制计划
    void update_plan(A1CtrlStates &state, double dt);

    // 生成摆动腿控制命令
    void generate_swing_legs_ctrl(A1CtrlStates &state, double dt);

    // 计算关节扭矩
    void compute_joint_torques(A1CtrlStates &state);

    // 计算地面反作用力（GRF）
    Eigen::Matrix<double, 3, NUM_LEG> compute_grf(A1CtrlStates &state, double dt);

    // 计算机器人当前行走的表面角度（地面角度）
    Eigen::Vector3d compute_walking_surface(A1CtrlStates &state);

private:
    // 每个腿的Bezier曲线工具类，用于计算步态规划
    BezierUtils bezierUtils[NUM_LEG];

    // 根部加速度（6维：3维线性加速度，3维角加速度）
    Eigen::Matrix<double, 6, 1> root_acc;

    // 控制器的权重矩阵 Q 和 R，用于MPC求解
    Eigen::DiagonalMatrix<double, 6> Q;  // 权重矩阵 Q
    double R;  // 控制输入的权重

    // 摩擦系数
    double mu;

    // 最小和最大地面反作用力
    double F_min;
    double F_max;

    // 定义二次规划问题的QP矩阵和向量
    Eigen::SparseMatrix<double> hessian;  // Hessian矩阵
    Eigen::VectorXd gradient;  // 梯度向量
    Eigen::SparseMatrix<double> linearMatrix;  // 线性矩阵
    Eigen::VectorXd lowerBound;  // 下界
    Eigen::VectorXd upperBound;  // 上界

    // QP求解器
    OsqpEigen::Solver solver;
'''
ROS调试主题（用于发布控制过程中的一些数据）
'''
    ros::NodeHandle nh;

    // 各腿的ROS发布器，用于发布足端状态信息
    ros::Publisher pub_foot_start[NUM_LEG];  // 足端起始点
    ros::Publisher pub_foot_end[NUM_LEG];  // 足端结束点
    ros::Publisher pub_foot_path[NUM_LEG];  // 足端轨迹路径

    // 足端相关的ROS Marker信息（用于可视化）
    visualization_msgs::Marker foot_start_marker[NUM_LEG];
    visualization_msgs::Marker foot_end_marker[NUM_LEG];
    visualization_msgs::Marker foot_path_marker[NUM_LEG];

    // 其他调试主题
    ros::Publisher pub_terrain_angle;  // 地面角度

    // 足端目标位置（不同腿的目标）
    ros::Publisher pub_foot_pose_target_FL;
    ros::Publisher pub_foot_pose_target_FR;
    ros::Publisher pub_foot_pose_target_RL;
    ros::Publisher pub_foot_pose_target_RR;

    // 足端相对位置（不同腿的相对目标）
    ros::Publisher pub_foot_pose_target_rel_FL;
    ros::Publisher pub_foot_pose_target_rel_FR;
    ros::Publisher pub_foot_pose_target_rel_RL;
    ros::Publisher pub_foot_pose_target_rel_RR;

    // 足端位置误差
    ros::Publisher pub_foot_pose_error_FL;
    ros::Publisher pub_foot_pose_error_FR;
    ros::Publisher pub_foot_pose_error_RL;
    ros::Publisher pub_foot_pose_error_RR;

    // 欧拉角信息
    ros::Publisher pub_euler;

    // MPC不在前10个ticks内启动，以防止未初始化的NaN进入关节扭矩计算
    int mpc_init_counter;

    // 是否使用模拟时间
    std::string use_sim_time;

    // 滤波器（用于平滑和滤波）
    MovingWindowFilter terrain_angle_filter;  // 地面角度滤波器
    MovingWindowFilter recent_contact_x_filter[NUM_LEG];  // 每个腿的最近接触点的x方向滤波器
    MovingWindowFilter recent_contact_y_filter[NUM_LEG];  // 每个腿的最近接触点的y方向滤波器
    MovingWindowFilter recent_contact_z_filter[NUM_LEG];  // 每个腿的最近接触点的z方向滤波器
};



#endif //A1_CPP_A1ROBOTCONTROL_H
