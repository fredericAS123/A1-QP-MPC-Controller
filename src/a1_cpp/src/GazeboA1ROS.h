//
// Created by zixin on 11/1/21.
//

#ifndef A1_CPP_GAZEBOA1ROS_H
#define A1_CPP_GAZEBOA1ROS_H

// std
#include <Eigen/Dense>  // 引入 Eigen 库，用于矩阵和线性代数操作
#include <memory>  // 引入智能指针库
#include <set>  // 引入集合数据结构
#include <chrono>  // 引入时间相关库
#include <map>  // 引入映射数据结构
#include <mutex>  // 引入互斥量，用于多线程同步
#include <thread>  // 引入线程库
#include <condition_variable>  // 引入条件变量，用于线程间同步
#include <fstream>  // 引入文件操作库

// ROS
#include <ros/ros.h>  // ROS 核心库
#include <sensor_msgs/Joy.h>  // 用于接收遥控器输入
#include <sensor_msgs/Imu.h>  // 用于接收IMU数据
#include <sensor_msgs/JointState.h>  // 用于接收关节状态
#include <nav_msgs/Odometry.h>  // 用于接收里程计信息
#include <geometry_msgs/PoseStamped.h>  // 用于接收位姿数据
#include <geometry_msgs/TwistStamped.h>  // 用于接收速度数据
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // 用于接收带协方差的位姿数据
#include <geometry_msgs/PoseArray.h>  // 用于接收位姿数组
#include <geometry_msgs/Vector3Stamped.h>  // 用于接收向量数据
#include <geometry_msgs/WrenchStamped.h>  // 用于接收外力和力矩数据
#include <geometry_msgs/PointStamped.h>  // 用于接收点数据
#include <unitree_legged_msgs/MotorState.h>  // 用于接收电机状态数据
#include <unitree_legged_msgs/MotorCmd.h>  // 用于发送电机控制命令
#include <unitree_legged_msgs/LowCmd.h>  // 用于发送低级控制命令
// #include <gazebo_msgs/ModelStates.h>  // 这里注释掉的是 Gazebo 模型状态的相关头文件

// 控制参数
#include "A1Params.h"  // A1 控制参数头文件
#include "A1CtrlStates.h"  // A1 控制状态头文件
#include "A1RobotControl.h"  // A1 机器人控制类头文件
#include "A1BasicEKF.h"  // A1 扩展卡尔曼滤波器类头文件
#include "legKinematics/A1Kinematics.h"  // A1 腿部运动学类头文件
#include "utils/Utils.h"  // 常用工具函数头文件

#include "utils/filter.hpp"  // 滤波器类头文件

class GazeboA1ROS {
public:
    GazeboA1ROS(ros::NodeHandle &_nh);  // 构造函数，初始化 ROS 节点句柄

    bool update_foot_forces_grf(double dt);  // 更新脚部地面反作用力（GRF）

    bool main_update(double t, double dt);  // 主要更新函数，每次循环调用，处理控制和估计任务

    bool send_cmd();  // 发送控制命令到机器人

    // 回调函数
    void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom);  // 处理地面真实位姿数据回调

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);  // 处理IMU数据回调

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);  // 处理遥控器输入回调

    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理前左腿髋关节电机状态

    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理前左腿大腿电机状态

    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理前左腿小腿电机状态

    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理前右腿髋关节电机状态

    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理前右腿大腿电机状态

    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理前右腿小腿电机状态

    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理后左腿髋关节电机状态

    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理后左腿大腿电机状态

    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理后左腿小腿电机状态

    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理后右腿髋关节电机状态

    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理后右腿大腿电机状态

    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);  // 处理后右腿小腿电机状态

    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);  // 处理前左腿接触力传感器数据回调

    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);  // 处理前右腿接触力传感器数据回调

    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);  // 处理后左腿接触力传感器数据回调

    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);  // 处理后右腿接触力传感器数据回调


private:
    ros::NodeHandle nh;  // ROS 节点句柄，用于与 ROS 系统交互

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Publisher pub_joint_cmd[12];  // 12 个关节控制命令发布器
    ros::Subscriber sub_joint_msg[12];  // 12 个关节状态订阅器
    ros::Publisher pub_euler_d;  // 发布机器人目标欧拉角数据

    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];  // 4 个脚部接触力传感器数据订阅器
    ros::Subscriber sub_gt_pose_msg;  // 地面真实位姿数据订阅器
    ros::Subscriber sub_imu_msg;  // IMU 数据订阅器
    ros::Subscriber sub_joy_msg;  // 遥控器数据订阅器

    // debug estimation
    ros::Publisher pub_estimated_pose;  // 用于发布估计的机器人位姿数据

    // 遥控器命令变量
    double joy_cmd_velx = 0.0;  // 遥控器 x 轴速度命令
    double joy_cmd_vely = 0.0;  // 遥控器 y 轴速度命令
    double joy_cmd_velz = 0.0;  // 遥控器 z 轴速度命令

    double joy_cmd_pitch_rate = 0.0;  // 遥控器俯仰角速率命令
    double joy_cmd_roll_rate = 0.0;  // 遥控器横滚角速率命令
    double joy_cmd_yaw_rate = 0.0;  // 遥控器偏航角速率命令

    double joy_cmd_pitch_ang = 0.0;  // 遥控器俯仰角命令
    double joy_cmd_roll_ang = 0.0;  // 遥控器横滚角命令
    double joy_cmd_body_height = 0.3;  // 遥控器命令的机器人身体高度

    // 0 是站立，1 是行走
    int joy_cmd_ctrl_state = 0;  // 遥控器命令的控制状态
    bool joy_cmd_ctrl_state_change_request = false;  // 是否请求控制状态变化
    int prev_joy_cmd_ctrl_state = 0;  // 上一次控制状态
    bool joy_cmd_exit = false;  // 是否退出遥控器控制

    // 机器人腿部运动学相关参数
    Eigen::Vector3d p_br;  // IMU 坐标系到机器人身体坐标系的平移向量
    Eigen::Matrix3d R_br;  // IMU 坐标系到机器人身体坐标系的旋转矩阵
    double leg_offset_x[4] = {};  // 机器人四条腿的 x 偏移量
    double leg_offset_y[4] = {};  // 机器人四条腿的 y 偏移量
    double motor_offset[4] = {};  // 机器人四条腿的电机偏移量
    double upper_leg_length[4] = {};  // 机器人四条腿的上腿长度
    double lower_leg_length[4] = {};  // 机器人四条腿的下腿长度
    std::vector<Eigen::VectorXd> rho_fix_list;  // 固定腿部的参数列表
    std::vector<Eigen::VectorXd> rho_opt_list;  // 优化后的腿部参数列表
    A1Kinematics a1_kin;  // A1 机器人腿部运动学计算类

    // 控制相关变量
    A1CtrlStates a1_ctrl_states;  // 机器人控制状态
    A1RobotControl _root_control;  // 机器人根部控制类
    A1BasicEKF a1_estimate;  // A1 扩展卡尔曼滤波器

    // 滤波器变量
    MovingWindowFilter acc_x;  // x 轴加速度滤波器
    MovingWindowFilter acc_y;  // y 轴加速度滤波器
    MovingWindowFilter acc_z;  // z 轴加速度滤波器
    MovingWindowFilter gyro_x;  // x 轴陀螺仪滤波器
    MovingWindowFilter gyro_y;  // y 轴陀螺仪滤波器
    MovingWindowFilter gyro_z;  // z 轴陀螺仪滤波器
    MovingWindowFilter quat_w;  // 四元数 w 分量滤波器
    MovingWindowFilter quat_x;  // 四元数 x 分量滤波器
    MovingWindowFilter quat_y;  // 四元数 y 分量滤波器
    MovingWindowFilter quat_z;  // 四元数 z 分量滤波器
};

#endif //A1_CPP_GAZEBOA1ROS_H
