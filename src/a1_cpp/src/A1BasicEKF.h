// Created by shuoy on 11/1/21.

#ifndef A1_CPP_A1BASICEKF_H
#define A1_CPP_A1BASICEKF_H

#include "A1Params.h"
#include "A1CtrlStates.h"
#include "utils/Utils.h"

// 状态估计器参数
#define STATE_SIZE 18  // 机器人状态向量的大小，包含位置、速度、足部位置等
#define MEAS_SIZE 28   // 测量向量的大小，包含各个足部的位置、速度残差等
#define PROCESS_NOISE_PIMU 0.01  // 过程噪声，IMU的位置
#define PROCESS_NOISE_VIMU 0.01  // 过程噪声，IMU的速度
#define PROCESS_NOISE_PFOOT 0.01  // 过程噪声，足部位置
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001  // 传感器测量噪声，相对IMU的位置
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1  // 传感器测量噪声，相对IMU的速度
#define SENSOR_NOISE_ZFOOT 0.001  // 传感器测量噪声，足部的高度

// 基本误差状态卡尔曼滤波器（EKF），用于估计机器人位姿
// 假设IMU提供了已知的姿态（state.root_rot_mat）
class A1BasicEKF {
public:
    A1BasicEKF ();  // 构造函数
    A1BasicEKF (bool assume_flat_ground_);  // 可选构造函数，假设平地
    void init_state(A1CtrlStates& state);  // 初始化状态
    void update_estimation(A1CtrlStates& state, double dt);  // 更新状态估计
    bool is_inited() { return filter_initialized; }  // 判断滤波器是否已初始化

private:
    bool filter_initialized = false;  // 滤波器是否初始化标志

    // 状态向量
    // 0 1 2 位置 (x, y, z)
    // 3 4 5 速度 (vx, vy, vz)
    // 6 7 8 足部位置 FL (左前)
    // 9 10 11 足部位置 FR (右前)
    // 12 13 14 足部位置 RL (左后)
    // 15 16 17 足部位置 RR (右后)
    Eigen::Matrix<double, STATE_SIZE, 1> x;  // 估计的状态
    Eigen::Matrix<double, STATE_SIZE, 1> xbar;  // 过程更新后的估计状态
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;  // 状态协方差矩阵
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar;  // 过程更新后的状态协方差矩阵
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A;  // 状态转移矩阵
    Eigen::Matrix<double, STATE_SIZE, 3> B;  // 控制输入的影响矩阵
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;  // 过程噪声协方差矩阵

    // 观测量向量
    // 0 1 2   FL位置残差
    // 3 4 5   FR位置残差
    // 6 7 8   RL位置残差
    // 9 10 11 RR位置残差
    // 12 13 14 来自FL的速度残差
    // 15 16 17 来自FR的速度残差
    // 18 19 20 来自RL的速度残差
    // 21 22 23 来自RR的速度残差
    // 24 25 26 27 足部高度残差
    Eigen::Matrix<double, MEAS_SIZE, 1> y;  // 测量值
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat;  // 估计的测量值
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y;  // 测量值残差
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;  // S^-1*error_y，用于卡尔曼增益的计算
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C;  // 观测矩阵
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC;  // S^-1*C，用于卡尔曼增益的计算
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;  // 观测噪声协方差矩阵

    // 辅助矩阵
    Eigen::Matrix<double, 3, 3> eye3;  // 3x3 单位矩阵
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S;  // 创新协方差矩阵（或预拟合残差协方差矩阵）
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K;  // 卡尔曼增益

    bool assume_flat_ground = false;  // 是否假设地面平坦（用于足部力估计时使用），若地面不平坦，则高度不可靠。更新R

    // 处理足部力的变量
    double smooth_foot_force[4];  // 平滑后的足部力数据
    double estimated_contacts[4];  // 估计的接触状态（每个足部的接触情况）
};

#endif // A1_CPP_A1BASICEKF_H
