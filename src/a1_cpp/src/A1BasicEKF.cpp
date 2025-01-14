// Created by shuoy on 11/1/21.

#include "A1BasicEKF.h"

// 默认构造函数
A1BasicEKF::A1BasicEKF () {
    // 初始化 3x3 单位矩阵
    eye3.setIdentity();
    
    // 初始化观测矩阵 C
    C.setZero();
    for (int i = 0; i < NUM_LEG; ++i) {
        // 足部位置残差部分（负的位置）
        C.block<3, 3>(i * 3, 0) = -eye3;  // -pos
        C.block<3, 3>(i * 3, 6 + i * 3) = eye3;  // foot pos
        // 足部速度残差部分
        C.block<3, 3>(NUM_LEG * 3 + i * 3, 3) = eye3;  // vel
        // 足部高度残差部分
        C(NUM_LEG * 6 + i, 6 + i * 3 + 2) = 1;  // height z of foot
    }

    // 初始化过程噪声协方差矩阵 Q
    Q.setIdentity();
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * eye3;  // IMU的位置噪声
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * eye3;  // IMU的速度噪声
    for (int i = 0; i < NUM_LEG; ++i) {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = PROCESS_NOISE_PFOOT * eye3;  // 足部位置的过程噪声
    }

    // 初始化测量噪声矩阵 R
    R.setIdentity();
    for (int i = 0; i < NUM_LEG; ++i) {
        // 计算与足部位置相关的传感器噪声
        R.block<3, 3>(i * 3, i * 3) = SENSOR_NOISE_PIMU_REL_FOOT * eye3;  // fk位置估计噪声
        // 计算与足部速度相关的传感器噪声
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // fk速度估计噪声
        // 计算足部高度的传感器噪声
        R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = SENSOR_NOISE_ZFOOT;  // 足部高度估计噪声
    }

    // 状态转移矩阵 A 初始化为单位矩阵
    A.setIdentity();

    // 控制输入矩阵 B 初始化为零矩阵
    B.setZero();

    // 假设地面平坦
    assume_flat_ground = true;
}

// 带有假设地面平坦标志的构造函数
A1BasicEKF::A1BasicEKF (bool assume_flat_ground_): A1BasicEKF() {
    assume_flat_ground = assume_flat_ground_;
    
    // 如果不假设地面平坦，更新高度估计的噪声矩阵 R
    if (assume_flat_ground == false) {
        for (int i = 0; i < NUM_LEG; ++i) {
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = 1e5;  // 高度估计不可靠
        }
    }
}

// 初始化状态
void A1BasicEKF::init_state(A1CtrlStates& state) {
    filter_initialized = true;
    
    // 设置状态协方差矩阵 P
    P.setIdentity();  //设置单位阵
    P = P * 3;

    // 设置状态向量 x 的初始值
    x.setZero();
    x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.09);  // 初始化根位置（假设初始位置为0,0,0.09）  （0,0,0.09,...）[18*1]

    // 设置每个足部位置
    for (int i = 0; i < NUM_LEG; ++i) {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);  // 获取每个足部的位置（3*4取3*1向量）  in the robot frame
        x.segment<3>(6 + i * 3) = state.root_rot_mat * fk_pos + x.segment<3>(0);  // 根旋转矩阵乘以足部位置  世界坐标系（maybe）
    }
}

// 更新状态估计
void A1BasicEKF::update_estimation(A1CtrlStates& state, double dt) {
    // 根据最新的时间增量 dt 更新 A 和 B(27*27)
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;

    // 控制输入 u = 旋转矩阵与加速度传感器数据的乘积 + 重力加速度
    Eigen::Vector3d u = state.root_rot_mat * state.imu_acc + Eigen::Vector3d(0, 0, -9.81);//三维分量相加（x,y,z)

    // 足部接触估计
    if (state.movement_mode == 0) {  // 0 站立状态
        for (int i = 0; i < NUM_LEG; ++i) estimated_contacts[i] = 1.0;  // 所有足部都认为接触地面
    } else {  // 行走状态
        for (int i = 0; i < NUM_LEG; ++i) {
            estimated_contacts[i] = std::min(std::max((state.foot_force(i)) / (100.0 - 0.0), 0.0), 1.0);  // 足部接触估计基于足部力,被限制在0.0和1.0之间，表示足部接触地面的概率或程度
        }
    }

    // 更新过程噪声协方差矩阵 Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;  // IMU位置噪声更新
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;  // IMU速度噪声更新
    
    // 更新足部位置的噪声协方差 Q 和传感器噪声矩阵 R
    for (int i = 0; i < NUM_LEG; ++i) {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3;  // 足部位置噪声
        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;  // 足部位置估计噪声
        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;  // 足部速度估计噪声
        if (assume_flat_ground) {
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;  // 足部高度估计噪声
        }
    }
"""
估计值更新
"""
    // 过程更新
    xbar = A * x + B * u;  // 预测的状态
    Pbar = A * P * A.transpose() + Q;  // 更新后的状态协方差
    // 构造测量估计值
    yhat = C * xbar;

    // 获取实际测量值
    for (int i = 0; i < NUM_LEG; ++i) {
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3, 1>(0, i);  // 获取足部位置
        y.block<3, 1>(i * 3, 0) = state.root_rot_mat * fk_pos;  // fk估计
        Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3, 1>(0, i) - Utils::skew(state.imu_ang_vel) * fk_pos;  // 计算足部速度
        y.block<3, 1>(NUM_LEG * 3 + i * 3, 0) = (1.0 - estimated_contacts[i]) * x.segment<3>(3) + estimated_contacts[i] * state.root_rot_mat * leg_v;  // 速度估计
        y(NUM_LEG*6+i) =
                (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;                               // height z estimation
    }
"""
    以下四行直至Pbar * C.transpose() * Serror_y，求K
"""
    S = C * Pbar *C.transpose() + R; //创新协方差矩阵（或预拟合残差协方差矩阵）K的一部分
    S = 0.5*(S+S.transpose());
    error_y = y - yhat;
    Serror_y = S.fullPivHouseholderQr().solve(error_y);  //QR求解线性方程组 S * Serror_y = error_y--->Serror_y=S-1*error_y
    x = xbar + Pbar * C.transpose() * Serror_y;
"""
更新P，并化为对角阵
"""
    SC = S.fullPivHouseholderQr().solve(C);  ////QR求解线性方程组 S * SC = C--->SC=S-1*C
    P = Pbar - Pbar * C.transpose() * SC * Pbar;
    P = 0.5 * (P + P.transpose()); //变为对角阵
"""
// reduce position drift
// 检查矩阵P的前2x2子块的行列式是否大于1e-6
"""
if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
    // 如果行列式大于阈值，则将P矩阵中与位置估计相关的部分置零
    // 这可能是在减少位置估计的漂移
    P.block<2, 16>(0, 2).setZero(); // 将第1行到第2行，第3列到第18列的元素置零
    P.block<16, 2>(2, 0).setZero(); // 将第3行到第18行，第1列到第2列的元素置零
    P.block<2, 2>(0, 0) /= 10.0;    // 将前2x2子块的元素除以10，可能是在缩放协方差
}

"""final step
    将估计的值赋回到结构体A1CtrlStates的实例state中
"""
for (int i = 0; i < NUM_LEG; ++i) {
    // 检查每个腿的接触估计值是否小于0.5
    if (estimated_contacts[i] < 0.5) {
        // 如果小于0.5，则认为该腿没有接触，设置相应的状态为false
        state.estimated_contacts[i] = false;
    } else {
        // 否则，认为该腿有接触，设置相应的状态为true
        state.estimated_contacts[i] = true;
    }
}

// 将估计的根位置和速度赋值给state结构体
state.estimated_root_pos = x.segment<3>(0); // 获取状态向量x的前3个元素，代表估计的根位置
state.estimated_root_vel = x.segment<3>(3); // 获取状态向量x的第4到第6个元素，代表估计的根速度

// 将估计的根位置和线速度再次赋值给state结构体，可能与上面的赋值重复
state.root_pos = x.segment<3>(0);          // 估计的根位置
state.root_lin_vel = x.segment<3>(3);      // 估计的根线速度
