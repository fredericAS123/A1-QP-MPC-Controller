//
// Created by shuoy on 10/19/21.
//

#include "A1RobotControl.h"

A1RobotControl::A1RobotControl() {
    std::cout << "init A1RobotControl" << std::endl;
    // init QP solver
    // init some parameters
    Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
    R = 1e-3;
    mu = 0.7;
    F_min = 0;
    F_max = 180;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    // init mpc skip counter（标志位）
    mpc_init_counter = 0;

   // 约束矩阵固定
for (int i = 0; i < NUM_LEG; ++i) {
    // 提取 F_zi（法向力）位置，在矩阵的 (i, 2 + i * 3) 位置设为1
    linearMatrix.insert(i, 2 + i * 3) = 1;

    // 摩擦金字塔的第一个约束条件：F_xi < uF_zi
    // 这里通过设置线性约束：x方向的力小于摩擦系数乘以法向力
    linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
    linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;  // mu为摩擦系数
    lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY; // 下界设置为负无穷

    // 第二个约束条件：-F_xi < uF_zi  ==>  -F_xi -uF_zi < 0
    // 通过负号设置，使得 F_xi 和摩擦力的约束满足：F_xi + uF_zi <= 0
    linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
    linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
    lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;

    // 第三个约束条件：F_yi < uF_zi
    // 设置 y 方向上的力小于摩擦系数乘以法向力
    linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
    linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
    lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;

    // 第四个约束条件：-F_yi < uF_zi  ==>  -F_yi -uF_zi < 0
    // 同样的设置 y 方向上的力与摩擦力的关系
    linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
    linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
    lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
}

// 调试输出矩阵
// std::cout << Eigen::MatrixXd(linearMatrix) << std::endl;

// 初始化移动窗口滤波器用于过滤地形角度
terrain_angle_filter = MovingWindowFilter(100);

// 为每条腿初始化过滤器，用于分别对每条腿的接触点坐标（x, y, z）进行滤波
for (int i = 0; i < NUM_LEG; ++i) {
    recent_contact_x_filter[i] = MovingWindowFilter(60);
    recent_contact_y_filter[i] = MovingWindowFilter(60);
    recent_contact_z_filter[i] = MovingWindowFilter(60);
}

}

// A1RobotControl 类的构造函数
A1RobotControl::A1RobotControl(ros::NodeHandle &_nh) : A1RobotControl() {
    std::cout << "init nh" << std::endl;  // 输出日志，表示初始化 NodeHandle

    nh = _nh;  // 将外部传入的 NodeHandle 保存为成员变量
    _nh.param("use_sim_time", use_sim_time);  // 获取参数，判断是否使用仿真时间

    // 初始化调试发布器，分别用于每条腿的起始位置、结束位置和摆动路径
    for (int i = 0; i < NUM_LEG; ++i) {
        std::string id = std::to_string(i);  // 每条腿的ID
        std::string start_topic = "/isaac_a1/foot" + id + "/start_pos";  // 起始位置的ROS话题
        std::string end_topic = "/isaac_a1/foot" + id + "/end_pos";      // 结束位置的ROS话题
        std::string path_topic = "/isaac_a1/foot" + id + "/swing_path";  // 摆动路径的ROS话题

        // 初始化发布器，用于发布Marker消息
        pub_foot_start[i] = nh.advertise<visualization_msgs::Marker>(start_topic, 100);
        pub_foot_end[i] = nh.advertise<visualization_msgs::Marker>(end_topic, 100);
        pub_foot_path[i] = nh.advertise<visualization_msgs::Marker>(path_topic, 100);

        // 设置起始位置Marker的基本信息
        foot_start_marker[i].header.frame_id = "a1_world";  // 设置坐标系为 "a1_world"
        foot_start_marker[i].ns = "basic_shapes";  // 设置Marker的命名空间
        foot_start_marker[i].id = 10 + i;  // 设置Marker的ID，保证唯一性
        foot_start_marker[i].type = visualization_msgs::Marker::CYLINDER;  // 设置Marker类型为圆柱体
        foot_start_marker[i].action = visualization_msgs::Marker::ADD;  // 设置动作为添加
        foot_start_marker[i].scale.x = 0.08;  // 设置Marker的X轴尺寸（单位米）
        foot_start_marker[i].scale.y = 0.08;  // 设置Marker的Y轴尺寸（单位米）
        foot_start_marker[i].scale.z = 0.02;  // 设置Marker的Z轴尺寸（单位米）
        foot_start_marker[i].pose.orientation.x = 0.0;  // 设置姿态（方向）为无旋转
        foot_start_marker[i].pose.orientation.y = 0.0;
        foot_start_marker[i].pose.orientation.z = 0.0;
        foot_start_marker[i].pose.orientation.w = 1.0;
        foot_start_marker[i].color.r = 1.0f;  // 设置颜色为红色
        foot_start_marker[i].color.g = 0.0f;
        foot_start_marker[i].color.b = 0.0f;
        foot_start_marker[i].color.a = 1.0;  // 设置透明度为不透明

        // 设置结束位置Marker的基本信息
        foot_end_marker[i].header.frame_id = "a1_world";  // 设置坐标系为 "a1_world"
        foot_end_marker[i].ns = "basic_shapes";  // 设置命名空间
        foot_end_marker[i].id = 20 + i;  // 设置Marker的ID，保证唯一性
        foot_end_marker[i].type = visualization_msgs::Marker::CYLINDER;  // 设置Marker类型为圆柱体
        foot_end_marker[i].action = visualization_msgs::Marker::ADD;  // 设置动作为添加
        foot_end_marker[i].scale.x = 0.08;  // 设置X轴尺寸（单位米）
        foot_end_marker[i].scale.y = 0.08;  // 设置Y轴尺寸（单位米）
        foot_end_marker[i].scale.z = 0.02;  // 设置Z轴尺寸（单位米）
        foot_end_marker[i].color.r = 0.0f;  // 设置颜色为蓝色
        foot_end_marker[i].color.g = 0.0f;
        foot_end_marker[i].color.b = 1.0f;
        foot_end_marker[i].color.a = 1.0;  // 设置透明度为不透明

        // 设置路径Marker的基本信息
        foot_path_marker[i].header.frame_id = "a1_world";  // 设置坐标系为 "a1_world"
        foot_path_marker[i].ns = "basic_shapes";  // 设置命名空间
        foot_path_marker[i].id = 30 + i;  // 设置Marker的ID，保证唯一性
        foot_path_marker[i].type = visualization_msgs::Marker::LINE_STRIP;  // 设置Marker类型为线条
        foot_path_marker[i].action = visualization_msgs::Marker::ADD;  // 设置动作为添加
        foot_path_marker[i].scale.x = 0.02;  // 设置线条宽度
        foot_path_marker[i].points.resize(10);  // 设置路径点的数量为10
        foot_path_marker[i].colors.resize(10);  // 设置路径点颜色数组
        for (int k = 0; k < 10; k++) {
            foot_path_marker[i].colors[k].r = 0.0f;  // 设置颜色为绿色
            foot_path_marker[i].colors[k].g = 1.0f;
            foot_path_marker[i].colors[k].b = 0.0f;
            foot_path_marker[i].colors[k].a = 1.0f;  // 设置透明度为不透明
        }

        foot_path_marker[i].lifetime = ros::Duration();  // 设置Marker的生命周期为无限
    }

    // 发布器：发布地形角度调试信息
    pub_terrain_angle = nh.advertise<std_msgs::Float64>("a1_debug/terrain_angle", 100);
}
'''
用来根据机器人的当前状态（包括运动模式、步态计数器、机器人位置等）
更新每条腿的目标足底位置。具体包括在静止和行走模式下的不同处理方式，plan_contact.
以及使用Raibert启发式算法计算每条腿的足底位置(P=P_feed+P_back [(T_st*v/2)+(K_v*(v-v_d)))
'''
void A1RobotControl::update_plan(A1CtrlStates &state, double dt) {
    state.counter += 1;  // 更新计数器，用于跟踪循环次数

    if (!state.movement_mode) {
        // movement_mode == 0，表示机器人静止，所有脚都在地面上接触
        for (bool &plan_contact: state.plan_contacts) plan_contact = true;  // 设置所有腿的接触标志为真
        state.gait_counter_reset();  // 重置步态计数器
    } else {
        // movement_mode == 1，表示机器人处于行走模式
        for (int i = 0; i < NUM_LEG; ++i) {
            // 更新每条腿的步态计数器，步态计数器是控制每条腿的摆动周期的
            state.gait_counter(i) = state.gait_counter(i) + state.gait_counter_speed(i);
            state.gait_counter(i) = std::fmod(state.gait_counter(i), state.counter_per_gait);  // 保证计数器在一个周期内循环

            // 如果步态计数器小于等于单个摆动周期内的计数值，则设定该腿为接触地面
            if (state.gait_counter(i) <= state.counter_per_swing) {
                state.plan_contacts[i] = true;  // 该腿与地面接触
            } else {
                state.plan_contacts[i] = false;  // 该腿离地
            }
        }
    }
    // 更新足部计划: state.foot_pos_target_world
    Eigen::Vector3d lin_vel_world = state.root_lin_vel;  // 获取世界坐标系下的线速度
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * lin_vel_world;  // 计算机器人身体坐标系下的线速度

    // 使用Raibert启发式算法计算每条腿的足底位置
    state.foot_pos_target_rel = state.default_foot_pos;  // 设定腿部目标位置为默认足底位置
    for (int i = 0; i < NUM_LEG; ++i) {
        // 计算基于当前线速度与预期的速度差异的X方向和Y方向的偏移量  P=P_feed+P_back [(T_st*v/2)+(K_v*(v-v_d)).
        double delta_x =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(0);  //sqrt(std::abs(state.default_foot_pos(2)) / 9.8):与足底与地面的垂直距离即摆动轨迹相关的系数
        double delta_y =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(1);

        // 限制偏移量在合理范围内
        if (delta_x < -FOOT_DELTA_X_LIMIT) {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT) {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT) {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT) {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        // 更新每条腿的目标相对位置
        state.foot_pos_target_rel(0, i) += delta_x;
        state.foot_pos_target_rel(1, i) += delta_y;

        // 将相对位置转换为绝对位置，并更新目标位置
        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
    }
}

'''
1.初始化变量：初始化了与足部位置、速度、误差和力相关的矩阵变量，确保在后续的计算中使用的是零初值。
2.计算每条腿的足部位置：根据步态计数器判断是摆动脚还是支撑脚，并根据不同情况计算相应的目标位置。
3.贝塞尔曲线插值：使用贝塞尔曲线来生成摆动脚的目标位置，确保足部运动平滑过渡。
4.计算足部位置和速度误差：通过计算目标位置和当前足部位置的差异，得到了位置误差，通过计算目标速度和当前速度的差异，得到了速度误差。
5.动力学计算：通过比例控制和导数控制结合的位置误差和速度误差，计算了足部的受力。
6.提前接触检测：根据步态计数器和足部受力，检测是否发生提前接触。若发生提前接触，则认为该足部与地面接触。
7.平滑接触位置：使用滤波器平滑接触位置，减少传感器噪声影响。
8.更新状态：更新了机器人状态中的足部位置、接触状态以及足部受力信息。
'''
void A1RobotControl::generate_swing_legs_ctrl(A1CtrlStates &state, double dt) {
    state.joint_torques.setZero();  // 初始化关节扭矩为零
    // 获取当前的足部位置、目标足部位置及相关变量
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;   // 当前足部位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;   // 当前足部速度
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;     // 每个足部的spline插值时间
    spline_time.setZero();  // 初始化为零
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;   // 目标足部位置
    foot_pos_target.setZero();  // 初始化为零
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;    // 目标足部速度
    foot_vel_target.setZero();  // 初始化为零
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;    // 足部位置误差
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;    // 足部速度误差
    // 当前足部受力的初始化
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;   // 动力学计算的足部受力
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;    // 地面反作用力（GRF）
    // 遍历每条腿，进行足部位置和速度的计算
    for (int i = 0; i < NUM_LEG; ++i) {
        // 将足部的绝对位置转换到身体坐标系下
        foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i);

        // 根据步态计数器判断足部是摆动（swing）还是支撑（stance）
        if (state.gait_counter(i) <= state.counter_per_swing) {
            spline_time(i) = 0.0;  // 支撑脚时，spline时间为0
            // 保持支撑脚位置
            state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        } else {
            spline_time(i) = float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);  // 摆动脚，计算spline插值时间
        }

        // 使用Bezier曲线生成目标足部位置
        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                              state.foot_pos_start.block<3, 1>(0, i),
                                                                              state.foot_pos_target_rel.block<3, 1>(0, i),
                                                                              0.0);

        // 计算当前足部速度
        foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        // 计算目标足部速度
        foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        // 计算足部位置和速度误差
        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);

        // 计算足部受力：PD控制
        foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                            foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
    }

    // 更新当前足部位置
    state.foot_pos_cur = foot_pos_cur;

    // 检测提前接触
    bool last_contacts[NUM_LEG];

    for (int i = 0; i < NUM_LEG; ++i) {
        if (state.gait_counter(i) <= state.counter_per_swing * 1.5) {
            state.early_contacts[i] = false;  // 摆动脚未接触地面
        }

        // 如果足部计划接触，但足部受力超过阈值，判定为提前接触
        if (!state.plan_contacts[i] &&
            (state.gait_counter(i) > state.counter_per_swing * 1.5) &&
            (state.foot_force(i) > FOOT_FORCE_LOW)) {
            state.early_contacts[i] = true;
        }
        // 实际接触状态
        last_contacts[i] = state.contacts[i];
        state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];
        // 记录接触位置（使用滤波器进行平滑处理）
        if (state.contacts[i]) {
            state.foot_pos_recent_contact.block<3, 1>(0, i)
                    << recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0, i)),
                    recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1, i)),
                    recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2, i));
        }
    }
    // 输出足部接触的z坐标
    std::cout << "foot_pos_recent_contact z: " << state.foot_pos_recent_contact.block<1, 4>(2, 0) << std::endl;

    // 更新足部动力学计算的受力
    state.foot_forces_kin = foot_forces_kin;
}

'''
根据机器人当前每个腿部的状态（摆动腿或支撑腿），以及受力情况（如步态力矩或重力补偿），动态地计算并更新各个关节的控制力矩(jac * F = tau)
'''
void A1RobotControl::compute_joint_torques(A1CtrlStates &state) {
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;  // 定义关节力矩向量，大小为机器人的自由度数目（NUM_DOF）
    joint_torques.setZero();  // 初始化关节力矩为零

    mpc_init_counter++;  // 更新模型预测控制器的初始化计数器

    // 对于前10个周期，直接返回零力矩
    if (mpc_init_counter < 10) {
        state.joint_torques = joint_torques;
    } else {
        // 计算每个腿的关节力矩
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);  // 提取当前腿部的雅可比矩阵

            if (state.contacts[i]) {  // 如果是支撑腿
                // 使用支撑腿的地面反作用力（foot_forces_grf）来计算关节力矩
                joint_torques.segment<3>(i * 3) = jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i);
            } else {  // 如果是摆动腿
                // 使用摆动腿的动力学反馈（foot_forces_kin）来计算关节力矩
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);  // 求解逆运动学方程：jac * F = tau
            }
        }

        // 添加重力补偿力矩
        joint_torques += state.torques_gravity;

        // 防止关节力矩出现nan（非数值）值
        for (int i = 0; i < 12; ++i) {
            if (!isnan(joint_torques[i]))  // 如果该关节力矩不是nan，则更新
                state.joint_torques[i] = joint_torques[i];
        }
    }
}


Eigen::Matrix<double, 3, NUM_LEG> A1RobotControl::compute_grf(A1CtrlStates &state, double dt) {
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // first get parameters needed to construct the solver hessian and gradient
    // use euler angle to get desired angle
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // limit euler error to pi/2
    if (euler_error(2) > 3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) - 3.1415926 * 2 - state.root_euler(2);
    } else if (euler_error(2) < -3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) + 3.1415926 * 2 - state.root_euler(2);
    }

    // only do terrain adaptation in MPC
    if (state.stance_leg_control_type == 1) {
        Eigen::Vector3d surf_coef = compute_walking_surface(state);
        Eigen::Vector3d flat_ground_coef;
        flat_ground_coef << 0, 0, 1;
        double terrain_angle = 0;
        // only record terrain angle when the body is high,计算表面法线与平面法线之间的夹角，以便机器人根据地面倾斜进行姿态调整
        if (state.root_pos[2] > 0.1) {
            terrain_angle = terrain_angle_filter.CalculateAverage(Utils::cal_dihedral_angle(flat_ground_coef, surf_coef));
        } else {
            terrain_angle = 0;
        }

        if (terrain_angle > 0.5) {
            terrain_angle = 0.5;
        }
        if (terrain_angle < -0.5) {
            terrain_angle = -0.5;
        }

        // FL, FR, RL, RR。F_R_diff 计算的是最近四个脚在 Z 轴方向上的位置差异。这个差异用来判断机器人是否处于倾斜的地形上，如果大于某个阈值，说明地面存在较大倾斜。
        double F_R_diff = state.foot_pos_recent_contact(2, 0) + state.foot_pos_recent_contact(2, 1) - state.foot_pos_recent_contact(2, 2) -
                        state.foot_pos_recent_contact(2, 3);

        if (state.use_terrain_adapt) {//如果启用了地形适应 (use_terrain_adapt)，则根据 F_R_diff 调整期望的俯仰角度 (root_euler_d[1])，从而使机器人能够适应不平的地面。
            if (F_R_diff > 0.05) {
            state.root_euler_d[1] = -terrain_angle;
            } else {
            state.root_euler_d[1] = terrain_angle;
            }
        }

//将计算出的地形角度转换为角度单位并通过 ROS 发布出去，便于调试和查看机器人的地形适应效果。
        std_msgs::Float64 terrain_angle_msg;
        terrain_angle_msg.data = terrain_angle * (180 / 3.1415926);
        pub_terrain_angle.publish(terrain_angle_msg); // publish in deg
        std::cout << "desire pitch in deg: " << state.root_euler_d[1] * (180 / 3.1415926) << std::endl;
        std::cout << "terrain angle: " << terrain_angle << std::endl;

        // save calculated terrain pitch angle
        // TODO: limit terrain pitch angle to -30 to 30? 
        state.terrain_pitch_angle = terrain_angle;
    }
    //QP求解
    if (state.stance_leg_control_type == 0) { // 0: QP
        // desired acc in world frame
        root_acc.setZero();
        //PD线速度控制
        root_acc.block<3, 1>(0, 0) = state.kp_linear.cwiseProduct(state.root_pos_d - state.root_pos);
        root_acc.block<3, 1>(0, 0) += state.root_rot_mat * state.kd_linear.cwiseProduct(
                state.root_lin_vel_d - state.root_rot_mat.transpose() * state.root_lin_vel);
        //PD角速度控制
        root_acc.block<3, 1>(3, 0) = state.kp_angular.cwiseProduct(euler_error);
        root_acc.block<3, 1>(3, 0) += state.kd_angular.cwiseProduct(
                state.root_ang_vel_d - state.root_rot_mat.transpose() * state.root_ang_vel);

        // add gravity
        root_acc(2) += state.robot_mass * 9.8;

        // 创建了一个逆惯性矩阵，inertia_inv，用于计算腿部的地面反作用力
        Eigen::Matrix<double, 6, DIM_GRF> inertia_inv;
        for (int i = 0; i < NUM_LEG; ++i) {
            inertia_inv.block<3, 3>(0, i * 3).setIdentity();
            // TODO: confirm this should be root_rot_mat instead of root_rot_mat
            inertia_inv.block<3, 3>(3, i * 3) = state.root_rot_mat_z.transpose() * Utils::skew(state.foot_pos_abs.block<3, 1>(0, i));
        }
'''
R 和 Q 是调节代价函数的权重矩阵，R 通常用于表示力的代价（对力的变化敏感），Q 用于表示加速度的代价（对加速度的敏感度）。
hessian 是QP优化问题中的Hessian矩阵，它决定了力的优化过程中加速度和力之间的关系。
'''        
        Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian;
        dense_hessian.setIdentity();
        dense_hessian *= R;
        dense_hessian += inertia_inv.transpose() * Q * inertia_inv;
        hessian = dense_hessian.sparseView();
        // accidentally wrote this as -2* before. Huge problem
        gradient.block<3 * NUM_LEG, 1>(0, 0) = -inertia_inv.transpose() * Q * root_acc;

        // 根据每个脚的接触状态，设置相应的力的上下界，确保地面反作用力在允许范围内。
        for (int i = 0; i < NUM_LEG; ++i) {
            double c_flag = state.contacts[i] ? 1.0 : 0.0;
            lowerBound(i) = c_flag * F_min;
            upperBound(i) = c_flag * F_max;
        }
'''
创建一个OSQP求解器（OsqpEigen::Solver），这是一个基于梯度投影法的二次规划求解器，用于求解稀疏QP问题。
设置变量个数（3个力分量 * 每条腿），以及约束条件（地面反作用力的最小和最大约束）。
'''

        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.data()->setNumberOfVariables(3 * NUM_LEG);
        solver.data()->setNumberOfConstraints(NUM_LEG + 4 * NUM_LEG);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        auto t1 = std::chrono::high_resolution_clock::now();
        solver.initSolver();
        auto t2 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t3 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

        std::cout << "qp solver init time: " << ms_double_1.count() << "ms; solve time: " << ms_double_2.count() << "ms" << std::endl;

        Eigen::VectorXd QPSolution = solver.getSolution(); //12x1
        for (int i = 0; i < NUM_LEG; ++i) {
            // the QP solves for world frame force
            // here we convert the force into robot frame
            foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * QPSolution.segment<3>(i * 3);
        }

    } else if (state.stance_leg_control_type == 1) { // 1: MPC
        ConvexMpc mpc_solver = ConvexMpc(state.q_weights, state.r_weights);
        mpc_solver.reset();

        // initialize the mpc state at the first time step
        // state.mpc_states.resize(13);
        state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
                state.root_pos[0], state.root_pos[1], state.root_pos[2],
                state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
                state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
                -9.8;

        // previously we use dt passed by outer thread. It turns out that this dt is not stable on hardware.
        // if the thread is slowed down, dt becomes large, then MPC will output very large force and torque value
        // which will cause over current. Here we use a new mpc_dt, this should be roughly close to the average dt
        // of thread 1 
        double mpc_dt = 0.0025;

        // in simulation, use dt has no problem
        if (use_sim_time == "true") {
            mpc_dt = dt;
        }
'''
对于MPC的未来一段时间（设定为 PLAN_HORIZON）内的每个时间步，计算出期望的轨迹 state.mpc_states_d。这包括期望的欧拉角、位置、速度等
'''
        state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;
        // state.mpc_states_d.resize(13 * PLAN_HORIZON);
        for (int i = 0; i < PLAN_HORIZON; ++i) {
            state.mpc_states_d.segment(i * 13, 13)
                    <<
                    state.root_euler_d[0],
                    state.root_euler_d[1],
                    state.root_euler[2] + state.root_ang_vel_d[2] * mpc_dt * (i + 1),
                    state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
                    state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1),
                    state.root_pos_d[2],
                    state.root_ang_vel_d[0],
                    state.root_ang_vel_d[1],
                    state.root_ang_vel_d[2],
                    state.root_lin_vel_d_world[0],
                    state.root_lin_vel_d_world[1],
                    0,
                    -9.8;
        }
'''
计算系统矩阵A、B
'''
        // a single A_c is computed for the entire reference trajectory
        auto t1 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_A_mat_c(state.root_euler);

        // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
        // from the reference trajectory and foot placement controller
        // state.foot_pos_abs_mpc = state.foot_pos_abs;
        auto t2 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < PLAN_HORIZON; i++) {
            // calculate current B_c matrix
            mpc_solver.calculate_B_mat_c(state.robot_mass,
                                         state.a1_trunk_inertia,
                                         state.root_rot_mat,
                                         state.foot_pos_abs);
            // state.foot_pos_abs_mpc.block<3, 1>(0, 0) = state.foot_pos_abs_mpc.block<3, 1>(0, 0) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 1) = state.foot_pos_abs_mpc.block<3, 1>(0, 1) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 2) = state.foot_pos_abs_mpc.block<3, 1>(0, 2) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 3) = state.foot_pos_abs_mpc.block<3, 1>(0, 3) - state.root_lin_vel_d * mpc_dt;

            // state space discretization, calculate A_d and current B_d
            mpc_solver.state_space_discretization(mpc_dt);

            // store current B_d matrix
            mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
        }

        // calculate QP matrices
        auto t3 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_qp_mats(state);

        // solve
        auto t4 = std::chrono::high_resolution_clock::now();
        if (!solver.isInitialized()) {
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
            solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver.hessian);
            solver.data()->setGradient(mpc_solver.gradient);
            solver.data()->setLowerBound(mpc_solver.lb);
            solver.data()->setUpperBound(mpc_solver.ub);
            solver.initSolver();
        } else {
            solver.updateHessianMatrix(mpc_solver.hessian);
            solver.updateGradient(mpc_solver.gradient);
            solver.updateLowerBound(mpc_solver.lb);
            solver.updateUpperBound(mpc_solver.ub);
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t6 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
        std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
        std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
        std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

//        std::cout << "mpc cal A_mat_c: " << ms_double_1.count() << "ms" << std::endl;
//        std::cout << "mpc cal B_mat_d_list: " << ms_double_2.count() << "ms" << std::endl;
//        std::cout << "mpc cal qp mats: " << ms_double_3.count() << "ms" << std::endl;
//        std::cout << "mpc init time: " << ms_double_4.count() << "ms" << std::endl;
//        std::cout << "mpc solve time: " << ms_double_5.count() << "ms" << std::endl << std::endl;

        Eigen::VectorXd solution = solver.getSolution();
        // std::cout << solution.transpose() << std::endl;

        for (int i = 0; i < NUM_LEG; ++i) {
            if (!isnan(solution.segment<3>(i * 3).norm()))
                foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * solution.segment<3>(i * 3);
        }
    }
    return foot_forces_grf;
}
'''
目的是计算一个步态控制中涉及的“行走表面”模型的系数。
通过处理机器人的足部接触位置数据，推导出一个与地面接触的平面模型的系数。
这个平面通常用于表示机器人的脚和地面之间的接触状态
'''
Eigen::Vector3d A1RobotControl::compute_walking_surface(A1CtrlStates &state) {
    Eigen::Matrix<double, NUM_LEG, 3> W;//机器人的每只脚的位置信息
    Eigen::VectorXd foot_pos_z;//每只脚在 z 轴（即高度方向）上的位置
    Eigen::Vector3d a;//拟合出来的平面系数
    Eigen::Vector3d surf_coef;//最终的表面系数

    W.block<NUM_LEG, 1>(0, 0).setOnes();//在 W 的第一列设置为全 1
    W.block<NUM_LEG, 2>(0, 1) = state.foot_pos_recent_contact.block<2, NUM_LEG>(0, 0).transpose();//将每只脚的 x 和 y 坐标提取出来，并存储到 W 的后两列

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1, NUM_LEG>(2, 0).transpose();  //提取每只脚的 z 坐标（即高度）存储到 foot_pos_z 向量
//最小二乘法拟合平面系数
    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}
