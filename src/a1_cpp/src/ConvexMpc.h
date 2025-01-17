// 防止Eigen库使用堆分配，通常这是为了限制大矩阵的内存分配
#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <vector>
#include <chrono>

// 引入所需的Eigen库（线性代数，矩阵运算等）
#include "OsqpEigen/OsqpEigen.h"  // 用于OSQP求解器
#include <Eigen/Dense>  // Eigen库的矩阵与向量运算
#include <Eigen/Eigenvalues>  // 特征值相关运算
#include <unsupported/Eigen/MatrixFunctions>  // Eigen库中的矩阵函数支持

#include "A1CtrlStates.h"  // 控制状态类
#include "A1Params.h"  // 参数类
#include "utils/Utils.h"  // 工具类，用于其他常用函数

// ConvexMpc 类定义，主要用于模型预测控制（MPC）的实现
class ConvexMpc {
public:
    // 构造函数，接收权重矩阵 q_weights_ 和 r_weights_，用于MPC优化
    ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_);

    // 重置函数，重新初始化状态
    void reset();

    // 计算状态空间矩阵A_c，用于离散化后的系统动力学
    void calculate_A_mat_c(Eigen::Vector3d root_euler);

    // 计算输入矩阵B_c，描述控制输入对系统的影响
    void calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                           Eigen::Matrix<double, 3, NUM_LEG> foot_pos);

    // 对状态空间模型进行离散化
    void state_space_discretization(double dt);

    // 计算QP问题的矩阵（Hessian矩阵和梯度）
    void calculate_qp_mats(A1CtrlStates &state);

private:
    // MPC中的摩擦系数、最小和最大z轴地面反作用力 （摩擦锥约束）
    double mu;
    double fz_min;
    double fz_max;

    // 权重矩阵q和r，用于MPC的目标函数
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;//步长*维度
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc; //自由度*维度

    // 对应状态和控制输入的权重矩阵（对角矩阵形式）
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;

    // 稀疏矩阵表示，适用于求解大规模优化问题
    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::SparseMatrix<double> R_sparse;

    // 状态空间模型的矩阵（连续时间表示）
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_c_list;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_c;

    // 状态空间模型的矩阵（离散时间表示）
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d_list;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_d;

    // QP优化中的矩阵A和B（F、FAI）
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_qp;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> B_qp;
    Eigen::SparseMatrix<double> A_qp_sparse;
    Eigen::SparseMatrix<double> B_qp_sparse;

    // QP的标准形式：最小化 1/2 * x' * P * x + q' * x
    // 其中P为Hessian矩阵，q为梯度，Ac为约束矩阵
    Eigen::SparseMatrix<double> hessian; // P（Hessian矩阵） 该函数对每个变量的二阶偏导数组成的矩阵
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient; // q（梯度）
    Eigen::SparseMatrix<double> linear_constraints; // Ac（约束矩阵）
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> lb;  // 约束下界
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> ub;  // 约束上界
};

#endif //A1_CPP_CONVEXMPC_H
