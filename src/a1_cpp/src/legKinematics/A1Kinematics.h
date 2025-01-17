#ifndef VILEOM_A1KINEMATICS_H
#define VILEOM_A1KINEMATICS_H

#include <eigen3/Eigen/Dense>  // 引入Eigen库，用于矩阵和向量计算

class A1Kinematics {

    public:
        A1Kinematics() = default;  // 默认构造函数
        ~A1Kinematics() = default; // 默认析构函数

        // 以下是常量定义，表示优化和固定参数的大小
        const int RHO_OPT_SIZE = 3;   // rho_opt 表示与接触偏移量相关的参数，包含 cx, cy, cz，大小为3
        const int RHO_FIX_SIZE = 5;   // rho_fix 表示身体偏移量、股部偏移量、大腿长度和小腿长度，大小为5

        // 以下是接口函数声明，接收Eigen库的矩阵和向量进行计算：
        
        // 正向运动学函数：计算机器人的末端执行器位置（3x1）
        Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
        
        // 雅可比矩阵计算：计算末端执行器相对于机器人关节速度的映射（3x3）
        Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
        
        // 正向运动学的偏导数：计算正向运动学对 rho_opt 的偏导数（3x3）
        Eigen::Matrix3d dfk_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
        
        // 雅可比矩阵对关节角度 q 的偏导数（9x3）
        Eigen::Matrix<double, 9, 3> dJ_dq(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
        
        // 雅可比矩阵对 rho_opt 的偏导数（9x3）
        Eigen::Matrix<double, 9, 3> dJ_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    private:
        // 以下是使用基本C++接口，基于Matlab自动生成的函数，用于实现上述功能：
        
        // autoFunc_fk_derive：计算正向运动学的推导（3x1），从关节角度 q 和参数得到末端执行器位置
        void autoFunc_fk_derive(const double in1[3], const double in2[3], const double in3[5], double p_bf[3]);
        
        // autoFunc_d_fk_dq：计算正向运动学对关节角度的偏导数（9x3）
        void autoFunc_d_fk_dq(const double in1[3], const double in2[3], const double in3[5], double jacobian[9]);
        
        // autoFunc_d_fk_dc：计算正向运动学对接触偏移量的偏导数（9x3）
        void autoFunc_d_fk_dc(const double in1[3], const double in2[3], const double in3[5], double d_fk_dc[9]);
        
        // autoFunc_dJ_dq：计算雅可比矩阵对关节角度的偏导数（27x3）
        void autoFunc_dJ_dq(const double in1[3], const double in2[3], const double in3[5], double dJ_dq[27]);
        
        // autoFunc_dJ_dpho：计算雅可比矩阵对 rho_opt 的偏导数（27x3）
        void autoFunc_dJ_dpho(const double in1[3], const double [3], const double [5], double dJ_dpho[27]);
};

#endif //VILEOM_A1KINEMATICS_H
