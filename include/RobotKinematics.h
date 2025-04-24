#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <vector>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/SVD>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM_DOF 7           // 7自由度
#define MATRIX_SIZE 4       // 4x4齐次变换矩阵
#define MAX_ITER 2000       // 最大迭代次数
#define TOL 1e-6           // 收敛阈值
#define DAMPING 0.01       // 阻尼因子

using namespace Eigen;

class RobotKinematics {
public:
    // 构造函数
    RobotKinematics();
    
    // 设置MDH参数
    void setMDHParams(const double* alpha, const double* a, const double* d);
    
    // 正向运动学
    bool forwardKinematics(const VectorXd& theta, Matrix4d& T);
    
    // 逆向运动学 (基于阻尼最小二乘法的数值解法)
    bool inverseKinematics(const Matrix4d& T_des, const VectorXd& q0, VectorXd& q_result,
                          double tol = TOL, int max_iter = MAX_ITER);
    
    // 获取雅可比矩阵
    void getJacobian(const VectorXd& theta, MatrixXd& J);
    
    // 设置关节限位
    void setJointLimits(const VectorXd& lower_limits, const VectorXd& upper_limits);
    
    // 检查关节角度是否在限位内
    bool checkJointLimits(const VectorXd& q);
    
    // 欧拉角转旋转矩阵 (新增)
    Matrix3d eulerToRotation(const Vector3d& euler) const;

private:
    // MDH参数
    VectorXd alpha_;        // 连杆扭角
    VectorXd a_;           // 连杆长度
    VectorXd d_;           // 连杆偏距
    VectorXd theta_offset_; // 关节角度偏置
    
    // 关节限位
    VectorXd joint_lower_limits_;
    VectorXd joint_upper_limits_;
    bool has_joint_limits_;
    
    // 辅助函数
    Matrix4d computeTransform(double alpha, double a, double d, double theta);
    void computeError(const Matrix4d& T_current, const Matrix4d& T_des, VectorXd& error);
    void pseudoInverse(const MatrixXd& J, MatrixXd& J_pinv, double lambda = DAMPING);
};

#endif // ROBOT_KINEMATICS_H