#include "RobotKinematics.h"
#include <cmath>

RobotKinematics::RobotKinematics() {
    // 初始化向量
    alpha_ = VectorXd::Zero(ARM_DOF);
    a_ = VectorXd::Zero(ARM_DOF);
    d_ = VectorXd::Zero(ARM_DOF);
    theta_offset_ = VectorXd::Zero(ARM_DOF);
    
    has_joint_limits_ = false;
    joint_lower_limits_ = VectorXd::Constant(ARM_DOF, -M_PI);
    joint_upper_limits_ = VectorXd::Constant(ARM_DOF, M_PI);

    // 设置默认的MDH参数
    // 连杆扭角 alpha
    alpha_(0) = 0.0;
    alpha_(1) = -M_PI/2;
    alpha_(2) = M_PI/2;
    alpha_(3) = -M_PI/2;
    alpha_(4) = M_PI/2;
    alpha_(5) = -M_PI/2;
    alpha_(6) = M_PI/2;

    // 连杆长度 a 全部为0
    for(int i = 0; i < ARM_DOF; i++) {
        a_(i) = 0.0;
    }

    // 连杆偏距 d
    d_(0) = 0.1875;
    d_(1) = 0.0;
    d_(2) = 0.162;
    d_(3) = 0.0;
    d_(4) = 0.162;
    d_(5) = 0.0;
    d_(6) = 0.138;
}

void RobotKinematics::setMDHParams(const double* alpha, const double* a, const double* d) {
    for(int i = 0; i < ARM_DOF; i++) {
        alpha_(i) = alpha[i];
        a_(i) = a[i];
        d_(i) = d[i];
    }
}

void RobotKinematics::setJointLimits(const VectorXd& lower_limits, const VectorXd& upper_limits) {
    joint_lower_limits_ = lower_limits;
    joint_upper_limits_ = upper_limits;
    has_joint_limits_ = true;
}

bool RobotKinematics::checkJointLimits(const VectorXd& q) {
    if (!has_joint_limits_) return true;
    
    return ((q.array() >= joint_lower_limits_.array()) && 
            (q.array() <= joint_upper_limits_.array())).all();
}

void RobotKinematics::forwardKinematics(const VectorXd& theta, Matrix4d& T) {
    T = Matrix4d::Identity();
    
    for(int i = 0; i < ARM_DOF; i++) {
        Matrix4d A_i = computeTransform(alpha_(i), a_(i), d_(i), theta(i) + theta_offset_(i));
        T = T * A_i;
    }
}

bool RobotKinematics::inverseKinematics(const Matrix4d& T_des, const VectorXd& q0, 
                                      VectorXd& q_result, double tol, int max_iter) {
    VectorXd q = q0;
    MatrixXd J(6, ARM_DOF);    // 空间雅可比矩阵 6xN
    MatrixXd J_pinv;           // 伪逆矩阵
    VectorXd error(6);         // 位置和姿态误差
    Matrix4d T_current;        // 当前位姿矩阵
    double damping = 1e-6;     // 阻尼因子
    
    for(int iter = 0; iter < max_iter; iter++) {
        // 计算当前位姿
        forwardKinematics(q, T_current);
        
        // 计算误差
        computeError(T_current, T_des, error);
        
        // 检查收敛性
        if(error.norm() < tol) {
            q_result = q;
            return true;
        }
        
        // 计算雅可比矩阵
        getJacobian(q, J);
        
        // 在inverseKinematics中添加奇异值检查
        double singularity_threshold = 1e-6;
        JacobiSVD<MatrixXd> svd(J);
        if (svd.singularValues()(svd.singularValues().size()-1) < singularity_threshold) {
            // 处理奇异情况
            damping = damping * 1e2;  // 动态调整阻尼因子
        }
        
        // 计算伪逆（带阻尼项）
        pseudoInverse(J, J_pinv, damping);
        
        // 计算关节角度增量
        VectorXd dq = -J_pinv * error;
        
        // 更新关节角度
        q += dq;
        
        // 应用关节限位
        if(has_joint_limits_) {
            q = q.cwiseMax(joint_lower_limits_).cwiseMin(joint_upper_limits_);
        }
    }
    
    return false;  // 未收敛
}

void RobotKinematics::getJacobian(const VectorXd& theta, MatrixXd& J) {
    J.resize(6, ARM_DOF);
    Matrix4d T_current;
    forwardKinematics(theta, T_current);
    
    Matrix4d T_i = Matrix4d::Identity();
    Vector3d z_i_1, p_i_1, p_e;
    p_e = T_current.block<3,1>(0,3);
    
    for(int i = 0; i < ARM_DOF; i++) {
        // 计算第i-1个坐标系的z轴方向和原点位置
        z_i_1 = T_i.block<3,1>(0,2);
        p_i_1 = T_i.block<3,1>(0,3);
        
        // 计算线速度雅可比列
        J.block<3,1>(0,i) = z_i_1.cross(p_e - p_i_1);
        // 计算角速度雅可比列
        J.block<3,1>(3,i) = z_i_1;
        
        // 更新到下一个连杆的变换矩阵
        T_i = T_i * computeTransform(alpha_(i), a_(i), d_(i), theta(i));
    }
}

Matrix4d RobotKinematics::computeTransform(double alpha, double a, double d, double theta) {
    Matrix4d T;
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    
    T << ct,    -st,    0,     a,
         st*ca,  ct*ca, -sa,   -d*sa,
         st*sa,  ct*sa,  ca,    d*ca,
         0,      0,      0,     1;
    
    return T;
}

void RobotKinematics::computeError(const Matrix4d& T_current, const Matrix4d& T_des, 
                                 VectorXd& error) {
    error.resize(6);
    
    // 位置误差
    error.head(3) = T_current.block<3,1>(0,3) - T_des.block<3,1>(0,3);
    
    // 姿态误差（使用旋转矩阵的对数映射）
    Matrix3d R_current = T_current.block<3,3>(0,0);
    Matrix3d R_des = T_des.block<3,3>(0,0);
    Matrix3d R_error = R_current.transpose() * R_des;
    
    // 将旋转矩阵转换为轴角表示
    Vector3d omega;
    double angle = acos((R_error.trace() - 1) / 2);
    if(abs(angle) < 1e-10) {
        omega.setZero();
    } else {
        omega(0) = (R_error(2,1) - R_error(1,2)) / (2*sin(angle));
        omega(1) = (R_error(0,2) - R_error(2,0)) / (2*sin(angle));
        omega(2) = (R_error(1,0) - R_error(0,1)) / (2*sin(angle));
        omega *= angle;
    }
    error.tail(3) = omega;
}

void RobotKinematics::pseudoInverse(const MatrixXd& J, MatrixXd& J_pinv, double lambda) {
    // 使用SVD分解计算伪逆
    JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
    VectorXd S = svd.singularValues();
    
    // 构建奇异值的伪逆（带阻尼项）
    VectorXd S_pinv = S.array() / (S.array().square() + lambda * lambda);
    
    // 计算阻尼伪逆
    J_pinv = svd.matrixV() * S_pinv.asDiagonal() * svd.matrixU().transpose();
}