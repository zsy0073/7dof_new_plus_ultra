#include "RobotKinematics.h"
#include <cmath>
#include <Arduino.h> // 添加Arduino头文件以支持Serial对象

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

bool RobotKinematics::forwardKinematics(const VectorXd& theta, Matrix4d& T) {
    if (theta.size() != ARM_DOF) {
        return false;
    }
    
    T = Matrix4d::Identity();
    
    for(int i = 0; i < ARM_DOF; i++) {
        Matrix4d A_i = computeTransform(alpha_(i), a_(i), d_(i), theta(i) + theta_offset_(i));
        T = T * A_i;
    }
    
    return true;
}

bool RobotKinematics::inverseKinematics(const Matrix4d& T_des, const VectorXd& q0, 
                                      VectorXd& q_result, double tol, int max_iter) {
    VectorXd q = q0;
    MatrixXd J(6, ARM_DOF);    // 空间雅可比矩阵 6xN
    MatrixXd J_pinv;           // 伪逆矩阵
    VectorXd error(6);         // 位置和姿态误差
    Matrix4d T_current;        // 当前位姿矩阵
    double damping = DAMPING;  // 使用头文件中定义的阻尼因子
    
    // 输出目标位姿信息以便调试
    Vector3d target_pos = T_des.block<3,1>(0,3);
    Matrix3d target_rot = T_des.block<3,3>(0,0);
    Serial.printf("逆运动学: 目标=[%.3f, %.3f, %.3f]\n", 
                  target_pos(0), target_pos(1), target_pos(2));
    
    // 收敛过程的一些统计信息
    double min_error = std::numeric_limits<double>::max();
    VectorXd best_q = q0;
    int stalled_count = 0;  // 用于记录连续误差改善不明显的次数
    
    // 早期终止标志
    bool early_stop = false;
    
    // 尝试多个随机初值
    for (int attempt = 0; attempt < 1; attempt++) {  // 仅对初始点进行计算，外部已有多次尝试机制
        double last_error = std::numeric_limits<double>::max();
        
        for(int iter = 0; iter < max_iter; iter++) {
            // 计算当前位姿
            forwardKinematics(q, T_current);
            
            // 计算误差 - 使用改进的误差计算方式
            computeFullError(T_current, T_des, error);  // 使用更完整的误差度量
            double error_norm = error.norm();
            
            // 记录最佳解
            if(error_norm < min_error) {
                min_error = error_norm;
                best_q = q;
                stalled_count = 0;  // 重置停滞计数
                
                // 输出改进的解
                if(iter > 0 && iter % 50 == 0) {
                    Serial.printf("  迭代%d: 误差=%.6f\n", iter, min_error);
                }
            } else {
                stalled_count++;  // 误差没有改善，增加停滞计数
            }
            
            // 检查收敛性 - 早期终止条件
            if(error_norm < tol) {
                Serial.printf("逆运动学收敛! 迭代次数=%d, 误差=%.6f\n", iter, error_norm);
                q_result = q;
                return true;
            }
            
            // 如果连续15次迭代都没有改善，增加随机扰动 (从30减少到15)
            if (stalled_count > 15) {
                // 尝试更有针对性的扰动 - 主要在难以收敛的方向上添加扰动
                VectorXd perturb = VectorXd::Zero(ARM_DOF);
                
                // 找出贡献最小的关节方向
                MatrixXd J_contrib = J.cwiseAbs();
                VectorXd joint_contrib = J_contrib.colwise().sum();
                int min_contrib_idx = 0;
                double min_contrib = joint_contrib(0);
                for (int i = 1; i < ARM_DOF; i++) {
                    if (joint_contrib(i) < min_contrib) {
                        min_contrib = joint_contrib(i);
                        min_contrib_idx = i;
                    }
                }
                
                // 在贡献小的方向上添加较大扰动，其他方向小扰动
                for (int i = 0; i < ARM_DOF; i++) {
                    double scale = (i == min_contrib_idx) ? 0.1 : 0.03;
                    perturb(i) = (double(rand()) / RAND_MAX - 0.5) * scale;
                }
                
                q += perturb;
                
                // 应用关节限位
                if(has_joint_limits_) {
                    q = q.cwiseMax(joint_lower_limits_).cwiseMin(joint_upper_limits_);
                }
                
                stalled_count = 0;
                
                // 如果进行了过多次扰动尝试，提前结束
                if (iter > max_iter/2 && min_error > 0.1) {
                    early_stop = true;
                    break;
                }
                
                continue;
            }
            
            // 更有效率的雅可比计算策略
            if (iter % 5 == 0) {
                // 每5次迭代使用一次数值雅可比（更准确但计算量大）
                getJacobianNumerical(q, T_des, J);
            } else {
                // 其他时候使用解析雅可比（更快）
                getJacobian(q, J);
            }
            
            // 检查雅可比矩阵有效性
            if(!J.allFinite()) {
                Serial.println("警告: 雅可比矩阵包含无效值");
                q_result = best_q;
                return min_error < 0.1; // 放宽容差
            }
            
            // 计算DLS伪逆并进行额外优化
            // 自适应阻尼 - 在误差大时使用大阻尼，误差小时减小阻尼
            damping = DAMPING * (1.0 + 5.0 * std::min(1.0, error_norm));
            
            // 计算伪逆
            pseudoInverse(J, J_pinv, damping);
            
            // 计算关节角度增量
            VectorXd dq = -J_pinv * error;
            
            // 实现简化的线搜索 - 减少线搜索次数从10减至5
            double alpha = 1.0;  // 初始步长
            bool step_accepted = false;
            Matrix4d T_new;
            VectorXd q_new, error_new;
            double current_error = error_norm;
            
            for (int ls = 0; ls < 5; ls++) {  // 最多5次线搜索尝试
                q_new = q + alpha * dq;
                
                // 应用关节限位
                if(has_joint_limits_) {
                    q_new = q_new.cwiseMax(joint_lower_limits_).cwiseMin(joint_upper_limits_);
                }
                
                // 计算新误差
                forwardKinematics(q_new, T_new);
                error_new.resize(6);
                computeFullError(T_new, T_des, error_new);
                double new_error = error_new.norm();
                
                if (new_error < current_error * 0.99) {  // 只接受显著改进
                    // 接受这一步
                    q = q_new;
                    step_accepted = true;
                    break;
                }
                
                // 减小步长继续尝试
                alpha *= 0.5;
                
                if (alpha < 1e-4) {  // 避免过小的步长
                    break;
                }
            }
            
            // 如果线搜索失败，使用直接缩小的步长
            if (!step_accepted) {
                double fixed_alpha = 0.01;  // 固定小步长
                q_new = q + fixed_alpha * dq;
                
                // 应用关节限位
                if(has_joint_limits_) {
                    q_new = q_new.cwiseMax(joint_lower_limits_).cwiseMin(joint_upper_limits_);
                }
                
                q = q_new;
            }
            
            // 每隔50次检查是否停滞
            if (iter > 0 && iter % 50 == 0) {
                if (fabs(last_error - error_norm) < 0.001) {
                    // 误差几乎不变，跳出循环
                    Serial.printf("  误差停滞, 提前结束: %.6f -> %.6f\n", last_error, error_norm);
                    break;
                }
                last_error = error_norm;
            }
            
            // 新增：如果达到了较好的解但仍未达到容差，提前结束
            if (error_norm < 0.05 && iter > max_iter/4) {
                Serial.printf("  达到较好解，提前结束: 误差=%.6f\n", error_norm);
                break;
            }
        }
        
        if (early_stop) {
            Serial.println("多次扰动后仍难以收敛，提前结束");
            break;
        }
    }
    
    // 所有尝试结束，返回最佳解
    q_result = best_q;
    
    // 输出最终结果
    Serial.printf("逆运动学最终: 误差=%.6f\n", min_error);
                 
    return min_error < 0.1;  // 接受稍大的误差，提高成功率
}

// 添加使用MATLAB类似方式的数值微分雅可比计算
void RobotKinematics::getJacobianNumerical(const VectorXd& q, const Matrix4d& T_des, MatrixXd& J) {
    const double delta = 1e-6;  // 数值微分步长
    J.resize(6, ARM_DOF);
    
    // 计算当前误差
    Matrix4d T_current;
    forwardKinematics(q, T_current);
    VectorXd error_current(6);
    computeFullError(T_current, T_des, error_current);
    
    // 对每个关节进行扰动，计算雅可比列
    for (int i = 0; i < ARM_DOF; i++) {
        // 扰动当前关节角
        VectorXd q_perturbed = q;
        q_perturbed(i) += delta;
        
        // 计算扰动后的误差
        Matrix4d T_perturbed;
        forwardKinematics(q_perturbed, T_perturbed);
        VectorXd error_perturbed(6);
        computeFullError(T_perturbed, T_des, error_perturbed);
        
        // 计算雅可比列
        J.col(i) = (error_perturbed - error_current) / delta;
    }
}

// 计算完整误差 (位置 + 完整方向差异)
void RobotKinematics::computeFullError(const Matrix4d& T_current, const Matrix4d& T_des, VectorXd& error) {
    error.resize(6);
    
    // 位置误差
    error.head(3) = T_current.block<3,1>(0,3) - T_des.block<3,1>(0,3);
    
    // 姿态误差 (使用对数映射)
    Matrix3d R_current = T_current.block<3,3>(0,0);
    Matrix3d R_des = T_des.block<3,3>(0,0);
    
    // 计算旋转误差矩阵
    Matrix3d R_error = R_current.transpose() * R_des;
    
    // 提取轴角表示
    double cos_theta = (R_error.trace() - 1.0) / 2.0;
    cos_theta = std::min(std::max(cos_theta, -1.0), 1.0);  // 确保在[-1,1]范围内
    
    double theta = acos(cos_theta);
    
    if (theta < 1e-10) {
        // 如果角度几乎为零，方向轴任意
        error.tail(3).setZero();
    } else {
        // 计算旋转轴
        Vector3d axis;
        axis(0) = R_error(2,1) - R_error(1,2);
        axis(1) = R_error(0,2) - R_error(2,0);
        axis(2) = R_error(1,0) - R_error(0,1);
        
        if (axis.norm() < 1e-10) {
            // 处理奇异情况 (当theta接近pi时)
            // 尝试直接从旋转矩阵对角线元素提取轴
            Vector3d diag(R_error(0,0), R_error(1,1), R_error(2,2));
            int max_idx = 0;
            for (int i = 1; i < 3; i++) {
                if (diag(i) > diag(max_idx)) max_idx = i;
            }
            
            axis.setZero();
            axis(max_idx) = 1.0;
            
            // 应用符号校正
            double sign = (max_idx == 1) ? -1.0 : 1.0;
            error.tail(3) = sign * M_PI * axis;
        } else {
            // 正常情况
            axis.normalize();
            error.tail(3) = theta * axis;
        }
    }
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
    try {
        // 检查输入矩阵是否有效
        if (!J.allFinite()) {
            Serial.println("警告: 雅可比矩阵包含无效值，无法计算伪逆");
            throw std::runtime_error("雅可比矩阵包含无效值");
        }
        
        // 获取矩阵的行和列数
        int rows = J.rows();
        int cols = J.cols();
        
        // 检查矩阵大小以避免内存问题
        if (rows == 0 || cols == 0) {
            throw std::runtime_error("雅可比矩阵大小为零");
        }

        // 安全的伪逆计算方法，避免SVD可能引起的问题
        // 使用更稳定的方法直接计算 (J^T * J + lambda * I)^-1 * J^T 或 J^T * (J * J^T + lambda * I)^-1
        J_pinv.resize(cols, rows); // 确保输出矩阵尺寸正确

        if (rows >= cols) {
            // 如果J是"胖"矩阵或方阵: (J^T·J + lambda·I)^(-1)·J^T
            // 计算 J^T·J
            MatrixXd JTJ = J.transpose() * J;
            
            // 添加正则化项，提高数值稳定性
            for (int i = 0; i < JTJ.rows(); ++i) {
                JTJ(i, i) += lambda;
            }
            
            // 确保JTJ可逆
            bool invertible = true;
            for (int i = 0; i < JTJ.rows(); ++i) {
                if (abs(JTJ(i, i)) < 1e-8) {
                    invertible = false;
                    break;
                }
            }
            
            if (invertible) {
                try {
                    // 计算 (J^T·J + lambda·I)^(-1)·J^T
                    MatrixXd JTJ_inv = JTJ.inverse();
                    J_pinv = JTJ_inv * J.transpose();
                } catch (...) {
                    Serial.println("矩阵求逆失败，使用替代方法");
                    J_pinv = MatrixXd::Zero(cols, rows);
                }
            } else {
                Serial.println("矩阵接近奇异，使用零矩阵");
                J_pinv = MatrixXd::Zero(cols, rows);
            }
        } else {
            // 如果J是"瘦"矩阵: J^T·(J·J^T + lambda·I)^(-1)
            // 计算 J·J^T
            MatrixXd JJT = J * J.transpose();
            
            // 添加正则化项，提高数值稳定性
            for (int i = 0; i < JJT.rows(); ++i) {
                JJT(i, i) += lambda;
            }
            
            // 确保JJT可逆
            bool invertible = true;
            for (int i = 0; i < JJT.rows(); ++i) {
                if (abs(JJT(i, i)) < 1e-8) {
                    invertible = false;
                    break;
                }
            }
            
            if (invertible) {
                try {
                    // 计算 J^T·(J·J^T + lambda·I)^(-1)
                    MatrixXd JJT_inv = JJT.inverse();
                    J_pinv = J.transpose() * JJT_inv;
                } catch (...) {
                    Serial.println("矩阵求逆失败，使用替代方法");
                    J_pinv = MatrixXd::Zero(cols, rows);
                }
            } else {
                Serial.println("矩阵接近奇异，使用零矩阵");
                J_pinv = MatrixXd::Zero(cols, rows);
            }
        }
        
        // 验证计算结果的有效性
        if (!J_pinv.allFinite()) {
            throw std::runtime_error("计算得到的伪逆包含无效值");
        }
    }
    catch (const std::exception& e) {
        Serial.print("伪逆计算异常: ");
        Serial.println(e.what());
        
        // 出错时返回零矩阵
        J_pinv = MatrixXd::Zero(J.cols(), J.rows());
    }
    catch (...) {
        Serial.println("伪逆计算未知异常，使用零矩阵");
        J_pinv = MatrixXd::Zero(J.cols(), J.rows());
    }
}

// 欧拉角转旋转矩阵 (RPY: Roll-Pitch-Yaw) 实现
Matrix3d RobotKinematics::eulerToRotation(const Vector3d& euler) const {
    // 提取欧拉角 (RPY)
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);
    
    // 计算三角函数值
    double sr = sin(roll), cr = cos(roll);
    double sp = sin(pitch), cp = cos(pitch);
    double sy = sin(yaw), cy = cos(yaw);
    
    // 构建旋转矩阵
    Matrix3d R = Matrix3d::Zero();
    
    R(0,0) = cy*cp;    R(0,1) = cy*sp*sr-sy*cr;    R(0,2) = cy*sp*cr+sy*sr;
    R(1,0) = sy*cp;    R(1,1) = sy*sp*sr+cy*cr;    R(1,2) = sy*sp*cr-cy*sr;
    R(2,0) = -sp;      R(2,1) = cp*sr;             R(2,2) = cp*cr;
    
    return R;
}