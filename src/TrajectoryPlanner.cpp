#include "TrajectoryPlanner.h"
#include <math.h>
#include <Eigen/Geometry>  // 用于四元数计算
#include <Arduino.h>       // 添加Arduino头文件以支持Serial对象

TrajectoryPlanner::TrajectoryPlanner() {
    sample_time_ = 0.01;      // 默认采样时间10ms
    max_joint_vel_ = 1.0;     // 默认最大关节速度1 rad/s
    max_joint_acc_ = 2.0;     // 默认最大关节加速度2 rad/s^2
    max_points_ = 50;        // 默认最大点数为50个点
}

void TrajectoryPlanner::setTimeParams(double sample_time, double max_joint_vel, double max_joint_acc) {
    sample_time_ = sample_time;
    max_joint_vel_ = max_joint_vel;
    max_joint_acc_ = max_joint_acc;
}

// 设置最大轨迹点数
void TrajectoryPlanner::setMaxPoints(int max_points) {
    if (max_points > 0) {
        max_points_ = max_points;
    }
}

// 获取当前最大点数设置
int TrajectoryPlanner::getMaxPoints() const {
    return max_points_;
}

// 计算适应点数限制的采样时间
double TrajectoryPlanner::calculateSampleTime(double duration, int desired_points) {
    // 基于持续时间和期望点数计算采样时间
    // 确保至少有2个点（起点和终点）
    desired_points = std::max(2, desired_points);
    
    // 计算新的采样时间
    double new_sample_time = duration / (desired_points - 1);
    
    // 确保采样时间不会过小，为安全设置下限
    double min_sample_time = 0.005; // 5ms
    return std::max(min_sample_time, new_sample_time);
}

void TrajectoryPlanner::planJointTrajectory(const VectorXd& q_start, const VectorXd& q_end,
                                          double duration, MatrixXd& trajectory, VectorXd& time_points) {
    // 根据最大点数限制调整采样时间
    double adjusted_sample_time = calculateSampleTime(duration, max_points_);
    double effective_sample_time = std::max(sample_time_, adjusted_sample_time);
    
    // 计算采用调整后采样时间的点数
    int n_points = ceil(duration / effective_sample_time) + 1;
    
    // 确保点数不超过最大限制
    if (n_points > max_points_) {
        n_points = max_points_;
        effective_sample_time = duration / (n_points - 1);
    }
    
    // 只输出关键参数信息
    Serial.printf("[轨迹] 关节轨迹: 时间=%.2f秒, 采样=%.4f秒, 点数=%d\n", 
                  duration, effective_sample_time, n_points);
    
    trajectory.resize(n_points, ARM_DOF);
    time_points.resize(n_points);
    
    // 为每个关节计算五次多项式系数
    MatrixXd coeffs(ARM_DOF, 6);  // 每个关节6个系数
    for(int i = 0; i < ARM_DOF; i++) {
        VectorXd joint_coeffs;
        computeQuinticCoeff(q_start(i), q_end(i), duration, joint_coeffs);
        coeffs.row(i) = joint_coeffs;
    }
    
    // 计算轨迹点
    for(int j = 0; j < n_points; j++) {
        // 均匀分布时间点
        double t = j * duration / (n_points - 1);
        if(t > duration) t = duration;
        time_points(j) = t;
        
        for(int i = 0; i < ARM_DOF; i++) {
            trajectory(j, i) = quinticPosition(t, coeffs.row(i));
        }
        
        // 移除每个点的详细输出，不再输出中间计算过程
    }
    
    Serial.println("[轨迹] 关节轨迹规划完成");
}

void TrajectoryPlanner::planCartesianLine(const Matrix4d& T_start, const Matrix4d& T_end,
                                        double duration, RobotKinematics& kinematics,
                                        MatrixXd& trajectory, VectorXd& time_points) {
    // 根据最大点数限制调整采样时间
    double adjusted_sample_time = calculateSampleTime(duration, max_points_);
    double effective_sample_time = std::max(sample_time_, adjusted_sample_time);
    
    // 计算采用调整后采样时间的点数
    int n_points = ceil(duration / effective_sample_time) + 1;
    
    // 确保点数不超过最大限制
    if (n_points > max_points_) {
        n_points = max_points_;
        effective_sample_time = duration / (n_points - 1);
    }
    
    // 输出调试信息
    Serial.printf("笛卡尔直线轨迹规划: 持续时间=%.2f秒, 采样时间=%.4f秒, 点数=%d\n", 
                  duration, effective_sample_time, n_points);
    
    trajectory.resize(n_points, ARM_DOF);
    time_points.resize(n_points);
    
    // 提取起点和终点的位置和姿态
    Vector3d p_start = T_start.block<3,1>(0,3);
    Vector3d p_end = T_end.block<3,1>(0,3);
    Matrix3d R_start = T_start.block<3,3>(0,0);
    Matrix3d R_end = T_end.block<3,3>(0,0);
    
    // 验证起点位置是否有效，不能为全零位置
    bool valid_start = false;
    for(int i = 0; i < 3; i++) {
        if(abs(p_start(i)) > 0.001) {
            valid_start = true;
            break;
        }
    }
    
    if(!valid_start) {
        Serial.println("错误: 起点位置无效 (全为零)! 使用安全位置替代");
        // 使用一个安全的默认位置作为备选
        p_start << 0.341, 0.000, 0.231;  // 使用终端输出中看到的当前位置
    }
    
    // 输出起点和终点信息以便调试
    Serial.printf("起点位置: [%.3f, %.3f, %.3f]\n", p_start(0), p_start(1), p_start(2));
    Serial.printf("终点位置: [%.3f, %.3f, %.3f]\n", p_end(0), p_end(1), p_end(2));
    
    // 检查路径长度，确保不超过工作空间
    double path_length = (p_end - p_start).norm();
    Serial.printf("路径长度: %.3f 米\n", path_length);
    
    if(path_length > 1.0) {  // 假设最大工作空间半径为1米
        Serial.println("错误: 路径长度超出工作空间限制!");
        throw std::runtime_error("Path length exceeds workspace limits");
    }
    
    // 计算每个时间点的位姿
    bool solution_found = false;
    int max_attempts = 5;  // 最大重试次数
    
    for(int i = 0; i < n_points; i++) {
        // 均匀分布时间点
        double t = i * duration / (n_points - 1);
        if(t > duration) t = duration;
        time_points(i) = t;
        
        // 线性插值位置
        double s = t / duration;
        Vector3d p = p_start + s * (p_end - p_start);
        
        // 球面线性插值旋转
        Matrix3d R;
        interpolateRotation(R_start, R_end, s, R);
        
        // 构建当前位姿矩阵
        Matrix4d T_current = Matrix4d::Identity();
        T_current.block<3,3>(0,0) = R;
        T_current.block<3,1>(0,3) = p;
        
        // 调试信息：当前在计算哪个点
        Serial.printf("计算直线轨迹点: %d/%d, 时间=%.2f秒, 位置=[%.3f, %.3f, %.3f]\n", 
                      i+1, n_points, t, p(0), p(1), p(2));
        
        // 使用多次尝试求解逆运动学，确保找到最优解
        solution_found = false;
        VectorXd q_current;
        VectorXd q_best;
        double min_joint_movement = std::numeric_limits<double>::max();
        
        for(int attempt = 0; attempt < max_attempts; attempt++) {
            Serial.printf("  尝试求解逆运动学 %d/%d\n", attempt+1, max_attempts);
            
            VectorXd q_attempt;
            bool success;
            
            if(i == 0) {
                // 使用随机初始猜测值
                VectorXd q0 = VectorXd::Random(ARM_DOF) * M_PI;
                success = kinematics.inverseKinematics(T_current, q0, q_attempt);
            } else {
                // 使用上一点作为初始猜测值
                success = kinematics.inverseKinematics(T_current, trajectory.row(i-1), q_attempt);
            }
            
            if(success) {
                // 计算关节运动量
                double joint_movement;
                if(i == 0) {
                    joint_movement = q_attempt.norm();  // 相对于零位置
                } else {
                    joint_movement = (q_attempt - trajectory.row(i-1)).norm();
                }
                
                Serial.printf("  逆运动学求解成功，关节运动量=%.3f\n", joint_movement);
                
                // 更新最优解
                if(joint_movement < min_joint_movement) {
                    min_joint_movement = joint_movement;
                    q_best = q_attempt;
                    solution_found = true;
                }
            } else {
                Serial.printf("  逆运动学求解失败\n");
            }
        }
        
        if(!solution_found) {
            Serial.printf("错误: 在时间=%.2f秒处无法找到有效的逆运动学解!\n", t);
            throw std::runtime_error("No valid inverse kinematics solution found at time " + std::to_string(t));
        }
        
        trajectory.row(i) = q_best;
        
        // 检查速度约束
        if(i > 0) {
            VectorXd joint_vel = (trajectory.row(i) - trajectory.row(i-1)) / effective_sample_time;
            double max_vel = joint_vel.cwiseAbs().maxCoeff();
            Serial.printf("  最大关节速度=%.3f rad/s\n", max_vel);
            
            if(max_vel > max_joint_vel_) {
                Serial.printf("错误: 在时间=%.2f秒处关节速度超出限制 (%.3f > %.3f)!\n", 
                             t, max_vel, max_joint_vel_);
                throw std::runtime_error("Joint velocity limit exceeded at time " + std::to_string(t));
            }
        }
        
        Serial.printf("轨迹点 %d/%d 计算完成\n", i+1, n_points);
    }
    
    // 最后检查整个轨迹的动力学约束
    Serial.println("检查整个轨迹的动力学约束...");
    if(!checkDynamicConstraints(trajectory, effective_sample_time)) {
        Serial.println("错误: 轨迹中存在违反动力学约束的情况!");
        throw std::runtime_error("Dynamic constraints violated in the trajectory");
    }
    
    Serial.println("笛卡尔直线轨迹规划完成");
}

void TrajectoryPlanner::planCartesianArc(const Matrix4d& T_start, const Matrix4d& T_end,
                                       const Vector3d& via_point, double duration,
                                       RobotKinematics& kinematics,
                                       MatrixXd& trajectory, VectorXd& time_points) {
    // 根据最大点数限制调整采样时间
    double adjusted_sample_time = calculateSampleTime(duration, max_points_);
    double effective_sample_time = std::max(sample_time_, adjusted_sample_time);
    
    // 计算采用调整后采样时间的点数
    int n_points = ceil(duration / effective_sample_time) + 1;
    
    // 确保点数不超过最大限制
    if (n_points > max_points_) {
        n_points = max_points_;
        effective_sample_time = duration / (n_points - 1);
    }
    
    // 输出调试信息
    Serial.printf("笛卡尔圆弧轨迹规划: 持续时间=%.2f秒, 采样时间=%.4f秒, 点数=%d\n", 
                  duration, effective_sample_time, n_points);
    
    trajectory.resize(n_points, ARM_DOF);
    time_points.resize(n_points);
    
    // 提取起点和终点的位置
    Vector3d p_start = T_start.block<3,1>(0,3);
    Vector3d p_end = T_end.block<3,1>(0,3);
    Matrix3d R_start = T_start.block<3,3>(0,0);
    Matrix3d R_end = T_end.block<3,3>(0,0);
    
    // 计算圆弧中心和半径
    Vector3d v1 = via_point - p_start;
    Vector3d v2 = p_end - p_start;
    Vector3d normal = v1.cross(v2);
    if (normal.norm() < 1e-6) {
        // 三点共线，无法形成圆弧
        throw std::runtime_error("Points are collinear, cannot form an arc");
    }
    normal.normalize();
    
    // 使用三点确定圆
    Matrix3d A;
    A << 2*(p_end-p_start), 2*(via_point-p_start), normal;
    Vector3d b;
    b << (p_end-p_start).squaredNorm(), 
         (via_point-p_start).squaredNorm(),
         0;
    Vector3d x = A.colPivHouseholderQr().solve(b);
    Vector3d center = p_start + x.head<2>()(0) * (p_end-p_start).normalized() + 
                               x.head<2>()(1) * (via_point-p_start).normalized();
    double radius = (p_start - center).norm();
    
    // 计算起点和终点的角度
    Vector3d v_start = p_start - center;
    Vector3d v_end = p_end - center;
    double angle_start = atan2(v_start(1), v_start(0));
    double angle_end = atan2(v_end(1), v_end(0));
    if(angle_end < angle_start) angle_end += 2*M_PI;
    
    // 生成轨迹点
    for(int i = 0; i < n_points; i++) {
        // 均匀分布时间点
        double t = i * duration / (n_points - 1);
        if(t > duration) t = duration;
        time_points(i) = t;
        
        // 计算当前角度
        double s = t / duration;
        double current_angle = angle_start + s * (angle_end - angle_start);
        
        // 计算当前位置
        Vector3d p_current = center + radius * Vector3d(cos(current_angle), 
                                                      sin(current_angle), 
                                                      0);
        
        // 计算当前姿态
        Matrix3d R_current;
        interpolateRotation(R_start, R_end, s, R_current);
        
        // 构建当前位姿矩阵
        Matrix4d T_current = Matrix4d::Identity();
        T_current.block<3,3>(0,0) = R_current;
        T_current.block<3,1>(0,3) = p_current;
        
        // 求解逆运动学
        VectorXd q_current;
        if(i == 0) {
            VectorXd q0 = VectorXd::Zero(ARM_DOF);
            kinematics.inverseKinematics(T_current, q0, q_current);
        } else {
            kinematics.inverseKinematics(T_current, trajectory.row(i-1), q_current);
        }
        
        trajectory.row(i) = q_current;
        
        // 每计算一个点就输出信息
        Serial.printf("计算圆弧轨迹点: %d/%d, 时间=%.2f秒, 角度=%.2f, 位置=[%.3f, %.3f, %.3f]\n", 
                     i+1, n_points, t, current_angle * 180/M_PI, p_current(0), p_current(1), p_current(2));
    }
}

void TrajectoryPlanner::planPickAndPlace(const Matrix4d& T_current, const Matrix4d& pick_pos, const Matrix4d& place_pos,
                                       double lift_height, double move_time,
                                       RobotKinematics& kinematics,
                                       MatrixXd& trajectory, VectorXd& time_points) {
    std::vector<TrajectorySegment> segments;
    
    // 创建提升位置矩阵
    Matrix4d pick_lift = pick_pos;
    pick_lift(2,3) += lift_height;
    Matrix4d place_lift = place_pos;
    place_lift(2,3) += lift_height;
    
    // 定义轨迹段
    TrajectorySegment seg;
    
    // 1. 从当前位置到抓取位置上方
    seg.type = SegmentType::CARTESIAN_LINE;
    seg.duration = move_time;
    seg.start_pose = T_current;  // 使用传入的当前机械臂位姿作为起点
    seg.end_pose = pick_lift;
    segments.push_back(seg);
    
    // 2. 下降到抓取位置
    seg.duration = move_time/2;
    seg.start_pose = pick_lift;
    seg.end_pose = pick_pos;
    segments.push_back(seg);
    
    // 3. 抬升物体
    seg.duration = move_time/2;
    seg.start_pose = pick_pos;
    seg.end_pose = pick_lift;
    segments.push_back(seg);
    
    // 4. 移动到放置位置上方
    seg.duration = move_time;
    seg.start_pose = pick_lift;
    seg.end_pose = place_lift;
    segments.push_back(seg);
    
    // 5. 下降到放置位置
    seg.duration = move_time/2;
    seg.start_pose = place_lift;
    seg.end_pose = place_pos;
    segments.push_back(seg);
    
    // 6. 抬升到放置位置上方
    seg.duration = move_time/2;
    seg.start_pose = place_pos;
    seg.end_pose = place_lift;
    segments.push_back(seg);
    
    // 执行复合轨迹规划
    planCompositeTrajectory(segments, kinematics, trajectory, time_points);
}

void TrajectoryPlanner::computeQuinticCoeff(double q0, double qf, double T, VectorXd& coeffs) {
    coeffs.resize(6);
    
    // 边界条件
    double v0 = 0, vf = 0;  // 初始和终止速度
    double a0 = 0, af = 0;  // 初始和终止加速度
    
    // 计算五次多项式系数
    coeffs(0) = q0;
    coeffs(1) = v0;
    coeffs(2) = a0/2;
    coeffs(3) = (20*qf - 20*q0 - (8*vf + 12*v0)*T - (3*a0 - af)*T*T)/(2*T*T*T);
    coeffs(4) = (30*q0 - 30*qf + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T*T)/(2*T*T*T*T);
    coeffs(5) = (12*qf - 12*q0 - (6*vf + 6*v0)*T - (a0 - af)*T*T)/(2*T*T*T*T*T);
}

double TrajectoryPlanner::quinticPosition(double t, const VectorXd& coeffs) {
    return coeffs(0) + coeffs(1)*t + coeffs(2)*t*t + 
           coeffs(3)*t*t*t + coeffs(4)*t*t*t*t + coeffs(5)*t*t*t*t*t;
}

double TrajectoryPlanner::quinticVelocity(double t, const VectorXd& coeffs) {
    return coeffs(1) + 2*coeffs(2)*t + 3*coeffs(3)*t*t + 
           4*coeffs(4)*t*t*t + 5*coeffs(5)*t*t*t*t;
}

double TrajectoryPlanner::quinticAcceleration(double t, const VectorXd& coeffs) {
    return 2*coeffs(2) + 6*coeffs(3)*t + 
           12*coeffs(4)*t*t + 20*coeffs(5)*t*t*t;
}

void TrajectoryPlanner::interpolateRotation(const Matrix3d& R_start, const Matrix3d& R_end,
                                          double t, Matrix3d& R_interp) {
    try {
        // 检查输入矩阵是否有效
        if (!R_start.allFinite() || !R_end.allFinite()) {
            // 如果输入矩阵包含NaN或Inf，则使用恒等矩阵
            Serial.println("警告: 插值旋转矩阵包含无效值，使用恒等矩阵");
            R_interp = Matrix3d::Identity();
            return;
        }
        
        // 标准化参数t，确保在[0,1]范围内
        t = std::max(0.0, std::min(1.0, t));
        
        // 尝试构造四元数
        Quaterniond q_start, q_end;
        
        // 安全地从旋转矩阵创建四元数
        Matrix3d Rs_orthogonal = R_start;
        Matrix3d Re_orthogonal = R_end;
        
        // 正交化矩阵以确保它们是有效的旋转矩阵
        Eigen::JacobiSVD<Matrix3d> svd_start(Rs_orthogonal, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<Matrix3d> svd_end(Re_orthogonal, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        Rs_orthogonal = svd_start.matrixU() * svd_start.matrixV().transpose();
        Re_orthogonal = svd_end.matrixU() * svd_end.matrixV().transpose();
        
        // 使用正交化后的矩阵创建四元数
        q_start = Quaterniond(Rs_orthogonal);
        q_end = Quaterniond(Re_orthogonal);
        
        // 归一化四元数
        q_start.normalize();
        q_end.normalize();
        
        // 确保走最短路径
        if(q_start.dot(q_end) < 0) {
            q_end.coeffs() = -q_end.coeffs();
        }
        
        // 执行球面线性插值
        Quaterniond q_interp;
        
        // 一种更安全的球面插值方法
        if (t <= 0.0) {
            q_interp = q_start;
        } else if (t >= 1.0) {
            q_interp = q_end;
        } else {
            // 计算夹角
            double dot = q_start.dot(q_end);
            dot = std::max(-1.0, std::min(1.0, dot)); // 限制在[-1,1]范围内
            double omega = acos(dot);
            
            // 避免除以零错误
            if (fabs(omega) < 1e-10) {
                q_interp = q_start;
            } else {
                double sinOmega = sin(omega);
                double s0 = sin((1.0 - t) * omega) / sinOmega;
                double s1 = sin(t * omega) / sinOmega;
                
                // 手动混合四元数
                q_interp.w() = s0 * q_start.w() + s1 * q_end.w();
                q_interp.x() = s0 * q_start.x() + s1 * q_end.x();
                q_interp.y() = s0 * q_start.y() + s1 * q_end.y();
                q_interp.z() = s0 * q_start.z() + s1 * q_end.z();
                q_interp.normalize();
            }
        }
        
        // 将四元数转换为旋转矩阵
        R_interp = q_interp.toRotationMatrix();
        
        // 确保输出是有效的旋转矩阵
        if (!R_interp.allFinite()) {
            Serial.println("警告: 插值结果包含无效值，使用恒等矩阵");
            R_interp = Matrix3d::Identity();
        }
    }
    catch (const std::exception& e) {
        Serial.print("旋转插值异常: ");
        Serial.println(e.what());
        // 出错时返回恒等矩阵
        R_interp = Matrix3d::Identity();
    }
    catch (...) {
        Serial.println("旋转插值未知异常");
        // 出错时返回恒等矩阵
        R_interp = Matrix3d::Identity();
    }
}

double TrajectoryPlanner::calculateMinimumTime(const VectorXd& q_start, const VectorXd& q_end) {
    double max_time = 0;
    for(int i = 0; i < ARM_DOF; i++) {
        double dq = abs(q_end(i) - q_start(i));
        // 考虑速度和加速度约束
        double t1 = sqrt(dq / max_joint_acc_);  // 加速到最大速度所需时间
        double t2 = dq / max_joint_vel_;        // 匀速运动所需时间
        max_time = std::max(max_time, 2*t1 + t2);
    }
    return max_time;
}

bool TrajectoryPlanner::checkDynamicConstraints(const MatrixXd& trajectory, double dt) {
    int n_points = trajectory.rows();
    
    for(int i = 1; i < n_points-1; i++) {
        // 计算速度（使用中心差分）
        VectorXd vel = (trajectory.row(i+1) - trajectory.row(i-1)) / (2*dt);
        
        // 计算加速度
        VectorXd acc = (trajectory.row(i+1) - 2*trajectory.row(i) + trajectory.row(i-1)) / (dt*dt);
        
        // 检查约束
        if(vel.cwiseAbs().maxCoeff() > max_joint_vel_ ||
           acc.cwiseAbs().maxCoeff() > max_joint_acc_) {
            return false;
        }
    }
    return true;
}

void TrajectoryPlanner::planCompositeTrajectory(const std::vector<TrajectorySegment>& segments,
                                              RobotKinematics& kinematics,
                                              MatrixXd& trajectory, VectorXd& time_points) {
    // 复合轨迹规划需要特殊处理点数限制
    // 首先统计所有段的总持续时间
    double total_duration = 0;
    for(const auto& seg : segments) {
        total_duration += seg.duration;
    }
    
    // 计算每个段应该分配的最大点数
    std::vector<int> segment_max_points;
    int total_max_points = max_points_ - segments.size() + 1; // 减去重叠的点（每段末尾和下一段开头是同一个点）
    
    // 确保至少有最小点数
    total_max_points = std::max(total_max_points, (int)segments.size() + 1);
    
    // 根据持续时间比例分配点数给每个段
    for(const auto& seg : segments) {
        // 计算该段应分配的点数比例
        double ratio = seg.duration / total_duration;
        // 给每个段分配至少2个点（起点和终点）
        int seg_points = std::max(2, (int)ceil(ratio * total_max_points));
        segment_max_points.push_back(seg_points);
    }
    
    // 调整点数分配以确保总点数在限制范围内
    int sum_points = 0;
    for(int points : segment_max_points) {
        sum_points += points - 1; // 减去每段与下一段的重叠点
    }
    sum_points += 1; // 加回最后一段的终点
    
    // 如果总点数超过限制，按比例减少
    if(sum_points > max_points_) {
        double scale_factor = (double)(max_points_ - 1) / (sum_points - 1);
        sum_points = 0;
        for(size_t i = 0; i < segment_max_points.size(); i++) {
            // 重新计算点数，确保至少有2个点
            segment_max_points[i] = std::max(2, (int)ceil((segment_max_points[i] - 1) * scale_factor) + 1);
            sum_points += (i < segment_max_points.size() - 1) ? (segment_max_points[i] - 1) : segment_max_points[i];
        }
    }
    
    // 输出调试信息
    Serial.printf("复合轨迹规划: 总持续时间=%.2f秒, 总段数=%d, 总点数=%d\n", 
                  total_duration, segments.size(), sum_points);
    
    // 预分配内存
    trajectory.resize(sum_points, ARM_DOF);
    time_points.resize(sum_points);
    
    int current_point = 0;
    double current_time = 0;
    
    // 处理每个轨迹段
    for(size_t seg_idx = 0; seg_idx < segments.size(); seg_idx++) {
        const auto& seg = segments[seg_idx];
        int seg_max_points = segment_max_points[seg_idx];
        
        // 临时存储每个段的轨迹
        MatrixXd seg_traj;
        VectorXd seg_times;
        
        // 计算这个段适合的采样时间
        double seg_sample_time = calculateSampleTime(seg.duration, seg_max_points);
        
        // 暂时调整采样时间以计算这个段
        double original_sample_time = sample_time_;
        setTimeParams(seg_sample_time, max_joint_vel_, max_joint_acc_);
        
        switch(seg.type) {
            case SegmentType::JOINT_SPACE:
                planJointTrajectory(seg.start_config, seg.end_config,
                                  seg.duration, seg_traj, seg_times);
                break;
                
            case SegmentType::CARTESIAN_LINE:
                planCartesianLine(seg.start_pose, seg.end_pose,
                                seg.duration, kinematics, seg_traj, seg_times);
                break;
                
            case SegmentType::CARTESIAN_ARC:
                planCartesianArc(seg.start_pose, seg.end_pose,
                               seg.via_point, seg.duration,
                               kinematics, seg_traj, seg_times);
                break;
        }
        
        // 恢复原始采样时间
        setTimeParams(original_sample_time, max_joint_vel_, max_joint_acc_);
        
        // 合并到完整轨迹中
        int n_seg_points = seg_traj.rows();
        
        // 如果不是第一段，跳过第一个点（因为它应该与上一段的最后一个点相同）
        int start_idx = (seg_idx == 0) ? 0 : 1;
        
        // 复制轨迹数据
        for(int i = start_idx; i < n_seg_points; i++) {
            trajectory.row(current_point) = seg_traj.row(i);
            time_points(current_point) = seg_times(i) + current_time;
            current_point++;
        }
        
        // 更新累计时间
        current_time += seg.duration;
    }
    
    // 调整实际使用的轨迹大小（以防计算中出现误差）
    if(current_point < sum_points) {
        trajectory.conservativeResize(current_point, ARM_DOF);
        time_points.conservativeResize(current_point);
        
        Serial.printf("实际轨迹点数: %d (小于预期的 %d)\n", current_point, sum_points);
    }
}