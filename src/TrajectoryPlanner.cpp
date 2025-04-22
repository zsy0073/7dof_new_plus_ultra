#include "TrajectoryPlanner.h"
#include <math.h>
#include <Eigen/Geometry>  // 用于四元数计算

TrajectoryPlanner::TrajectoryPlanner() {
    sample_time_ = 0.01;      // 默认采样时间10ms
    max_joint_vel_ = 1.0;     // 默认最大关节速度1 rad/s
    max_joint_acc_ = 2.0;     // 默认最大关节加速度2 rad/s^2
}

void TrajectoryPlanner::setTimeParams(double sample_time, double max_joint_vel, double max_joint_acc) {
    sample_time_ = sample_time;
    max_joint_vel_ = max_joint_vel;
    max_joint_acc_ = max_joint_acc;
}

void TrajectoryPlanner::planJointTrajectory(const VectorXd& q_start, const VectorXd& q_end,
                                          double duration, MatrixXd& trajectory, VectorXd& time_points) {
    int n_points = ceil(duration / sample_time_) + 1;
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
        double t = j * sample_time_;
        if(t > duration) t = duration;
        time_points(j) = t;
        
        for(int i = 0; i < ARM_DOF; i++) {
            trajectory(j, i) = quinticPosition(t, coeffs.row(i));
        }
    }
}

void TrajectoryPlanner::planCartesianLine(const Matrix4d& T_start, const Matrix4d& T_end,
                                        double duration, RobotKinematics& kinematics,
                                        MatrixXd& trajectory, VectorXd& time_points) {
    int n_points = ceil(duration / sample_time_) + 1;
    trajectory.resize(n_points, ARM_DOF);
    time_points.resize(n_points);
    
    // 提取起点和终点的位置和姿态
    Vector3d p_start = T_start.block<3,1>(0,3);
    Vector3d p_end = T_end.block<3,1>(0,3);
    Matrix3d R_start = T_start.block<3,3>(0,0);
    Matrix3d R_end = T_end.block<3,3>(0,0);
    
    // 检查路径长度，确保不超过工作空间
    double path_length = (p_end - p_start).norm();
    if(path_length > 1.0) {  // 假设最大工作空间半径为1米
        throw std::runtime_error("Path length exceeds workspace limits");
    }
    
    // 计算每个时间点的位姿
    bool solution_found = false;
    int max_attempts = 5;  // 最大重试次数
    
    for(int i = 0; i < n_points; i++) {
        double t = i * sample_time_;
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
        
        // 使用多次尝试求解逆运动学，确保找到最优解
        solution_found = false;
        VectorXd q_current;
        VectorXd q_best;
        double min_joint_movement = std::numeric_limits<double>::max();
        
        for(int attempt = 0; attempt < max_attempts; attempt++) {
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
                
                // 更新最优解
                if(joint_movement < min_joint_movement) {
                    min_joint_movement = joint_movement;
                    q_best = q_attempt;
                    solution_found = true;
                }
            }
        }
        
        if(!solution_found) {
            throw std::runtime_error("No valid inverse kinematics solution found at time " + std::to_string(t));
        }
        
        trajectory.row(i) = q_best;
        
        // 检查速度约束
        if(i > 0) {
            VectorXd joint_vel = (trajectory.row(i) - trajectory.row(i-1)) / sample_time_;
            if(joint_vel.cwiseAbs().maxCoeff() > max_joint_vel_) {
                throw std::runtime_error("Joint velocity limit exceeded at time " + std::to_string(t));
            }
        }
    }
    
    // 最后检查整个轨迹的动力学约束
    if(!checkDynamicConstraints(trajectory, sample_time_)) {
        throw std::runtime_error("Dynamic constraints violated in the trajectory");
    }
}

void TrajectoryPlanner::planCartesianArc(const Matrix4d& T_start, const Matrix4d& T_end,
                                       const Vector3d& via_point, double duration,
                                       RobotKinematics& kinematics,
                                       MatrixXd& trajectory, VectorXd& time_points) {
    int n_points = ceil(duration / sample_time_) + 1;
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
        double t = i * sample_time_;
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
    }
}

void TrajectoryPlanner::planPickAndPlace(const Matrix4d& pick_pos, const Matrix4d& place_pos,
                                       double lift_height, double move_time,
                                       RobotKinematics& kinematics,
                                       MatrixXd& trajectory, VectorXd& time_points) {
    std::vector<TrajectorySegment> segments;
    
    // 创建提升位置矩阵
    Matrix4d pick_lift = pick_pos;
    pick_lift(2,3) += lift_height;
    Matrix4d place_lift = place_pos;
    place_lift(2,3) += lift_height;
    
    // 初始姿态
    VectorXd q_home = VectorXd::Zero(ARM_DOF);
    q_home << 0, M_PI/6, 0, M_PI/3, 0, M_PI/3, 0;
    
    // 定义轨迹段
    TrajectorySegment seg;
    
    // 1. 从初始位置到抓取位置上方
    seg.type = SegmentType::CARTESIAN_LINE;
    seg.duration = move_time;
    seg.start_pose = Matrix4d::Identity();
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
    // 使用四元数进行球面线性插值
    Quaterniond q_start(R_start);
    Quaterniond q_end(R_end);
    
    // 确保走最短路径
    if(q_start.dot(q_end) < 0) {
        q_end.coeffs() = -q_end.coeffs();
    }
    
    // 执行球面线性插值
    Quaterniond q_interp = q_start.slerp(t, q_end);
    R_interp = q_interp.toRotationMatrix();
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
    // 计算总点数
    int total_points = 0;
    for(const auto& seg : segments) {
        total_points += ceil(seg.duration / sample_time_) + 1;
    }
    
    trajectory.resize(total_points, ARM_DOF);
    time_points.resize(total_points);
    
    int current_point = 0;
    double current_time = 0;
    
    // 处理每个轨迹段
    for(const auto& seg : segments) {
        MatrixXd seg_traj;
        VectorXd seg_times;
        
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
        
        // 合并到完整轨迹中
        int n_seg_points = seg_traj.rows();
        trajectory.block(current_point, 0, n_seg_points, ARM_DOF) = seg_traj;
        time_points.segment(current_point, n_seg_points) = seg_times.array() + current_time;
        
        current_point += n_seg_points;
        current_time += seg.duration;
    }
    
    // 调整实际使用的轨迹大小
    trajectory.conservativeResize(current_point, ARM_DOF);
    time_points.conservativeResize(current_point);
}