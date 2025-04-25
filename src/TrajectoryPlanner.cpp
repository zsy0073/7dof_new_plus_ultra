#include "TrajectoryPlanner.h"
#include <math.h>
#include <Eigen/Geometry>  // 用于四元数计算
#include <Arduino.h>       // 添加Arduino头文件以支持Serial对象
#include "RotationUtils.h" // 添加RotationUtils头文件

// 定义日志级别
#define LOG_ERROR   0
#define LOG_WARNING 1
#define LOG_INFO    2
#define LOG_DEBUG   3

// 设置当前日志级别（可以在config.h中定义）
#define CURRENT_LOG_LEVEL LOG_WARNING

// 日志宏，根据日志级别控制输出
#define LOG_E(fmt, ...) if(CURRENT_LOG_LEVEL >= LOG_ERROR)   { Serial.printf("[ERROR] " fmt "\n", ##__VA_ARGS__); }
#define LOG_W(fmt, ...) if(CURRENT_LOG_LEVEL >= LOG_WARNING) { Serial.printf("[WARN]  " fmt "\n", ##__VA_ARGS__); }
#define LOG_I(fmt, ...) if(CURRENT_LOG_LEVEL >= LOG_INFO)    { Serial.printf("[INFO]  " fmt "\n", ##__VA_ARGS__); }
#define LOG_D(fmt, ...) if(CURRENT_LOG_LEVEL >= LOG_DEBUG)   { Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__); }

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
    
    LOG_I("关节轨迹规划: 时长=%.2fs, 采样=%.4fs, 点数=%d", duration, effective_sample_time, n_points);
    
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
        
        LOG_D("轨迹点 #%d (t=%.2fs): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]°", 
            j+1, t, 
            trajectory(j, 0)*180/M_PI, trajectory(j, 1)*180/M_PI, 
            trajectory(j, 2)*180/M_PI, trajectory(j, 3)*180/M_PI, 
            trajectory(j, 4)*180/M_PI, trajectory(j, 5)*180/M_PI, 
            trajectory(j, 6)*180/M_PI);
    }
    
    // 平滑轨迹中的角度突变
    smoothJointTrajectory(trajectory, 60.0); // 60度作为默认阈值
    
    LOG_I("关节轨迹规划完成");
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
    
    LOG_I("笛卡尔直线轨迹: 时长=%.2fs, 采样=%.4fs, 点数=%d", duration, effective_sample_time, n_points);
    
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
        LOG_W("起点位置无效 (全为零)! 使用安全位置替代");
        // 使用一个安全的默认位置作为备选
        p_start << 0.341, 0.000, 0.231;
    }
    
    LOG_D("起点: [%.3f, %.3f, %.3f], 终点: [%.3f, %.3f, %.3f]", 
          p_start(0), p_start(1), p_start(2), p_end(0), p_end(1), p_end(2));
    
    // 检查路径长度，确保不超过工作空间
    double path_length = (p_end - p_start).norm();
    LOG_D("路径长度: %.3f 米", path_length);
    
    if(path_length > 1.0) {  // 假设最大工作空间半径为1米
        LOG_E("路径长度超出工作空间限制!");
        throw std::runtime_error("Path length exceeds workspace limits");
    }
    
    // 定义一个非奇异的参考配置作为初始位置
    VectorXd reference_config(ARM_DOF);
    // 使用以度为单位的参考配置 (0 30 0 60 0 45 0)
    reference_config << 0 * M_PI/180.0, 30 * M_PI/180.0, 0 * M_PI/180.0, 
                      60 * M_PI/180.0, 0 * M_PI/180.0, 45 * M_PI/180.0, 
                      0 * M_PI/180.0;

    // 计算每个时间点的位姿
    bool solution_found = false;
    int max_attempts = 1;  // 最大重试次数
    
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
        
        LOG_D("计算点 %d/%d: t=%.2fs, 位置=[%.3f, %.3f, %.3f]", i+1, n_points, t, p(0), p(1), p(2));
        
        // 使用多次尝试求解逆运动学，确保找到最优解
        solution_found = false;
        VectorXd q_current;
        VectorXd q_best;
        double min_joint_movement = std::numeric_limits<double>::max();
        
        for(int attempt = 0; attempt < max_attempts; attempt++) {
            LOG_D("  尝试求解逆运动学 %d/%d", attempt+1, max_attempts);
            
            VectorXd q_attempt;
            bool success;
            
            if(i == 0) {
                // 对于第一个点，使用预定义的非奇异参考配置作为初始猜测值
                success = kinematics.inverseKinematics(T_current, reference_config, q_attempt);
                
                // 如果使用参考配置失败，输出警告但不要尝试其他配置
                if(!success) {
                    LOG_W("无奇异参考配置逆解失败！使用参考配置作为退化方案");
                    q_attempt = reference_config;
                    success = true;  // 强制使用参考配置
                }
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
                
                LOG_D("  逆运动学求解成功，关节运动量=%.3f", joint_movement);
                
                // 更新最优解
                if(joint_movement < min_joint_movement) {
                    min_joint_movement = joint_movement;
                    q_best = q_attempt;
                    solution_found = true;
                }
            } else {
                LOG_D("  逆运动学求解失败");
            }
        }
        
        if(!solution_found) {
            LOG_E("无法找到有效的逆运动学解! t=%.2fs", t);
            throw std::runtime_error("No valid inverse kinematics solution found at time " + std::to_string(t));
        }
        
        trajectory.row(i) = q_best;
        
        // 检查速度约束
        if(i > 0) {
            VectorXd joint_vel = (trajectory.row(i) - trajectory.row(i-1)) / effective_sample_time;
            double max_vel = joint_vel.cwiseAbs().maxCoeff();
            
            if(max_vel > max_joint_vel_) {
                LOG_W("关节速度超出限制 t=%.2fs (%.3f > %.3f)", t, max_vel, max_joint_vel_);
            }
        }
    }
    
    // 最后检查整个轨迹的动力学约束
    LOG_D("检查整个轨迹的动力学约束...");
    if(!checkDynamicConstraints(trajectory, effective_sample_time)) {
        LOG_W("轨迹中存在违反动力学约束的情况!");
    }
    
    // 平滑轨迹中的角度突变
    smoothJointTrajectory(trajectory, 60.0);
    
    LOG_I("笛卡尔直线轨迹规划完成");
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
    
    LOG_I("笛卡尔圆弧轨迹: 时长=%.2fs, 采样=%.4fs, 点数=%d", duration, effective_sample_time, n_points);
    
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
        LOG_E("三点共线，无法形成圆弧");
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
    
    // 非奇异参考配置
    VectorXd reference_config(ARM_DOF);
    reference_config << 0 * M_PI/180.0, 30 * M_PI/180.0, 0 * M_PI/180.0, 
                       60 * M_PI/180.0, 0 * M_PI/180.0, 45 * M_PI/180.0, 
                       0 * M_PI/180.0;
    
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
            // 对于第一个点，使用预定义的非奇异参考配置
            bool success = kinematics.inverseKinematics(T_current, reference_config, q_current);
            // 如果使用参考配置失败，强制使用参考配置
            if(!success) {
                LOG_W("圆弧轨迹中无奇异参考配置逆解失败！使用参考配置作为退化方案");
                q_current = reference_config;
            }
        } else {
            kinematics.inverseKinematics(T_current, trajectory.row(i-1), q_current);
        }
        
        trajectory.row(i) = q_current;
        
        LOG_D("圆弧点 %d/%d: t=%.2fs, 角度=%.1f°, 位置=[%.3f, %.3f, %.3f]", 
             i+1, n_points, t, current_angle*180/M_PI, p_current(0), p_current(1), p_current(2));
    }
    
    // 平滑轨迹中的角度突变
    smoothJointTrajectory(trajectory, 60.0);
    
    LOG_I("笛卡尔圆弧轨迹规划完成");
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
    
    // 7. 新增：从放置位置上方回到预定义的非奇异初始位置
    
    // 定义标准非奇异初始位置关节配置 (0 30 0 60 0 45 0度)
    VectorXd non_singular_config(ARM_DOF);
    non_singular_config << 0 * M_PI/180.0, 30 * M_PI/180.0, 0 * M_PI/180.0, 
                           60 * M_PI/180.0, 0 * M_PI/180.0, 45 * M_PI/180.0, 
                           0 * M_PI/180.0;
    
    // 使用正运动学计算非奇异初始位置的位姿矩阵
    Matrix4d non_singular_pose;
    kinematics.forwardKinematics(non_singular_config, non_singular_pose);
    
    // 添加返回非奇异初始位置的轨迹段
    seg.duration = move_time;
    seg.start_pose = place_lift;
    seg.end_pose = non_singular_pose;  // 使用非奇异初始位置作为返回目标
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
    // 使用RotationUtils工具类进行旋转矩阵插值
    if (!RotationUtils::interpolateRotation(R_start, R_end, t, R_interp)) {
        LOG_W("旋转插值失败，使用恒等矩阵");
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
    
    LOG_I("复合轨迹: 总时长=%.2fs, 段数=%d, 总点数=%d", total_duration, segments.size(), sum_points);
    
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
        
        LOG_D("实际轨迹点数: %d (小于预期的 %d)", current_point, sum_points);
    }
    
    // 平滑轨迹中的角度突变
    smoothJointTrajectory(trajectory, 60.0);
    
    LOG_I("复合轨迹规划完成，实际点数: %d", current_point);
}

// 改进：全面检测和修复轨迹中的角度突变
void TrajectoryPlanner::smoothJointTrajectory(MatrixXd& trajectory, double angle_threshold) {
    if (trajectory.rows() <= 1) {
        return; // 轨迹点太少，无需平滑
    }
    
    // 角度阈值（默认为60度，即约1.05弧度）
    double rad_threshold = angle_threshold * M_PI / 180.0;
    
    LOG_D("平滑轨迹: 点数=%d, 阈值=%.1f°", trajectory.rows(), angle_threshold);
    
    int total_corrections = 0;

    // 对所有关节进行平滑处理
    for (int joint = 0; joint < ARM_DOF; joint++) {
        // 第一遍：检测大角度突变，并使用2π跳变修正
        for (int i = 1; i < trajectory.rows(); i++) {
            // 当前角度与前一个角度
            double prev_angle = trajectory(i-1, joint);
            double curr_angle = trajectory(i, joint);
            
            // 计算角度差
            double angle_diff = curr_angle - prev_angle;
            
            // 检查是否为2π跳变（超过330度的变化，接近2π）
            if (fabs(angle_diff) > (330 * M_PI / 180.0)) {
                // 2π跳变修正
                if (angle_diff > 0) {
                    trajectory(i, joint) = curr_angle - 2*M_PI;
                } else {
                    trajectory(i, joint) = curr_angle + 2*M_PI;
                }
                
                LOG_D("修正2π跳变: 关节%d, 点%d", joint+1, i);
                total_corrections++;
            }
        }
        
        // 第二遍：检测关节限位附近的突变
        double joint_limit = 2.094; // 约120度的弧度表示
        
        for (int i = 1; i < trajectory.rows(); i++) {
            double prev_angle = trajectory(i-1, joint);
            double curr_angle = trajectory(i, joint);
            double angle_diff = curr_angle - prev_angle;
            
            // 仅处理超过阈值的角度变化
            if (fabs(angle_diff) > rad_threshold) {
                LOG_D("检测到角度突变: 关节%d, 点%d, 差值%.2f°", joint+1, i, angle_diff*180/M_PI);
                
                // 情况1: 从接近正限位跳变到接近负限位
                if (prev_angle > (joint_limit*0.8) && curr_angle < (-joint_limit*0.8)) {
                    double adjusted_angle = curr_angle + 2*joint_limit;
                    // 确保调整后的角度不超过正限位
                    if (adjusted_angle <= joint_limit) {
                        trajectory(i, joint) = adjusted_angle;
                        total_corrections++;
                    }
                }
                // 情况2: 从接近负限位跳变到接近正限位
                else if (prev_angle < (-joint_limit*0.8) && curr_angle > (joint_limit*0.8)) {
                    double adjusted_angle = curr_angle - 2*joint_limit;
                    // 确保调整后的角度不低于负限位
                    if (adjusted_angle >= -joint_limit) {
                        trajectory(i, joint) = adjusted_angle;
                        total_corrections++;
                    }
                }
            }
        }
    }
    
    // 第三遍：进行平滑过渡处理，减小仍然存在的较大角度变化
    for (int joint = 0; joint < ARM_DOF; joint++) {
        for (int i = 1; i < trajectory.rows()-1; i++) {
            double prev_angle = trajectory(i-1, joint);
            double curr_angle = trajectory(i, joint);
            double next_angle = trajectory(i+1, joint);
            
            double diff1 = fabs(curr_angle - prev_angle);
            double diff2 = fabs(next_angle - curr_angle);
            
            // 如果当前点与前后点都有较大差异，可能是孤立的异常点
            if (diff1 > rad_threshold && diff2 > rad_threshold) {
                // 使用线性插值替换
                double avg_angle = (prev_angle + next_angle) / 2.0;
                trajectory(i, joint) = avg_angle;
                LOG_D("平滑孤立点: 关节%d, 点%d", joint+1, i);
                total_corrections++;
            }
        }
    }
    
    if (total_corrections > 0) {
        LOG_I("轨迹平滑完成: 修正了%d处角度突变", total_corrections);
    }
}