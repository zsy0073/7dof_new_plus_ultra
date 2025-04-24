#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include "RobotKinematics.h"

using namespace Eigen;

// 轨迹段类型枚举
enum class SegmentType {
    JOINT_SPACE,    // 关节空间轨迹
    CARTESIAN_LINE, // 笛卡尔直线轨迹
    CARTESIAN_ARC   // 笛卡尔圆弧轨迹
};

// 轨迹段类
struct TrajectorySegment {
    SegmentType type;
    double duration;
    VectorXd start_config;
    VectorXd end_config;
    Matrix4d start_pose;
    Matrix4d end_pose;
    VectorXd via_point;  // 用于圆弧轨迹的中间点
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner();
    
    // 设置采样时间和速度限制
    void setTimeParams(double sample_time, double max_joint_vel, double max_joint_acc);
    
    // 设置最大轨迹点数
    void setMaxPoints(int max_points);
    
    // 获取当前最大点数设置
    int getMaxPoints() const;
    
    // 五次多项式轨迹规划
    void planJointTrajectory(const VectorXd& q_start, const VectorXd& q_end,
                           double duration, MatrixXd& trajectory, VectorXd& time_points);
    
    // 笛卡尔空间直线轨迹规划
    void planCartesianLine(const Matrix4d& T_start, const Matrix4d& T_end,
                         double duration, RobotKinematics& kinematics,
                         MatrixXd& trajectory, VectorXd& time_points);
                         
    // 笛卡尔空间圆弧轨迹规划
    void planCartesianArc(const Matrix4d& T_start, const Matrix4d& T_end,
                        const Vector3d& via_point, double duration,
                        RobotKinematics& kinematics,
                        MatrixXd& trajectory, VectorXd& time_points);
    
    // 复合轨迹规划（支持多段轨迹）
    void planCompositeTrajectory(const std::vector<TrajectorySegment>& segments,
                               RobotKinematics& kinematics,
                               MatrixXd& trajectory, VectorXd& time_points);
                               
    // 笛卡尔空间Pick&Place轨迹规划
    void planPickAndPlace(const Matrix4d& T_current, const Matrix4d& pick_pos, const Matrix4d& place_pos,
                        double lift_height, double move_time,
                        RobotKinematics& kinematics,
                        MatrixXd& trajectory, VectorXd& time_points);

private:
    double sample_time_;      // 采样时间间隔
    double max_joint_vel_;    // 最大关节速度
    double max_joint_acc_;    // 最大关节加速度
    int max_points_;          // 最大轨迹点数
    
    // 计算适应点数限制的采样时间
    double calculateSampleTime(double duration, int desired_points);
    
    // 五次多项式系数计算
    void computeQuinticCoeff(double q0, double qf, double T,
                           VectorXd& coeffs);
    
    // 计算五次多项式在给定时间的位置
    double quinticPosition(double t, const VectorXd& coeffs);
    
    // 计算五次多项式在给定时间的速度
    double quinticVelocity(double t, const VectorXd& coeffs);
    
    // 计算五次多项式在给定时间的加速度
    double quinticAcceleration(double t, const VectorXd& coeffs);
    
    // 姿态插值（使用四元数球面插值）
    void interpolateRotation(const Matrix3d& R_start, const Matrix3d& R_end,
                           double t, Matrix3d& R_interp);
                           
    // 计算最小轨迹时间
    double calculateMinimumTime(const VectorXd& q_start, const VectorXd& q_end);
    
    // 速度和加速度约束检查
    bool checkDynamicConstraints(const MatrixXd& trajectory, double dt);
};

#endif // TRAJECTORY_PLANNER_H