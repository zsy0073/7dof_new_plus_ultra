#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <Eigen.h>
#include <Eigen/Dense>
#include "Config.h"

using namespace Eigen;

class Utils {
public:
  // ===== 舵机角度转换工具 =====
  
  // 将角度值(0-240度)转换为舵机控制值(0-1000)
  static int angleToPulse(float angle);
  
  // 将舵机控制值(0-1000)转换为角度值(0-240度)
  static float pulseToAngle(int pulse);
  
  // 将轨迹规划角度(-120到+120度，0为中位)转换为舵机物理角度(0-240度)
  static float trajAngleToServoAngle(float trajAngle);
  
  // 将舵机物理角度(0-240度)转换为轨迹规划角度(-120到+120度，0为中位)
  static float servoAngleToTrajAngle(float servoAngle);
  
  // 直接将轨迹规划角度转换为舵机控制值
  static int trajAngleToPulse(float trajAngle);
  
  // 直接将舵机控制值转换为轨迹规划角度
  static float pulseToTrajAngle(int pulse);
  
  // 简化版根据移动距离计算移动时间
  static int calculateMoveTime(int distance);
  
  // 计算自动复位的时间
  static int calculateAutoResetTime();
  
  // ===== 旋转矩阵工具 =====
  
  // 创建X轴旋转矩阵
  static Matrix3d rotationX(double angle);
  
  // 创建Y轴旋转矩阵
  static Matrix3d rotationY(double angle);
  
  // 创建Z轴旋转矩阵
  static Matrix3d rotationZ(double angle);
  
  // 欧拉角转旋转矩阵 (ZYX顺序)
  static Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw);
  
  // 旋转矩阵转欧拉角 (ZYX顺序)
  static bool rotationMatrixToEuler(const Matrix3d& R, double& roll, double& pitch, double& yaw);
  
  // 检查矩阵是否为有效的旋转矩阵
  static bool isValidRotationMatrix(const Matrix3d& R, double tolerance = 1e-6);
  
  // 正交化旋转矩阵 (Gram-Schmidt正交化)
  static void orthogonalizeRotationMatrix(Matrix3d& R);
  
  // 在两个旋转矩阵之间进行球面线性插值
  static bool interpolateRotation(const Matrix3d& R_start, const Matrix3d& R_end, 
                                 double t, Matrix3d& R_interp);
  
  // ===== 其他辅助计算函数 =====
  
  // 计算五次多项式系数
  static void computeQuinticCoeff(double q0, double qf, double T, VectorXd& coeffs);
  
  // 计算五次多项式位置
  static double quinticPosition(double t, const VectorXd& coeffs);
  
  // 计算五次多项式速度
  static double quinticVelocity(double t, const VectorXd& coeffs);
  
  // 计算五次多项式加速度
  static double quinticAcceleration(double t, const VectorXd& coeffs);
  
  // 计算最小运动时间
  static double calculateMinimumTime(const VectorXd& q_start, const VectorXd& q_end);
};

// ===== 显示工具函数 =====
// 这些函数不是类的一部分，可以直接在全局范围内调用

// 显示普通消息
void displayMessage();

// 显示进度消息
void displayProgressMessage(const String& task, int progress);

// 显示带有点数信息的进度消息
void displayProgressWithPoints(const String& task, int progress, int currentPoints, int totalPoints);

// 显示错误消息
void displayErrorMessage(const String& error);

#endif // UTILS_H