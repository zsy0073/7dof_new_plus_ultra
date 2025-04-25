#ifndef ROTATION_UTILS_H
#define ROTATION_UTILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Quaterniond;

/**
 * 旋转矩阵工具类，提供各种旋转表示方法之间的转换和插值功能
 */
class RotationUtils {
public:
    /**
     * 在两个旋转矩阵之间进行球面线性插值
     * @param R_start 起始旋转矩阵
     * @param R_end 结束旋转矩阵
     * @param t 插值参数，范围[0,1]
     * @param R_interp 输出插值后的旋转矩阵
     * @return 成功返回true，失败返回false
     */
    static bool interpolateRotation(const Matrix3d& R_start, const Matrix3d& R_end, 
                                   double t, Matrix3d& R_interp);
    
    /**
     * 欧拉角转旋转矩阵 (ZYX顺序)
     * @param roll X轴旋转角度 (弧度)
     * @param pitch Y轴旋转角度 (弧度)
     * @param yaw Z轴旋转角度 (弧度)
     * @return 旋转矩阵
     */
    static Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw);
    
    /**
     * 旋转矩阵转欧拉角 (ZYX顺序)
     * @param R 旋转矩阵
     * @param roll 输出X轴旋转角度 (弧度)
     * @param pitch 输出Y轴旋转角度 (弧度)
     * @param yaw 输出Z轴旋转角度 (弧度)
     * @return 成功返回true，失败返回false
     */
    static bool rotationMatrixToEuler(const Matrix3d& R, double& roll, double& pitch, double& yaw);
    
    /**
     * 检查旋转矩阵是否有效 (正交且行列式为1)
     * @param R 待检查的旋转矩阵
     * @param tolerance 容差
     * @return 有效返回true，无效返回false
     */
    static bool isValidRotationMatrix(const Matrix3d& R, double tolerance = 1e-6);
    
    /**
     * 正交化旋转矩阵，确保其为有效的旋转矩阵
     * @param R 输入输出旋转矩阵
     */
    static void orthogonalizeRotationMatrix(Matrix3d& R);
    
    /**
     * 获取单位矩阵
     * @return 3x3单位矩阵
     */
    static Matrix3d identity() {
        return Matrix3d::Identity();
    }
    
    /**
     * 创建绕X轴旋转的旋转矩阵
     * @param angle 旋转角度 (弧度)
     * @return 旋转矩阵
     */
    static Matrix3d rotationX(double angle);
    
    /**
     * 创建绕Y轴旋转的旋转矩阵
     * @param angle 旋转角度 (弧度)
     * @return 旋转矩阵
     */
    static Matrix3d rotationY(double angle);
    
    /**
     * 创建绕Z轴旋转的旋转矩阵
     * @param angle 旋转角度 (弧度)
     * @return 旋转矩阵
     */
    static Matrix3d rotationZ(double angle);
};

#endif // ROTATION_UTILS_H