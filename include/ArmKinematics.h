#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <Arduino.h>

// 常量定义
#define ARM_DOF 7              // 机械臂自由度(7轴)
#define MAX_ITERATIONS 100     // 逆解最大迭代次数
#define DEFAULT_TOLERANCE 1e-4 // 逆解默认收敛容差
#define IK_SUCCESS 0           // 逆解成功
#define IK_MAX_ITERATIONS 1    // 达到最大迭代次数
#define IK_INVALID_INPUT 2     // 输入参数无效

// 数据结构
typedef struct {
    float alpha[ARM_DOF];  // 关节扭角
    float a[ARM_DOF];      // 连杆长度
    float d[ARM_DOF];      // 关节偏置
    float theta[ARM_DOF];  // 关节角度
    float qlim[ARM_DOF][2]; // 关节限制 [min, max]
} RobotParams;

// 齐次变换矩阵 (4x4)
typedef struct {
    float data[4][4];
} Matrix4x4;

// 位姿类型
typedef struct {
    float position[3];     // 位置 [x, y, z]
    float orientation[3][3]; // 旋转矩阵
} Pose;

class ArmKinematics {
public:
    /**
     * 构造函数 - 初始化机器人参数(使用SRS七轴机械臂的MDH参数)
     */
    ArmKinematics();
    
    /**
     * 初始化自定义机器人参数
     * @param custom_params 用户自定义的机器人参数
     */
    ArmKinematics(const RobotParams& custom_params);
    
    /**
     * 正运动学计算 - 求解给定关节角度下的末端执行器位姿
     * @param joints 关节角度数组(弧度)
     * @param pose 输出的末端执行器位姿
     * @return true表示计算成功
     */
    bool forwardKinematics(const float joints[ARM_DOF], Matrix4x4 *pose);
    
    /**
     * 逆运动学计算 - 使用微分数值法求解末端执行器位姿对应的关节角度
     * @param target_pose 目标末端执行器位姿
     * @param initial_joints 初始猜测关节角度(弧度)，如为NULL则使用零位置
     * @param tolerance 收敛容差，如为0则使用默认值
     * @param max_iterations 最大迭代次数，如为0则使用默认值
     * @param result_joints 输出的关节角度(弧度)
     * @return 状态码：0表示成功，1表示达到最大迭代次数，2表示输入无效
     */
    int inverseKinematics(
        const Matrix4x4 *target_pose,
        const float *initial_joints,
        float tolerance,
        int max_iterations,
        float result_joints[ARM_DOF]
    );
    
    /**
     * 简化版逆运动学，使用默认参数
     * @param target_pose 目标末端执行器位姿
     * @param result_joints 输出的关节角度(弧度)
     * @return 状态码：0表示成功，1表示达到最大迭代次数，2表示输入无效
     */
    int inverseKinematics(
        const Matrix4x4 *target_pose,
        float result_joints[ARM_DOF]
    );
    
    /**
     * 更新机器人参数
     * @param new_params 新的机器人参数
     */
    void updateRobotParams(const RobotParams& new_params);
    
    /**
     * 获取当前机器人参数
     * @return 当前机器人参数
     */
    RobotParams getRobotParams();
    
    /**
     * 计算机械臂雅可比矩阵(用于逆解计算)
     * @param joints 当前关节角度
     * @param jacobian 输出的雅可比矩阵 (6×7)
     * @return true表示计算成功
     */
    bool calculateJacobian(
        const float joints[ARM_DOF],
        float jacobian[6][ARM_DOF]
    );
    
    /**
     * 将位姿矩阵设置为单位矩阵
     * @param m 要设置的矩阵
     */
    static void setIdentityMatrix(Matrix4x4 *m);
    
    /**
     * 创建位姿矩阵（位置+旋转）
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param rotX X轴旋转角度（弧度）
     * @param rotY Y轴旋转角度（弧度）
     * @param rotZ Z轴旋转角度（弧度）
     * @param m 输出的位姿矩阵
     */
    static void createPoseMatrix(float x, float y, float z, 
                                 float rotX, float rotY, float rotZ, 
                                 Matrix4x4 *m);
    
    /**
     * 打印位姿矩阵(调试用)
     */
    static void printMatrix(const Matrix4x4 *m);
    
    /**
     * 打印关节角度(调试用)
     */
    static void printJoints(const float joints[ARM_DOF]);

private:
    RobotParams robot_params;  // 机器人参数
    
    // 矩阵操作辅助函数
    static void matrixMultiply(const float A[4][4], const float B[4][4], float C[4][4]);
    
    // 计算MDH参数的变换矩阵
    static void calculateTransformationMatrix(float alpha, float a, float d, float theta, float T[4][4]);
    
    // 伪逆计算 (阻尼最小二乘法)
    void calculateDampedLeastSquares(
        const float jacobian[6][ARM_DOF], 
        const float error[6], 
        float lambda, 
        float alpha,
        float dq[ARM_DOF]
    );
};

#endif // ARM_KINEMATICS_H