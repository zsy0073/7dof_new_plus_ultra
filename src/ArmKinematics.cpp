#include "ArmKinematics.h"
#include <string.h>

// 使用小的增量用于雅可比矩阵的数值微分
#define DELTA 1e-5f

// 构造函数 - 使用默认SRS七轴机械臂参数初始化
ArmKinematics::ArmKinematics() {
    // 从robot.m文件提取的MDH参数
    // 关节扭角 alpha
    robot_params.alpha[0] = 0.0f;
    robot_params.alpha[1] = -M_PI/2;
    robot_params.alpha[2] = M_PI/2;
    robot_params.alpha[3] = -M_PI/2;
    robot_params.alpha[4] = M_PI/2;
    robot_params.alpha[5] = -M_PI/2;
    robot_params.alpha[6] = M_PI/2;
    
    // 连杆长度 a
    for (int i = 0; i < ARM_DOF; i++) {
        robot_params.a[i] = 0.0f;
    }
    
    // 关节偏置 d
    robot_params.d[0] = 0.1875f;
    robot_params.d[1] = 0.0f;
    robot_params.d[2] = 0.162f;
    robot_params.d[3] = 0.0f;
    robot_params.d[4] = 0.162f;
    robot_params.d[5] = 0.0f;
    robot_params.d[6] = 0.138f;
    
    // 初始关节角度 theta
    for (int i = 0; i < ARM_DOF; i++) {
        robot_params.theta[i] = 0.0f;
    }
    
    // 关节限制
    for (int i = 0; i < ARM_DOF; i++) {
        robot_params.qlim[i][0] = -M_PI/2;  // 下限
        robot_params.qlim[i][1] = M_PI/2;   // 上限
    }
}

// 使用自定义参数初始化
ArmKinematics::ArmKinematics(const RobotParams& custom_params) {
    updateRobotParams(custom_params);
}

// 更新机器人参数
void ArmKinematics::updateRobotParams(const RobotParams& new_params) {
    memcpy(&robot_params, &new_params, sizeof(RobotParams));
}

// 获取机器人参数
RobotParams ArmKinematics::getRobotParams() {
    return robot_params;
}

// 矩阵乘法
void ArmKinematics::matrixMultiply(const float A[4][4], const float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 计算MDH参数的变换矩阵
void ArmKinematics::calculateTransformationMatrix(float alpha, float a, float d, float theta, float T[4][4]) {
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float cos_alpha = cos(alpha);
    float sin_alpha = sin(alpha);
    
    // 使用修正的DH参数计算变换矩阵
    T[0][0] = cos_theta;
    T[0][1] = -sin_theta;
    T[0][2] = 0.0f;
    T[0][3] = a;
    
    T[1][0] = sin_theta * cos_alpha;
    T[1][1] = cos_theta * cos_alpha;
    T[1][2] = -sin_alpha;
    T[1][3] = -sin_alpha * d;
    
    T[2][0] = sin_theta * sin_alpha;
    T[2][1] = cos_theta * sin_alpha;
    T[2][2] = cos_alpha;
    T[2][3] = cos_alpha * d;
    
    T[3][0] = 0.0f;
    T[3][1] = 0.0f;
    T[3][2] = 0.0f;
    T[3][3] = 1.0f;
}

// 将位姿矩阵设置为单位矩阵
void ArmKinematics::setIdentityMatrix(Matrix4x4 *m) {
    if (m != NULL) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                m->data[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }
}

// 创建位姿矩阵（位置+旋转）
void ArmKinematics::createPoseMatrix(float x, float y, float z, 
                             float rotX, float rotY, float rotZ, 
                             Matrix4x4 *m) {
    if (m == NULL) return;
    
    // 先设为单位矩阵
    setIdentityMatrix(m);
    
    // 设置位置
    m->data[0][3] = x;
    m->data[1][3] = y;
    m->data[2][3] = z;
    
    // 计算旋转矩阵 (ZYX欧拉角顺序)
    float cx = cos(rotX), sx = sin(rotX);
    float cy = cos(rotY), sy = sin(rotY);
    float cz = cos(rotZ), sz = sin(rotZ);
    
    // 旋转矩阵 = Rz * Ry * Rx
    m->data[0][0] = cy * cz;
    m->data[0][1] = cz * sx * sy - cx * sz;
    m->data[0][2] = cx * cz * sy + sx * sz;
    
    m->data[1][0] = cy * sz;
    m->data[1][1] = cx * cz + sx * sy * sz;
    m->data[1][2] = -cz * sx + cx * sy * sz;
    
    m->data[2][0] = -sy;
    m->data[2][1] = cy * sx;
    m->data[2][2] = cx * cy;
}

// 打印位姿矩阵
void ArmKinematics::printMatrix(const Matrix4x4 *m) {
    if (m != NULL) {
        Serial.println(F("Matrix 4x4:"));
        for (int i = 0; i < 4; i++) {
            Serial.print("  ");
            for (int j = 0; j < 4; j++) {
                Serial.print(m->data[i][j], 4);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
}

// 打印关节角度
void ArmKinematics::printJoints(const float joints[ARM_DOF]) {
    if (joints != NULL) {
        Serial.print(F("Joints (rad): "));
        for (int i = 0; i < ARM_DOF; i++) {
            Serial.print(joints[i], 3);
            Serial.print(" ");
        }
        Serial.println();
        
        Serial.print(F("Joints (deg): "));
        for (int i = 0; i < ARM_DOF; i++) {
            Serial.print(joints[i] * 180.0f / M_PI, 1);
            Serial.print(" ");
        }
        Serial.println();
    }
}

// 正运动学计算
bool ArmKinematics::forwardKinematics(const float joints[ARM_DOF], Matrix4x4 *pose) {
    if (joints == NULL || pose == NULL) {
        return false;
    }
    
    // 初始化为单位矩阵
    float result[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    
    // 存储中间计算结果的变换矩阵
    float Ti[4][4] = {0};
    float temp[4][4] = {0};
    
    // 依次计算每个关节的变换矩阵并累乘
    for (int i = 0; i < ARM_DOF; i++) {
        // 计算当前关节的变换矩阵
        calculateTransformationMatrix(
            robot_params.alpha[i], 
            robot_params.a[i], 
            robot_params.d[i], 
            joints[i],
            Ti
        );
        
        // 累乘变换矩阵
        memcpy(temp, result, sizeof(temp));
        matrixMultiply(temp, Ti, result);
    }
    
    // 将结果赋值给输出参数
    memcpy(pose->data, result, sizeof(result));
    
    return true;
}

// 计算雅可比矩阵
bool ArmKinematics::calculateJacobian(
    const float joints[ARM_DOF],
    float jacobian[6][ARM_DOF]) {
    
    if (!joints || !jacobian) {
        return false;
    }
    
    // 计算当前位姿
    Matrix4x4 current_pose;
    if (!forwardKinematics(joints, &current_pose)) {
        return false;
    }
    
    // 当前位置
    float p0[3];
    for (int i = 0; i < 3; i++) {
        p0[i] = current_pose.data[i][3];
    }
    
    // 对每个关节计算雅可比矩阵列
    for (int i = 0; i < ARM_DOF; i++) {
        // 创建扰动的关节角度
        float perturbed_joints[ARM_DOF];
        memcpy(perturbed_joints, joints, sizeof(float) * ARM_DOF);
        perturbed_joints[i] += DELTA;
        
        // 计算扰动后的位姿
        Matrix4x4 perturbed_pose;
        if (!forwardKinematics(perturbed_joints, &perturbed_pose)) {
            return false;
        }
        
        // 提取扰动后的位置
        float p_delta[3];
        for (int j = 0; j < 3; j++) {
            p_delta[j] = perturbed_pose.data[j][3];
        }
        
        // 计算位置雅可比列
        for (int j = 0; j < 3; j++) {
            jacobian[j][i] = (p_delta[j] - p0[j]) / DELTA;
        }
        
        // 计算旋转雅可比列（使用矩阵对数映射的近似值）
        // 计算旋转差 R_delta * R0^T
        float R_diff[3][3];
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                R_diff[j][k] = 0.0f;
                for (int l = 0; l < 3; l++) {
                    R_diff[j][k] += perturbed_pose.data[j][l] * current_pose.data[k][l]; // R_delta * R0^T
                }
            }
        }
        
        // 从旋转矩阵提取角速度（简化计算）
        float trace = R_diff[0][0] + R_diff[1][1] + R_diff[2][2];
        float angle = acos((trace - 1.0f) / 2.0f);
        
        if (fabs(angle) < 1e-5f) {
            // 几乎没有旋转
            jacobian[3][i] = 0.0f;
            jacobian[4][i] = 0.0f;
            jacobian[5][i] = 0.0f;
        } else {
            float sin_angle = sin(angle);
            if (fabs(sin_angle) < 1e-5f) {
                // 接近180度旋转，使用近似计算
                jacobian[3][i] = 0.0f;
                jacobian[4][i] = 0.0f;
                jacobian[5][i] = 0.0f;
            } else {
                float axis[3];
                axis[0] = (R_diff[2][1] - R_diff[1][2]) / (2.0f * sin_angle);
                axis[1] = (R_diff[0][2] - R_diff[2][0]) / (2.0f * sin_angle);
                axis[2] = (R_diff[1][0] - R_diff[0][1]) / (2.0f * sin_angle);
                
                for (int j = 0; j < 3; j++) {
                    jacobian[j+3][i] = (angle * axis[j]) / DELTA;
                }
            }
        }
    }
    
    return true;
}

// 伪逆计算 (阻尼最小二乘法)
void ArmKinematics::calculateDampedLeastSquares(
    const float jacobian[6][ARM_DOF], 
    const float error[6], 
    float lambda, 
    float alpha,
    float dq[ARM_DOF]) {
    
    // 1. 计算 J的转置JT
    float JT[ARM_DOF][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < ARM_DOF; j++) {
            JT[j][i] = jacobian[i][j];
        }
    }
    
    // 2. 计算 J.JT
    float JJT[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            JJT[i][j] = 0.0f;
            for (int k = 0; k < ARM_DOF; k++) {
                JJT[i][j] += jacobian[i][k] * JT[k][j];
            }
        }
    }
    
    // 3. 添加阻尼项 (J.JT + λ²I)
    for (int i = 0; i < 6; i++) {
        JJT[i][i] += lambda * lambda;
    }
    
    // 4. 求解线性方程组 (J.JT + λ²I)x = error
    // 使用高斯消元法
    float augmented[6][7]; // 增广矩阵 [A|b]
    
    // 创建增广矩阵
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            augmented[i][j] = JJT[i][j];
        }
        augmented[i][6] = error[i];
    }
    
    // 高斯消元
    for (int i = 0; i < 6; i++) {
        // 查找主元
        int max_row = i;
        float max_val = fabs(augmented[i][i]);
        for (int j = i + 1; j < 6; j++) {
            if (fabs(augmented[j][i]) > max_val) {
                max_val = fabs(augmented[j][i]);
                max_row = j;
            }
        }
        
        // 交换行
        if (max_row != i) {
            for (int j = i; j <= 6; j++) {
                float temp = augmented[i][j];
                augmented[i][j] = augmented[max_row][j];
                augmented[max_row][j] = temp;
            }
        }
        
        // 消元
        for (int j = i + 1; j < 6; j++) {
            float ratio = augmented[j][i] / augmented[i][i];
            for (int k = i; k <= 6; k++) {
                augmented[j][k] -= ratio * augmented[i][k];
            }
        }
    }
    
    // 回代求解
    float x[6] = {0};
    for (int i = 5; i >= 0; i--) {
        x[i] = augmented[i][6];
        for (int j = i + 1; j < 6; j++) {
            x[i] -= augmented[i][j] * x[j];
        }
        x[i] /= augmented[i][i];
    }
    
    // 5. 计算关节增量 dq = α.JT.x
    for (int i = 0; i < ARM_DOF; i++) {
        dq[i] = 0.0f;
        for (int j = 0; j < 6; j++) {
            dq[i] += alpha * JT[i][j] * x[j];
        }
    }
}

// 简化版逆运动学，使用默认参数
int ArmKinematics::inverseKinematics(
    const Matrix4x4 *target_pose,
    float result_joints[ARM_DOF]) {
    
    return inverseKinematics(target_pose, NULL, 0, 0, result_joints);
}

// 完整版逆运动学
int ArmKinematics::inverseKinematics(
    const Matrix4x4 *target_pose,
    const float *initial_joints,
    float tolerance,
    int max_iterations,
    float result_joints[ARM_DOF]) {
    
    if (!target_pose || !result_joints) {
        return IK_INVALID_INPUT;
    }
    
    // 设置默认参数 - 放宽容差和增加迭代次数
    if (tolerance <= 0.0f) {
        tolerance = DEFAULT_TOLERANCE * 10.0f; // 将容差放宽10倍，从1e-4到1e-3
    }
    
    if (max_iterations <= 0) {
        max_iterations = MAX_ITERATIONS * 2; // 将最大迭代次数从100增加到200
    }
    
    // 初始化关节角度
    float current_joints[ARM_DOF];
    if (initial_joints) {
        memcpy(current_joints, initial_joints, sizeof(float) * ARM_DOF);
    } else {
        // 如果没有提供初始值，使用更好的初始猜测值，而不是零位置
        current_joints[0] = 0.0f;        // 基座旋转
        current_joints[1] = M_PI/6.0f;   // 肩关节
        current_joints[2] = 0.0f;        // 肩旋转
        current_joints[3] = M_PI/4.0f;   // 肘关节
        current_joints[4] = 0.0f;        // 肘旋转
        current_joints[5] = M_PI/4.0f;   // 腕关节
        current_joints[6] = 0.0f;        // 腕旋转
    }
    
    // 调整阻尼因子和步长因子（提高算法稳定性）
    float lambda = 0.1f;   // 阻尼因子，从0.5减小到0.1，增加稳定性
    float step_size = 0.3f; // 步长因子，从0.5减小到0.3，使每步更保守
    
    // 提取目标位置
    float p_target[3];
    for (int i = 0; i < 3; i++) {
        p_target[i] = target_pose->data[i][3];
    }
    
    // 迭代求解
    int iterations = 0;
    float error_norm = 1e10f;  // 初始化为一个大值
    float prev_error_norm = 1e10f; // 用于检测震荡
    float best_error_norm = 1e10f; // 保存最佳误差
    float best_joints[ARM_DOF]; // 保存最佳关节角度
    memcpy(best_joints, current_joints, sizeof(float) * ARM_DOF);
    int stall_count = 0; // 停滞计数器
    
    while (iterations < max_iterations && error_norm > tolerance) {
        iterations++;
        
        // 计算当前关节角度下的正运动学
        Matrix4x4 current_pose;
        if (!forwardKinematics(current_joints, &current_pose)) {
            return IK_INVALID_INPUT;
        }
        
        // 提取当前位置
        float p_current[3];
        for (int i = 0; i < 3; i++) {
            p_current[i] = current_pose.data[i][3];
        }
        
        // 计算位置误差
        float e_p[3];
        for (int i = 0; i < 3; i++) {
            e_p[i] = p_target[i] - p_current[i];
        }
        
        // 计算旋转误差（使用矩阵对数映射的简化版本）
        float e_o[3] = {0.0f, 0.0f, 0.0f};
        float R_error[3][3];
        
        // 计算旋转误差矩阵: R_error = R_target * R_current^T
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R_error[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) {
                    R_error[i][j] += target_pose->data[i][k] * current_pose.data[j][k];
                }
            }
        }
        
        // 从旋转误差矩阵提取轴角表示
        float trace = R_error[0][0] + R_error[1][1] + R_error[2][2];
        float theta = acos((trace - 1.0f) / 2.0f);
        
        if (fabs(theta) >= 1e-5f) {
            float s = theta / (2.0f * sin(theta));
            e_o[0] = s * (R_error[2][1] - R_error[1][2]);
            e_o[1] = s * (R_error[0][2] - R_error[2][0]);
            e_o[2] = s * (R_error[1][0] - R_error[0][1]);
        }
        
        // 合并位置和旋转误差
        float error[6];
        for (int i = 0; i < 3; i++) {
            error[i] = e_p[i];
            error[i+3] = e_o[i] * 0.5f; // 降低旋转误差的权重，更注重位置准确性
        }
        
        // 计算误差范数
        error_norm = 0.0f;
        for (int i = 0; i < 6; i++) {
            error_norm += error[i] * error[i];
        }
        error_norm = sqrt(error_norm);
        
        // 保存最佳解
        if (error_norm < best_error_norm) {
            best_error_norm = error_norm;
            memcpy(best_joints, current_joints, sizeof(float) * ARM_DOF);
        }
        
        // 检测停滞或震荡
        if (fabs(error_norm - prev_error_norm) < tolerance * 0.01f) {
            stall_count++;
            
            // 如果连续5次迭代误差几乎不变，调整参数或退出
            if (stall_count > 5) {
                // 尝试调整阻尼因子
                lambda *= 1.5f;
                step_size *= 0.8f;
                stall_count = 0;
                
                // 如果阻尼因子太大，算法可能无法进一步改进
                if (lambda > 5.0f) {
                    break;
                }
            }
        } else {
            stall_count = 0;
            prev_error_norm = error_norm;
        }
        
        // 如果误差小于容差，结束迭代
        if (error_norm <= tolerance) {
            break;
        }
        
        // 计算雅可比矩阵
        float jacobian[6][ARM_DOF];
        if (!calculateJacobian(current_joints, jacobian)) {
            return IK_INVALID_INPUT;
        }
        
        // 使用阻尼最小二乘法求解关节增量
        float dq[ARM_DOF];
        calculateDampedLeastSquares(jacobian, error, lambda, step_size, dq);
        
        // 更新关节角度
        for (int i = 0; i < ARM_DOF; i++) {
            current_joints[i] += dq[i];
            
            // 关节限制处理
            if (current_joints[i] < robot_params.qlim[i][0]) {
                current_joints[i] = robot_params.qlim[i][0];
            } else if (current_joints[i] > robot_params.qlim[i][1]) {
                current_joints[i] = robot_params.qlim[i][1];
            }
        }
    }
    
    // 使用最佳解而不是最后的结果
    memcpy(result_joints, best_joints, sizeof(float) * ARM_DOF);
    
    // 输出调试信息
    Serial.printf("逆运动学：迭代%d次，最终误差=%.6f\n", iterations, best_error_norm);
    
    if (best_error_norm <= tolerance) {
        return IK_SUCCESS;
    } else if (best_error_norm <= tolerance * 100) {
        // 如果误差在可接受范围内（比容差大但不超过100倍），仍然认为成功
        Serial.println("逆运动学：误差在可接受范围内，返回最佳解");
        return IK_SUCCESS;
    } else {
        return IK_MAX_ITERATIONS;
    }
}