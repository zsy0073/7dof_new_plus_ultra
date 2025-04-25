#include "RotationUtils.h"
#include <cmath>

bool RotationUtils::interpolateRotation(const Matrix3d& R_start, const Matrix3d& R_end, 
                                      double t, Matrix3d& R_interp) {
    // 标准化参数t，确保在[0,1]范围内
    t = std::max(0.0, std::min(1.0, t));
    
    // 检查输入矩阵是否有效
    if (!R_start.allFinite() || !R_end.allFinite()) {
        R_interp = Matrix3d::Identity();
        return false;
    }
    
    // 创建工作副本，确保输入矩阵为正交矩阵
    Matrix3d Rs_orthogonal = R_start;
    Matrix3d Re_orthogonal = R_end;
    
    // 正交化矩阵
    orthogonalizeRotationMatrix(Rs_orthogonal);
    orthogonalizeRotationMatrix(Re_orthogonal);
    
    // 从旋转矩阵构造四元数
    Quaterniond q_start(Rs_orthogonal);
    Quaterniond q_end(Re_orthogonal);
    
    // 归一化四元数
    q_start.normalize();
    q_end.normalize();
    
    // 确保走最短路径
    if(q_start.dot(q_end) < 0) {
        q_end.coeffs() = -q_end.coeffs();
    }
    
    // 执行球面线性插值
    Quaterniond q_interp;
    
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
    return R_interp.allFinite();
}

Matrix3d RotationUtils::eulerToRotationMatrix(double roll, double pitch, double yaw) {
    // 使用ZYX顺序的欧拉角创建旋转矩阵 (yaw-pitch-roll)
    Matrix3d Rx = rotationX(roll);
    Matrix3d Ry = rotationY(pitch);
    Matrix3d Rz = rotationZ(yaw);
    
    // 组合旋转 (R = Rz * Ry * Rx)
    return Rz * Ry * Rx;
}

bool RotationUtils::rotationMatrixToEuler(const Matrix3d& R, double& roll, double& pitch, double& yaw) {
    if (!isValidRotationMatrix(R)) {
        return false;
    }
    
    // 从旋转矩阵提取欧拉角 (ZYX顺序)
    // 处理万向节锁问题 (Gimbal lock)
    double threshold = 0.99999;
    
    if (fabs(R(2,0)) > threshold) {
        // 万向节锁情况
        yaw = atan2(R(0,1), R(0,2));
        pitch = M_PI/2 * (R(2,0) < 0 ? 1 : -1);
        roll = 0;
    } else {
        pitch = -asin(R(2,0));
        roll = atan2(R(2,1)/cos(pitch), R(2,2)/cos(pitch));
        yaw = atan2(R(1,0)/cos(pitch), R(0,0)/cos(pitch));
    }
    
    return true;
}

bool RotationUtils::isValidRotationMatrix(const Matrix3d& R, double tolerance) {
    // 检查矩阵是否有效 (有限值)
    if (!R.allFinite()) {
        return false;
    }
    
    // 检查正交性 (R*R^T = I)
    Matrix3d I = R * R.transpose();
    if ((I - Matrix3d::Identity()).norm() > tolerance) {
        return false;
    }
    
    // 检查行列式是否为1 (保证是旋转矩阵而非反射矩阵)
    if (fabs(R.determinant() - 1.0) > tolerance) {
        return false;
    }
    
    return true;
}

void RotationUtils::orthogonalizeRotationMatrix(Matrix3d& R) {
    // 使用SVD分解正交化旋转矩阵
    Eigen::JacobiSVD<Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    
    // 确保行列式为正
    if (R.determinant() < 0) {
        Matrix3d V = svd.matrixV();
        V.col(2) = -V.col(2);
        R = svd.matrixU() * V.transpose();
    }
}

Matrix3d RotationUtils::rotationX(double angle) {
    Matrix3d R = Matrix3d::Identity();
    double c = cos(angle);
    double s = sin(angle);
    R(1,1) = c;
    R(1,2) = -s;
    R(2,1) = s;
    R(2,2) = c;
    return R;
}

Matrix3d RotationUtils::rotationY(double angle) {
    Matrix3d R = Matrix3d::Identity();
    double c = cos(angle);
    double s = sin(angle);
    R(0,0) = c;
    R(0,2) = s;
    R(2,0) = -s;
    R(2,2) = c;
    return R;
}

Matrix3d RotationUtils::rotationZ(double angle) {
    Matrix3d R = Matrix3d::Identity();
    double c = cos(angle);
    double s = sin(angle);
    R(0,0) = c;
    R(0,1) = -s;
    R(1,0) = s;
    R(1,1) = c;
    return R;
}