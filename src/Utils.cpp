#include "Utils.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_MLX90614.h>
#include "Config.h"

// 引用外部变量
extern Adafruit_SSD1306 display;
extern Adafruit_MLX90614 mlx;
extern ArmStatus armStatus;

// ===== 舵机角度转换工具 =====

// 将角度值(0-240度)转换为舵机控制值(0-1000)
int Utils::angleToPulse(float angle) {
  // 确保角度在有效范围内
  angle = constrain(angle, 0, 240);
  // 线性映射
  return map(angle * 100, 0, 24000, 0, 1000);
}

// 将舵机控制值(0-1000)转换为角度值(0-240度)
float Utils::pulseToAngle(int pulse) {
  // 确保脉冲值在有效范围内
  pulse = constrain(pulse, 0, 1000);
  // 线性映射
  return map(pulse, 0, 1000, 0, 24000) / 100.0;
}

// 将轨迹规划角度(-120到+120度，0为中位)转换为舵机物理角度(0-240度)
float Utils::trajAngleToServoAngle(float trajAngle) {
  // 约束输入角度范围
  trajAngle = constrain(trajAngle, -120, 120);
  // 将-120到+120范围映射到0-240范围
  return trajAngle + 120;
}

// 将舵机物理角度(0-240度)转换为轨迹规划角度(-120到+120度，0为中位)
float Utils::servoAngleToTrajAngle(float servoAngle) {
  // 约束输入角度范围
  servoAngle = constrain(servoAngle, 0, 240);
  // 将0-240范围映射到-120到+120范围
  return servoAngle - 120;
}

// 直接将轨迹规划角度转换为舵机控制值
int Utils::trajAngleToPulse(float trajAngle) {
  float servoAngle = trajAngleToServoAngle(trajAngle);
  return angleToPulse(servoAngle);
}

// 直接将舵机控制值转换为轨迹规划角度
float Utils::pulseToTrajAngle(int pulse) {
  float servoAngle = pulseToAngle(pulse);
  return servoAngleToTrajAngle(servoAngle);
}

// 简化版根据移动距离计算移动时间
int Utils::calculateMoveTime(int distance) {
  // 使用简单线性关系，减少计算复杂度
  if (distance < 50) {
    return 1; // 短距离用固定短时间
  } else if (distance < 200) {
    return 300; // 中距离用固定中等时间
  } else {
    return 600; // 长距离用固定长时间
  }
}

// 计算自动复位的时间
int Utils::calculateAutoResetTime() {
  return 500; // 简单地返回一个固定值
}

// ===== 旋转矩阵工具 =====

// 创建X轴旋转矩阵
Matrix3d Utils::rotationX(double angle) {
  Matrix3d R = Matrix3d::Zero();
  double c = cos(angle);
  double s = sin(angle);
  
  R(0,0) = 1.0;
  R(1,1) = c;
  R(1,2) = -s;
  R(2,1) = s;
  R(2,2) = c;
  
  return R;
}

// 创建Y轴旋转矩阵
Matrix3d Utils::rotationY(double angle) {
  Matrix3d R = Matrix3d::Zero();
  double c = cos(angle);
  double s = sin(angle);
  
  R(0,0) = c;
  R(0,2) = s;
  R(1,1) = 1.0;
  R(2,0) = -s;
  R(2,2) = c;
  
  return R;
}

// 创建Z轴旋转矩阵
Matrix3d Utils::rotationZ(double angle) {
  Matrix3d R = Matrix3d::Zero();
  double c = cos(angle);
  double s = sin(angle);
  
  R(0,0) = c;
  R(0,1) = -s;
  R(1,0) = s;
  R(1,1) = c;
  R(2,2) = 1.0;
  
  return R;
}

// 欧拉角转旋转矩阵 (ZYX顺序)
Matrix3d Utils::eulerToRotationMatrix(double roll, double pitch, double yaw) {
  // 使用ZYX顺序的欧拉角创建旋转矩阵 (yaw-pitch-roll)
  Matrix3d Rx = rotationX(roll);
  Matrix3d Ry = rotationY(pitch);
  Matrix3d Rz = rotationZ(yaw);
  
  // 组合旋转 (R = Rz * Ry * Rx)
  return Rz * Ry * Rx;
}

// 旋转矩阵转欧拉角 (ZYX顺序)
bool Utils::rotationMatrixToEuler(const Matrix3d& R, double& roll, double& pitch, double& yaw) {
  // 确保输入是有效的旋转矩阵
  if (!isValidRotationMatrix(R)) {
    return false;
  }
  
  // 提取欧拉角 (ZYX顺序)
  pitch = atan2(-R(2,0), sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
  
  // 处理万向锁问题 (Gimbal lock)
  if (fabs(pitch) > 0.99 * M_PI / 2) {
    // 接近万向锁，yaw和roll不唯一 (选择yaw=0)
    yaw = 0;
    roll = atan2(R(0,1), R(1,1));
  } else {
    yaw = atan2(R(1,0), R(0,0));
    roll = atan2(R(2,1), R(2,2));
  }
  
  return true;
}

// 检查矩阵是否为有效的旋转矩阵
bool Utils::isValidRotationMatrix(const Matrix3d& R, double tolerance) {
  // 检查矩阵的行列式是否接近1
  if (fabs(R.determinant() - 1.0) > tolerance) {
    return false;
  }
  
  // 检查R * R^T是否接近单位矩阵
  Matrix3d I = R * R.transpose();
  Matrix3d Identity = Matrix3d::Identity();
  
  return (I - Identity).norm() < tolerance;
}

// 正交化旋转矩阵 (Gram-Schmidt正交化)
void Utils::orthogonalizeRotationMatrix(Matrix3d& R) {
  // 提取三个列向量
  Vector3d x = R.col(0);
  Vector3d y = R.col(1);
  Vector3d z = R.col(2);
  
  // 对第一列进行归一化
  x.normalize();
  
  // 确保y正交于x
  y = y - x * (x.dot(y));
  y.normalize();
  
  // 计算z为x和y的叉积 (确保右手系)
  z = x.cross(y);
  
  // 更新旋转矩阵
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
}

// 在两个旋转矩阵之间进行球面线性插值
bool Utils::interpolateRotation(const Matrix3d& R_start, const Matrix3d& R_end, 
                              double t, Matrix3d& R_interp) {
  // 确保输入矩阵是有效的旋转矩阵
  if (!isValidRotationMatrix(R_start) || !isValidRotationMatrix(R_end)) {
    return false;
  }
  
  // 创建正交化的拷贝
  Matrix3d Rs_orthogonal = R_start;
  Matrix3d Re_orthogonal = R_end;
  
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
    dot = std::min(std::max(dot, -1.0), 1.0); // 限制在[-1,1]范围内
    double omega = acos(dot);
    
    if (fabs(omega) < 1e-10) {
      // 如果角度几乎为0，使用线性插值
      q_interp.coeffs() = (1.0-t) * q_start.coeffs() + t * q_end.coeffs();
      q_interp.normalize();
    } else {
      // 球面线性插值 (SLERP)
      double s0 = sin((1.0-t) * omega) / sin(omega);
      double s1 = sin(t * omega) / sin(omega);
      
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

// ===== 其他辅助计算函数 =====

// 计算五次多项式系数
void Utils::computeQuinticCoeff(double q0, double qf, double T, VectorXd& coeffs) {
  coeffs.resize(6);
  
  // 五次多项式系数计算
  coeffs(0) = q0;
  coeffs(1) = 0; // 零初始速度
  coeffs(2) = 0; // 零初始加速度
  coeffs(3) = 10*(qf-q0)/(T*T*T);
  coeffs(4) = -15*(qf-q0)/(T*T*T*T);
  coeffs(5) = 6*(qf-q0)/(T*T*T*T*T);
}

// 计算五次多项式位置
double Utils::quinticPosition(double t, const VectorXd& coeffs) {
  return coeffs(0) + coeffs(1)*t + coeffs(2)*t*t + 
         coeffs(3)*t*t*t + coeffs(4)*t*t*t*t + coeffs(5)*t*t*t*t*t;
}

// 计算五次多项式速度
double Utils::quinticVelocity(double t, const VectorXd& coeffs) {
  return coeffs(1) + 2*coeffs(2)*t + 3*coeffs(3)*t*t + 
         4*coeffs(4)*t*t*t + 5*coeffs(5)*t*t*t*t;
}

// 计算五次多项式加速度
double Utils::quinticAcceleration(double t, const VectorXd& coeffs) {
  return 2*coeffs(2) + 6*coeffs(3)*t + 
         12*coeffs(4)*t*t + 20*coeffs(5)*t*t*t;
}

// 计算最小运动时间
double Utils::calculateMinimumTime(const VectorXd& q_start, const VectorXd& q_end) {
  // 简单版本：根据最大关节差异计算最小时间
  double maxDiff = 0;
  for (int i = 0; i < q_start.size(); i++) {
    double diff = fabs(q_end(i) - q_start(i));
    if (diff > maxDiff) maxDiff = diff;
  }
  
  // 根据最大差异和关节速度限制计算时间
  double maxSpeed = 0.5; // 假设的最大关节速度 (rad/s)
  return maxDiff / maxSpeed;
}
