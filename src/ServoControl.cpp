#include "ServoControl.h"
#include "ActionGroup.h"

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 简化状态追踪，仅保留基本信息
ServoState servoStates[8]; // 7个关节 + 1个夹持器

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 记录最后一次舵机移动预计完成的时间
unsigned long servoLastMoveEndTime = 0;

// 定义用于跟踪执行状态的变量
bool isExecuting_ = false;  // 添加这一行来定义变量

// 角度转换辅助函数 - 用于240度舵机
// 将角度值(0-240度)转换为舵机控制值(0-1000)
int angleToPulse(float angle) {
  // 约束角度范围
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  // 线性映射：角度范围(0-240) -> 控制值范围(0-1000)
  return map(angle * 10, SERVO_MIN_ANGLE * 10, SERVO_MAX_ANGLE * 10, 0, 1000);
}

// 将舵机控制值(0-1000)转换为角度值(0-240度)
float pulseToAngle(int pulse) {
  // 约束脉冲范围
  pulse = constrain(pulse, 0, 1000);
  // 线性映射：控制值范围(0-1000) -> 角度范围(0-240)
  return map(pulse * 10, 0, 1000 * 10, SERVO_MIN_ANGLE * 10, SERVO_MAX_ANGLE * 10) / 10.0;
}

// 将轨迹规划角度(-120到+120度，0为中位)转换为舵机物理角度(0-240度)
float trajAngleToServoAngle(float trajAngle) {
  // 轨迹规划角度范围：-120到+120度，0为中位
  // 舵机物理角度范围：0-240度，120为中位
  // 简单转换公式：舵机角度 = 轨迹角度 + 120
  float servoAngle = trajAngle + 120.0f;
  
  // 约束到舵机的物理限制范围
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  return servoAngle;
}

// 将舵机物理角度(0-240度)转换为轨迹规划角度(-120到+120度，0为中位)
float servoAngleToTrajAngle(float servoAngle) {
  // 舵机物理角度范围：0-240度，120为中位
  // 轨迹规划角度范围：-120到+120度，0为中位
  // 简单转换公式：轨迹角度 = 舵机角度 - 120
  float trajAngle = servoAngle - 120.0f;
  
  // 约束到轨迹规划的角度范围
  trajAngle = constrain(trajAngle, -120.0f, 120.0f);
  
  return trajAngle;
}

// 直接将轨迹规划角度转换为舵机控制值
int trajAngleToPulse(float trajAngle, int servo_id) {
  // 准备一个数组存储所有关节的方向系数
  static const int jointDirections[7] = {
    JOINT_DIRECTION_1, JOINT_DIRECTION_2, JOINT_DIRECTION_3, 
    JOINT_DIRECTION_4, JOINT_DIRECTION_5, JOINT_DIRECTION_6, 
    JOINT_DIRECTION_7
  };
  
  // 查找舵机ID对应的关节索引
  int jointIndex = -1;
  for (int i = 0; i < 7; i++) {
    if (servo_id == jointServos[i]) {
      jointIndex = i;
      break;
    }
  }
  
  // 如果找到匹配的关节索引，应用方向系数
  if (jointIndex >= 0) {
    trajAngle = trajAngle * jointDirections[jointIndex];
  }
  
  // 先转换为舵机物理角度
  float servoAngle = trajAngleToServoAngle(trajAngle);
  
  // 再转换为舵机控制值
  return angleToPulse(servoAngle);
}

// 重载原始函数，无参数版本，用于兼容现有代码
int trajAngleToPulse(float trajAngle) {
  // 由于不知道是哪个关节，我们无法应用方向系数
  // 但为了兼容性保留此函数
  // 先转换为舵机物理角度
  float servoAngle = trajAngleToServoAngle(trajAngle);
  
  // 再转换为舵机控制值
  return angleToPulse(servoAngle);
}

// 直接将舵机控制值转换为轨迹规划角度
float pulseToTrajAngle(int pulse) {
  // 先转换为舵机物理角度
  float servoAngle = pulseToAngle(pulse);
  
  // 再转换为轨迹规划角度
  return servoAngleToTrajAngle(servoAngle);
}

// 初始化舵机
void initServo() {
  Serial.println("初始化舵机控制...");
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);  // 舵机控制
  
  // 创建舵机批量控制数组
  LobotServo servoArray[8]; // 7个关节 + 1个夹持器
  
  // 设置所有舵机到中心位置
  for (int i = 0; i < 7; i++) {
    servoArray[i].ID = jointServos[i];
    servoArray[i].Position = CENTER_POSITION;
    servoStates[i].currentPos = CENTER_POSITION;
    armStatus.joints[i] = CENTER_POSITION;
  }
  
  // 设置夹持器到中心位置
  servoArray[7].ID = gripperServo;
  servoArray[7].Position = CENTER_POSITION;
  servoStates[7].currentPos = CENTER_POSITION;
  armStatus.gripper = CENTER_POSITION;
  
  // 使用自动计算的复位时间（与X键模式相同）
  int moveTime = calculateAutoResetTime();
  
  Serial.print("初始化舵机使用自动复位时间: ");
  Serial.print(moveTime);
  Serial.println("ms");
  
  // 使用移动多个舵机的函数 - 它会自动应用方向系数
  moveMultipleServos(servoArray, 8, moveTime);
  
  // 标记为已更新
  armStatus.updated = true;
  
  Serial.println("所有舵机已同时初始化到中心位置");
}

// 简化版根据移动距离计算移动时间
int calculateMoveTime(int distance) {
  // 使用简单线性关系，减少计算复杂度
  if (distance < 50) {
    return 1; // 短距离用固定短时间
  } else if (distance < 200) {
    return 300; // 中距离用固定中等时间
  } else {
    return 600; // 长距离用固定长时间
  }
}

// 获取舵机索引 - 简化版本
int getServoIndex(int servoId) {
  if (servoId == gripperServo) return 7;
  if (servoId >= 1 && servoId <= 7) return servoId - 1;
  return -1; // 无效ID
}

// 直接控制舵机 - 简化版并增加完成时间计算
void moveServo(int servoId, int position) {
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 应用方向系数(仅对关节舵机)
  static const int jointDirections[7] = {
    JOINT_DIRECTION_1, JOINT_DIRECTION_2, JOINT_DIRECTION_3, 
    JOINT_DIRECTION_4, JOINT_DIRECTION_5, JOINT_DIRECTION_6, 
    JOINT_DIRECTION_7
  };
  
  // 仅对关节舵机处理方向系数
  if (servoId >= 1 && servoId <= 7) {
    int jointIndex = servoId - 1;
    if (jointDirections[jointIndex] == -1) {
      // 如果方向系数为-1，对位置值取反
      // 舵机控制值范围是0-1000，中心点是500
      position = 1000 - position;
    }
  }
  
  // 打印舵机移动信息
  Serial.print("舵机移动: ID=");
  Serial.print(servoId);
  Serial.print(", 当前位置=");
  Serial.print(servoStates[index].currentPos);
  Serial.print(", 目标位置=");
  Serial.print(position);
  
  // 计算移动时间
  int distance = abs(position - servoStates[index].currentPos);
  int moveTime = calculateMoveTime(distance);
  
  Serial.print(", 距离=");
  Serial.print(distance);
  Serial.print(", 时间=");
  Serial.print(moveTime);
  Serial.println("ms");
  
  // 发送舵机命令
  servos.moveServo(servoId, position, moveTime);
  
  // 更新当前位置记录
  servoStates[index].currentPos = position;
  
  // 更新最后一次舵机移动预计完成的时间
  // 添加额外的50ms缓冲，确保命令完全执行
  servoLastMoveEndTime = millis() + moveTime + 0;
  
  // 记录动作组（如果正在记录）
  if (isRecording) {
    recordAction(servoId, position, moveTime);
  }
}

// 复位所有舵机到中心位置
void resetServos() {
  Serial.println("复位所有舵机到中心位置");
  
  // 使用自动计算的复位时间
  int moveTime = calculateAutoResetTime();
  
  Serial.print("使用自动复位时间: ");
  Serial.print(moveTime);
  Serial.println("ms");
  
  // 创建舵机批量控制数组
  LobotServo servoArray[8]; // 7个关节 + 1个夹持器
  
  // 设置所有舵机到中心位置
  for (int i = 0; i < 7; i++) {
    servoArray[i].ID = jointServos[i];
    servoArray[i].Position = CENTER_POSITION;
    servoStates[i].currentPos = CENTER_POSITION;
    armStatus.joints[i] = CENTER_POSITION;
  }
  
  // 设置夹持器到中心位置
  servoArray[7].ID = gripperServo;
  servoArray[7].Position = CENTER_POSITION;
  servoStates[7].currentPos = CENTER_POSITION;
  armStatus.gripper = CENTER_POSITION;
  
  // 使用移动多个舵机的函数 - 它会自动应用方向系数
  moveMultipleServos(servoArray, 8, moveTime);
  
  // 标记为已更新
  armStatus.updated = true;
  
  // 如果正在记录动作组，则记录复位动作
  if (isRecording) {
    // 使用特殊ID表示这是一个复位命令
    recordAction(RESET_ACTION_ID, 0, moveTime);
  }
}

// 自动计算适合的复位时间
int calculateAutoResetTime() {
  int maxDistance = 0;
  int totalDistance = 0;
  int activeServoCount = 0;
  
  // 计算所有舵机到中心位置的最大距离和总距离
  for (int i = 0; i < 7; i++) {
    int distance = abs(CENTER_POSITION - servoStates[i].currentPos);
    if (distance > POSITION_DEADZONE) {
      maxDistance = max(maxDistance, distance);
      totalDistance += distance;
      activeServoCount++;
    }
  }
  
  // 计算夹持器到中心的距离
  int gripperDistance = abs(CENTER_POSITION - servoStates[7].currentPos);
  if (gripperDistance > POSITION_DEADZONE) {
    maxDistance = max(maxDistance, gripperDistance);
    totalDistance += gripperDistance;
    activeServoCount++;
  }
  
  // 如果没有舵机需要移动，返回最小时间
  if (activeServoCount == 0) {
    return AUTO_RESET_TIME_MIN;
  }
  
  // 计算平均距离
  int avgDistance = totalDistance / activeServoCount;
  
  // 基于最大距离和平均距离动态调整复位时间
  int resetTime;
  
  if (maxDistance > RESET_DISTANCE_THRESHOLD) {
    // 对于大距离移动，使用更长的时间以确保平稳
    resetTime = AUTO_RESET_TIME_MIN + (maxDistance * RESET_TIME_FACTOR);
  } else {
    // 对于小距离移动，使用更快的复位时间
    resetTime = AUTO_RESET_TIME_MIN + (avgDistance * 1.2);
  }
  
  // 确保复位时间在合理范围内
  resetTime = constrain(resetTime, AUTO_RESET_TIME_MIN, AUTO_RESET_TIME_MAX);
  
  Serial.print("自动复位时间: ");
  Serial.print(resetTime);
  Serial.print("ms, 最大距离: ");
  Serial.print(maxDistance);
  Serial.print(", 平均距离: ");
  Serial.println(avgDistance);
  
  return resetTime;
}

// 处理舵机控制 - 现在支持在录制模式下使用手柄控制
void handleServoControl(int servoId, int position) {
  // 如果正在播放动作组，则不处理外部舵机控制命令
  if (isPlaying) {
    return;
  }
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 直接移动舵机
  moveServo(servoId, position);
  
  // 更新全局状态
  if (servoId == gripperServo) {
    armStatus.gripper = position;
  } else if (index >= 0 && index < 7) {
    armStatus.joints[index] = position;
  }
  
  // 标记为已更新
  armStatus.updated = true;
  armStatus.lastCommandTime = millis();
  
  // 注意：moveServo函数内部已经处理了动作组记录
  // 不需要在这里再次调用recordAction
}

// 添加批量控制多个舵机的功能
void moveMultipleServos(LobotServo servoArray[], int servoCount, int moveTime) {
  // 获取方向系数数组
  static const int jointDirections[7] = {
    JOINT_DIRECTION_1, JOINT_DIRECTION_2, JOINT_DIRECTION_3, 
    JOINT_DIRECTION_4, JOINT_DIRECTION_5, JOINT_DIRECTION_6, 
    JOINT_DIRECTION_7
  };
  
  // 创建临时数组用于发送命令
  LobotServo tempServoArray[servoCount];
  
  // 复制并应用方向系数
  for (int i = 0; i < servoCount; i++) {
    tempServoArray[i].ID = servoArray[i].ID;
    int position = servoArray[i].Position;
    
    // 仅对关节舵机处理方向系数
    if (servoArray[i].ID >= 1 && servoArray[i].ID <= 7) {
      int jointIndex = servoArray[i].ID - 1;
      if (jointDirections[jointIndex] == -1) {
        // 如果方向系数为-1，对位置值取反
        // 舵机控制值范围是0-1000，中心点是500
        position = 1000 - position;
      }
    }
    
    tempServoArray[i].Position = position;
  }
  
  // 批量发送命令 - 同时控制多个舵机
  servos.moveServos(tempServoArray, servoCount, moveTime);
  
  // 更新舵机状态 - 使用原始值（未经方向调整的）存储
  for (int i = 0; i < servoCount; i++) {
    int index = getServoIndex(servoArray[i].ID);
    if (index >= 0) {
      servoStates[index].currentPos = servoArray[i].Position;
      
      // 更新全局状态
      if (servoArray[i].ID == gripperServo) {
        armStatus.gripper = servoArray[i].Position;
      } else if (index >= 0 && index < 7) {
        armStatus.joints[index] = servoArray[i].Position;
      }
    }
  }
  
  // 更新最后一次舵机移动预计完成的时间
  servoLastMoveEndTime = millis() + moveTime;
  
  // 标记为已更新
  armStatus.updated = true;
  armStatus.lastCommandTime = millis();
  
  // 如果正在记录动作组，则记录这些舵机的移动
  if (isRecording) {
    for (int i = 0; i < servoCount; i++) {
      recordAction(servoArray[i].ID, servoArray[i].Position, moveTime);
    }
  }
  
  Serial.print("批量控制 ");
  Serial.print(servoCount);
  Serial.print(" 个舵机，时间：");
  Serial.println(moveTime);
}

// 检查所有舵机是否就绪（没有正在执行的命令）
bool isServosReady() {
    return !isExecuting_ && !isPlaying && 
           (millis() >= servoLastMoveEndTime);
}

// 获取当前关节角度（度）
void getCurrentJointAngles(float angles[7]) {
    for(int i = 0; i < 7; i++) {
        angles[i] = pulseToAngle(armStatus.joints[i]);
    }
}

// 设置关节角度（度）
void setJointAngles(const float angles[7], int moveTime) {
    LobotServo servoArray[7];
    
    // 准备一个数组存储所有关节的方向系数
    static const int jointDirections[7] = {
        JOINT_DIRECTION_1, JOINT_DIRECTION_2, JOINT_DIRECTION_3, 
        JOINT_DIRECTION_4, JOINT_DIRECTION_5, JOINT_DIRECTION_6, 
        JOINT_DIRECTION_7
    };
    
    // 将角度转换为控制值，应用方向系数
    for(int i = 0; i < 7; i++) {
        servoArray[i].ID = jointServos[i];
        // 应用方向系数
        float adjustedAngle = angles[i];
        if (jointDirections[i] == -1) {
            // 如果方向系数为-1，对角度取反
            // 由于舵机角度是0-240度范围，我们需要围绕中心点120度进行反转
            adjustedAngle = 240 - adjustedAngle;
        }
        servoArray[i].Position = angleToPulse(adjustedAngle);
    }
    
    // 批量控制所有舵机
    moveMultipleServos(servoArray, 7, moveTime);
}

// 获取当前关节的轨迹规划角度
void getCurrentTrajAngles(float angles[7]) {
  float servoAngles[7];
  
  // 先获取当前舵机物理角度
  getCurrentJointAngles(servoAngles);
  
  // 转换为轨迹规划角度
  for(int i = 0; i < 7; i++) {
    angles[i] = servoAngleToTrajAngle(servoAngles[i]);
  }
}

// 设置关节的轨迹规划角度
void setTrajAngles(const float trajAngles[7], int moveTime) {
  LobotServo servoArray[7];
  
  // 应用方向系数并转换为舵机控制值
  for(int i = 0; i < 7; i++) {
    // 直接使用带有servo_id参数的trajAngleToPulse函数，应用关节方向系数
    servoArray[i].ID = jointServos[i];
    servoArray[i].Position = trajAngleToPulse(trajAngles[i], jointServos[i]);
  }
  
  // 批量控制所有舵机
  moveMultipleServos(servoArray, 7, moveTime);
}

