#include "ServoControl.h"
#include "ActionGroup.h"  // 添加动作组头文件

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 简化状态追踪，仅保留基本信息
ServoState servoStates[8]; // 7个关节 + 1个夹持器

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 记录最后一次舵机移动预计完成的时间
unsigned long servoLastMoveEndTime = 0;

// 初始化舵机
void initServo() {
  Serial.println("初始化舵机控制...");
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);  // 舵机控制
  
  // 初始化所有舵机到中心位置
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], CENTER_POSITION, SERVO_MOVE_TIME);
    servoStates[i].currentPos = CENTER_POSITION;
    delay(20); // 短暂延时确保命令发送，减少延时时间
  }
  
  // 初始化夹持器
  servos.moveServo(gripperServo, CENTER_POSITION, SERVO_MOVE_TIME);
  servoStates[7].currentPos = CENTER_POSITION;
  
  Serial.println("所有舵机已初始化到中心位置");
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

// 直接控制舵机 - 增加额外的验证和调试信息
void moveServo(int servoId, int position) {
  // 防止无效舵机ID
  if (servoId <= 0 || servoId > 8) {
    Serial.print("无效舵机ID: ");
    Serial.println(servoId);
    return;
  }
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) {
    Serial.print("无效的舵机索引，ID: ");
    Serial.println(servoId);
    return;
  }
  
  // 获取当前位置
  int currentPos = servoStates[index].currentPos;
  
  // 打印舵机移动前的详细信息
  Serial.print("准备移动舵机 - ID=");
  Serial.print(servoId);
  Serial.print(", 索引=");
  Serial.print(index);
  Serial.print(", 当前位置=");
  Serial.print(currentPos);
  Serial.print(", 请求位置=");
  Serial.println(position);
  
  // 验证当前位置是否在合理范围内
  if (currentPos < 0 || currentPos > 1000) {
    Serial.print("舵机当前位置异常，重置为500: ");
    Serial.println(currentPos);
    servoStates[index].currentPos = 500;
    currentPos = 500;
  }
  
  // 约束目标位置范围
  int originalPosition = position;
  position = constrain(position, 0, 1000);
  
  if (originalPosition != position) {
    Serial.print("位置值已约束: 原值=");
    Serial.print(originalPosition);
    Serial.print(", 约束后=");
    Serial.println(position);
  }
  
  // 计算目标位置与当前位置的距离
  int distance = abs(position - currentPos);
  
  // 如果距离异常大（超过预期的最大变化），可能是命令错误
  if (distance > 500) {
    Serial.print("警告: 检测到异常大的位置变化! 距离=");
    Serial.print(distance);
    Serial.print(", 当前位置=");
    Serial.print(currentPos);
    Serial.print(", 目标位置=");
    Serial.println(position);
    
    // 可以选择在这里忽略这个命令或者应用平滑处理
    // 如果想忽略此命令，取消下面的注释
    // Serial.println("由于异常变化太大，忽略此命令");
    // return;
  }
  
  // 计算移动时间
  int moveTime = calculateMoveTime(distance);
  
  // 打印最终的舵机移动信息
  Serial.print("舵机移动: ID=");
  Serial.print(servoId);
  Serial.print(", 当前位置=");
  Serial.print(currentPos);
  Serial.print(", 目标位置=");
  Serial.print(position);
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
  servoLastMoveEndTime = millis() + moveTime;
  
  // 记录动作组（如果正在记录）
  if (isRecording) {
    recordAction(servoId, position, moveTime);
  }
}

// 复位所有舵机到中心位置
void resetServos() {
  Serial.println("复位所有舵机到中心位置");
  
  // 找出距离中心最远的舵机，计算最长的移动时间
  int maxDistance = 0;
  for (int i = 0; i < 7; i++) {
    int distance = abs(CENTER_POSITION - servoStates[i].currentPos);
    maxDistance = max(maxDistance, distance);
  }
  int distance = abs(CENTER_POSITION - servoStates[7].currentPos);
  maxDistance = max(maxDistance, distance);
  
  int moveTime = calculateMoveTime(maxDistance);
  
  // 同时移动所有舵机到中心位置
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], CENTER_POSITION, moveTime);
    servoStates[i].currentPos = CENTER_POSITION;
    armStatus.joints[i] = CENTER_POSITION;
  }
  
  // 移动夹持器
  servos.moveServo(gripperServo, CENTER_POSITION, moveTime);
  servoStates[7].currentPos = CENTER_POSITION;
  armStatus.gripper = CENTER_POSITION;
  
  // 更新最后一次舵机移动预计完成的时间
  // 添加额外的100ms缓冲，确保复位完全执行
  servoLastMoveEndTime = millis() + moveTime + 100;
  
  // 标记为已更新
  armStatus.updated = true;
  
  // 如果正在记录动作组，则记录复位动作
  if (isRecording) {
    // 使用特殊ID表示这是一个复位命令
    recordAction(RESET_ACTION_ID, 0, moveTime);
  }
}

// 处理舵机控制 - 简化版
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
}

// 移除updateServoStatus函数，简化控制流程
