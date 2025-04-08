#include "ServoControl.h"

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 初始化舵机状态数组
ServoState servoStates[8]; // 7个关节 + 1个夹持器

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 初始化舵机
void initServo() {
  Serial.println("初始化舵机控制...");
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);  // 舵机控制
  
  // 初始化所有舵机到中心位置
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], CENTER_POSITION, SERVO_MOVE_TIME);
    servoStates[i].currentPos = CENTER_POSITION;
    servoStates[i].targetPos = CENTER_POSITION;
    servoStates[i].isMoving = false;
    delay(50); // 短暂延时确保命令发送
  }
  
  // 初始化夹持器
  servos.moveServo(gripperServo, CENTER_POSITION, SERVO_MOVE_TIME);
  servoStates[7].currentPos = CENTER_POSITION;
  servoStates[7].targetPos = CENTER_POSITION;
  servoStates[7].isMoving = false;
  
  Serial.println("所有舵机已初始化到中心位置");
}

// 根据移动距离计算合适的移动时间
int calculateMoveTime(int distance) {
  // 根据距离线性调整移动时间
  int moveTime = map(distance, 0, 500, MIN_MOVE_TIME, MAX_MOVE_TIME);
  return constrain(moveTime, MIN_MOVE_TIME, MAX_MOVE_TIME);
}

// 获取舵机索引
int getServoIndex(int servoId) {
  if (servoId == gripperServo) {
    return 7;
  }
  
  for (int i = 0; i < 7; i++) {
    if (jointServos[i] == servoId) {
      return i;
    }
  }
  
  return -1; // 无效ID
}

// 直接移动舵机到指定位置
void moveServoDirectly(int servoId, int position, int moveTime) {
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 发送舵机命令
  servos.moveServo(servoId, position, moveTime);
  
  // 更新舵机状态
  servoStates[index].currentPos = position;
  servoStates[index].targetPos = position;
  servoStates[index].lastMoveTime = millis();
  servoStates[index].isMoving = false; // 标记为不在移动状态，因为这是直接控制
}

// 平滑移动舵机到指定位置
void moveServoSmoothly(int servoId, int targetPos) {
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 获取当前位置
  int currentPos = servoStates[index].currentPos;
  
  // 约束目标位置范围
  targetPos = constrain(targetPos, 0, 1000);
  
  // 如果变化太小，忽略此次命令
  if (abs(targetPos - currentPos) <= POSITION_DEADZONE) {
    return;
  }
  
  // 计算移动距离和时间
  int distance = abs(targetPos - currentPos);
  int moveTime = calculateMoveTime(distance);
  
  // 发送舵机命令
  servos.moveServo(servoId, targetPos, moveTime);
  
  // 更新舵机状态
  servoStates[index].targetPos = targetPos;
  servoStates[index].lastMoveTime = millis();
  servoStates[index].isMoving = true;
  
  // 延迟一小段时间等待命令执行开始
  delay(5);
}

// 更新舵机状态
void updateServoStatus() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 8; i++) {
    if (servoStates[i].isMoving) {
      // 检查移动是否应该完成
      int moveDistance = abs(servoStates[i].targetPos - servoStates[i].currentPos);
      int estimatedMoveTime = calculateMoveTime(moveDistance);
      
      if (currentTime - servoStates[i].lastMoveTime >= estimatedMoveTime) {
        // 移动应该已完成，更新当前位置
        servoStates[i].currentPos = servoStates[i].targetPos;
        servoStates[i].isMoving = false;
      }
    }
  }
}

// 复位所有舵机到中心位置
void resetServos() {
  Serial.println("复位所有舵机到中心位置");
  
  // 计算移动时间 - 找到离中心位置最远的舵机
  int maxDistance = 0;
  for (int i = 0; i < 7; i++) {
    maxDistance = max(maxDistance, abs(CENTER_POSITION - servoStates[i].currentPos));
  }
  maxDistance = max(maxDistance, abs(CENTER_POSITION - servoStates[7].currentPos));
  
  int resetMoveTime = calculateMoveTime(maxDistance);
  
  // 同时移动所有舵机到中心位置
  for (int i = 0; i < 7; i++) {
    int servoId = jointServos[i];
    moveServoDirectly(servoId, CENTER_POSITION, resetMoveTime);
    
    // 更新全局状态
    armStatus.joints[i] = CENTER_POSITION;
  }
  
  // 移动夹持器
  moveServoDirectly(gripperServo, CENTER_POSITION, resetMoveTime);
  armStatus.gripper = CENTER_POSITION;
  
  // 标记为已更新
  armStatus.updated = true;
}

// 处理手柄输入的舵机控制
void handleServoControl(int servoId, int position) {
  // 获取舵机索引
  int index;
  
  if (servoId == gripperServo) {
    index = 7;
  } else {
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        index = i;
        break;
      }
    }
  }
  
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 平滑移动到目标位置
  moveServoSmoothly(servoId, position);
  
  // 更新全局状态
  if (servoId == gripperServo) {
    armStatus.gripper = position;
  } else {
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        armStatus.joints[i] = position;
        break;
      }
    }
  }
  
  // 标记为已更新
  armStatus.updated = true;
  armStatus.lastCommandTime = millis();
}
