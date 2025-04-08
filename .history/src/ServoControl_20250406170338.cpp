#include "ServoControl.h"
#include "ActionGroup.h"
#include "ServoTasks.h"  // 添加新头文件

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
  
  // 设置舵机任务
  setupServoTasks();
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

// 直接控制舵机 - 现在使用异步方式
void moveServo(int servoId, int position) {
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 计算移动时间
  int currentPos = servoStates[index].currentPos;
  int distance = abs(position - currentPos);
  int moveTime = calculateMoveTime(distance);
  
  // 使用异步方式控制舵机
  moveServoAsync(servoId, position, moveTime);
  
  // 更新最后一次舵机移动预计完成的时间
  servoLastMoveEndTime = millis() + moveTime + 50;
}

// 复位所有舵机到中心位置
void resetServos() {
  Serial.println("复位所有舵机到中心位置");
  
  // 使用异步方式复位所有舵机
  resetServosAsync();
  
  // 更新移动预计完成时间
  int maxDistance = 0;
  for (int i = 0; i < 8; i++) {
    int distance = abs(CENTER_POSITION - servoStates[i].currentPos);
    maxDistance = max(maxDistance, distance);
  }
  
  int moveTime = calculateMoveTime(maxDistance);
  servoLastMoveEndTime = millis() + moveTime + 100;
  
  // 标记为已更新
  armStatus.updated = true;
  
  // 如果正在记录动作组，则记录复位动作
  if (isRecording && millis() - lastActionTime >= ACTION_INTERVAL) {
    // 使用特殊ID表示这是一个复位命令
    recordAction(RESET_ACTION_ID, 0, moveTime);
    lastActionTime = millis();
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
