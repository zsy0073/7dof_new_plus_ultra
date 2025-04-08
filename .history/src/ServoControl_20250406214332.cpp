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

// 防抖动控制 - 添加上一次位置记录和命令发送时间记录
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_INTERVAL = 100; // 命令发送最小间隔(毫秒)

// 初始化舵机
void initServo() {
  Serial.println("初始化舵机控制...");
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);
  
  // 延时确保串口初始化完成
  delay(200);
  
  // 初始化所有舵机到中心位置
  resetServos();
  
  Serial.println("所有舵机已初始化到中心位置");
}

// 根据移动距离计算移动时间 - 最简单版本
int calculateMoveTime(int distance) {
  // 根据距离返回合理的移动时间，避免移动过快导致抽搐
  if (distance < 50) {
    return 200;  // 小距离移动
  } else if (distance < 200) {
    return 500;  // 中距离移动
  } else {
    return 1000; // 大距离移动
  }
}

// 获取舵机索引
int getServoIndex(int servoId) {
  if (servoId == gripperServo) return 7;
  if (servoId >= 1 && servoId <= 7) return servoId - 1;
  return -1; // 无效ID
}

// 直接控制舵机 - 最简单的实现
void moveServo(int servoId, int position) {
  // 确保舵机ID有效
  if (servoId < 1 || servoId > 8) {
    Serial.print("无效舵机ID: ");
    Serial.println(servoId);
    return;
  }
  
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  
  // 检查命令发送时间间隔，避免命令发送过快
  unsigned long currentTime = millis();
  if (currentTime - lastCommandTime < COMMAND_INTERVAL) {
    return;
  }
  
  // 计算移动时间
  int distance = abs(position - servoStates[index].currentPos);
  int moveTime = calculateMoveTime(distance);
  
  // 更新命令发送时间
  lastCommandTime = currentTime;
  
  // 发送舵机命令
  servos.moveServo(servoId, position, moveTime);
  
  // 更新当前位置记录
  servoStates[index].currentPos = position;
  
  // 更新最后一次舵机移动预计完成的时间
  servoLastMoveEndTime = currentTime + moveTime + 100; // 添加100ms缓冲
  
  // 打印调试信息
  Serial.print("舵机移动: ID=");
  Serial.print(servoId);
  Serial.print(", 位置=");
  Serial.print(position);
  Serial.print(", 时间=");
  Serial.println(moveTime);
  
  // 记录动作组（如果正在记录）
  if (isRecording) {
    recordAction(servoId, position, moveTime);
  }
}

// 复位所有舵机到中心位置
void resetServos() {
  Serial.println("复位所有舵机到中心位置");
  
  // 所有舵机移动到中心位置，固定使用1000ms时间
  const int moveTime = 1000;
  
  // 关节舵机
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], CENTER_POSITION, moveTime);
    servoStates[i].currentPos = CENTER_POSITION;
    armStatus.joints[i] = CENTER_POSITION;
    delay(20); // 短暂延时，避免命令发送过快
  }
  
  // 夹持器
  servos.moveServo(gripperServo, CENTER_POSITION, moveTime);
  servoStates[7].currentPos = CENTER_POSITION;
  armStatus.gripper = CENTER_POSITION;
  
  // 更新最后一次舵机移动预计完成的时间
  servoLastMoveEndTime = millis() + moveTime + 200; // 添加200ms缓冲
  
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
  
  // 直接移动舵机，使用最简单的方式
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
