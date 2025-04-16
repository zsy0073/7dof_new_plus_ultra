#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <LobotServoController.h>
#include "Config.h"

extern LobotServoController servos;
extern unsigned long servoLastMoveEndTime;

// 舵机命令结构体 - 保留，在多个文件中使用
struct ServoCommand {
  int servoId;
  int position;
  bool isReset;
};

// 角度转换函数 - 用于240度舵机
int angleToPulse(float angle);
float pulseToAngle(int pulse);

// 函数声明 - 保留所有函数，它们都在使用中
void initServo();
int calculateMoveTime(int distance);
int calculateAutoResetTime(); // 添加自动计算复位时间的函数
int getServoIndex(int servoId);
void moveServo(int servoId, int position);
void resetServos();
void handleServoControl(int servoId, int position);

#endif  // SERVO_CONTROL_H
