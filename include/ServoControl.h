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

// 函数声明 - 保留所有函数，它们都在使用中
void initServo();
int calculateMoveTime(int distance);
int getServoIndex(int servoId);
void moveServo(int servoId, int position);
void resetServos();
void handleServoControl(int servoId, int position);

#endif // SERVO_CONTROL_H
