#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <LobotServoController.h>
#include "Config.h"

extern LobotServoController servos;
extern unsigned long servoLastMoveEndTime;

// 舵机命令结构体
struct ServoCommand {
  int servoId;
  int position;
  bool isReset;
};

// 简化后的函数声明
void initServo();
int calculateMoveTime(int distance);
int getServoIndex(int servoId);
void moveServo(int servoId, int position);
void resetServos();
void handleServoControl(int servoId, int position);

#endif // SERVO_CONTROL_H
