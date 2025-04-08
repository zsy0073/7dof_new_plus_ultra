#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <LobotServoController.h>
#include "Config.h"

extern LobotServoController servos;

// 函数声明
void initServo();
int calculateMoveTime(int distance);
int getServoIndex(int servoId);
void moveServoDirectly(int servoId, int position, int moveTime);
void moveServoSmoothly(int servoId, int targetPos);
void updateServoStatus();
void resetServos();
void handleServoControl(int servoId, int position);

#endif // SERVO_CONTROL_H
