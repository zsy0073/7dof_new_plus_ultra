#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <LobotServoController.h>
#include "Config.h"

// 非阻塞舵机控制状态结构
struct ServoMoveStatus {
  bool isMoving = false;
  unsigned long moveStartTime = 0;
  unsigned long moveDuration = 0;
  int targetPosition = 0;
  int servoId = 0;
};

extern LobotServoController servos;
extern ServoMoveStatus servoStatus[8]; // 7个关节 + 1个夹持器

// 函数声明
void initServo();
int calculateMoveTime(int currentPos, int targetPos);
float calculateSCurveValue(int step, int totalSteps);
int calculateStepDelay(int step, int totalSteps);
void nonBlockingDelay(unsigned long delayTime);
void checkAndUpdateServoStatus();
void startServoMove(int servoId, int position, int moveTime);
void smoothAccelMoveServo(int servoId, int currentPos, int targetPos);
void smoothAccelMoveServoNonBlocking(int servoId, int currentPos, int targetPos);
void smoothAccelResetServos();
void smoothAccelResetServosOptimized();

#endif // SERVO_CONTROL_H
