#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <LobotServoController.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "Config.h"

// 最大舵机数量（7个关节 + 1个夹持器）
#define MAX_SERVOS 8

// 舵机命令结构体
typedef struct {
  int position;          // 目标位置
  int moveTime;          // 移动时间
  unsigned long timestamp; // 命令时间戳
} ServoCommand_t;

extern LobotServoController servos;
extern unsigned long servoLastMoveEndTime;

// 舵机队列和任务句柄
extern QueueHandle_t servoQueues[MAX_SERVOS];
extern TaskHandle_t servoTasks[MAX_SERVOS];

// 命令和状态管理
extern ServoState servoStates[MAX_SERVOS];
extern volatile bool servosBusy[MAX_SERVOS]; // 每个舵机的忙碌状态

// 函数声明
void initServo();
int calculateMoveTime(int distance);
int getServoIndex(int servoId);
void moveServo(int servoId, int position, int moveTime = 0);
void resetServos();
void handleServoControl(int servoId, int position);

// 创建舵机控制任务
void createServoTasks();

// 单个舵机的控制任务
void servoTask(void* parameters);

#endif // SERVO_CONTROL_H
