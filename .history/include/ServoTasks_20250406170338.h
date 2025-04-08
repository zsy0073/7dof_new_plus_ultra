#ifndef SERVO_TASKS_H
#define SERVO_TASKS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Config.h"
#include "ServoControl.h"

// 舵机任务参数结构体
typedef struct {
  int servoId;             // 舵机ID
  int targetPosition;      // 目标位置
  int moveTime;            // 移动时间
  SemaphoreHandle_t done;  // 完成信号量
} ServoTaskParams;

// 每个舵机的异步控制任务
void servoMoveTask(void *parameter);

// 为每个舵机创建控制任务
void setupServoTasks();

// 异步移动指定舵机
void moveServoAsync(int servoId, int position, int moveTime);

// 等待所有舵机移动完成
void waitAllServosComplete();

// 复位所有舵机 - 异步版
void resetServosAsync();

// 外部变量声明
extern TaskHandle_t servoTaskHandles[8]; // 7个关节 + 1个夹持器
extern QueueHandle_t servoQueues[8];     // 每个舵机的命令队列
extern SemaphoreHandle_t allServosMutex; // 保护共享资源的互斥量

#endif // SERVO_TASKS_H
