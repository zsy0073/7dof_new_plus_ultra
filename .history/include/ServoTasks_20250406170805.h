#ifndef SERVO_TASKS_H
#define SERVO_TASKS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "ServoControl.h"

// 舵机命令类型
enum ServoCommandType {
  CMD_MOVE,          // 移动到指定位置
  CMD_RESET,         // 复位到中心位置
  CMD_STOP           // 停止当前动作
};

// 舵机命令结构体
typedef struct {
  ServoCommandType type;  // 命令类型
  int position;           // 目标位置 (仅用于MOVE)
  int moveTime;           // 移动时间 (毫秒)
  bool notifyComplete;    // 是否在完成时通知
} ServoTaskCommand;

// 初始化舵机任务系统
void initServoTasks();

// 异步移动单个舵机
void moveServoAsync(int servoId, int position);

// 使用自定义参数移动舵机
void moveServoWithParamsAsync(int servoId, int position, int moveTime, bool waitComplete);

// 复位单个舵机
void resetServoAsync(int servoId);

// 复位所有舵机
void resetAllServosAsync();

// 等待所有舵机动作完成
bool waitForAllServosComplete(int timeoutMs);

// 检查特定舵机是否处于运动状态
bool isServoMoving(int servoId);

// 检查是否有舵机正在运动
bool areAnyServosMoving();

// 外部变量
extern SemaphoreHandle_t servoMutex;       // 舵机通信互斥量
extern bool servoMovingStatus[8];          // 舵机运动状态

#endif // SERVO_TASKS_H
