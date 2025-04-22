#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Config.h"
#include "ServoControl.h"
#include "DisplayUtils.h"
#include "NetworkManager.h"
#include "PS2Controller.h"
#include "TrajectoryExecutor.h"

// 全局轨迹执行器
extern TrajectoryExecutor* trajectoryExecutor;

// 任务函数声明 - 保留所有任务函数，它们都在使用中
void networkTask(void *parameter);
void servoControlTask(void *parameter);
void displayAndTemperatureTask(void *parameter);
void ps2ControllerTask(void *parameter);
void trajectoryTask(void *parameter);
void setupTasks();

// 外部变量声明
extern QueueHandle_t servoCommandQueue;
extern volatile bool isCommandExecuting;
extern volatile bool isTrajectoryRunning;

#endif // TASKS_H
