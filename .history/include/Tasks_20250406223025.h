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
#include "PS2Controller.h"  // 添加PS2控制器头文件

// 任务函数声明
void networkTask(void *parameter);
void controllerTask(void *parameter);
void servoControlTask(void *parameter);  // 改名，统一处理所有舵机命令
void displayAndTemperatureTask(void *parameter);
void ps2ControllerTask(void *parameter); // 添加PS2控制器任务
void setupTasks();

// 外部变量声明
extern QueueHandle_t servoCommandQueue;
extern volatile bool isCommandExecuting;

#endif // TASKS_H
