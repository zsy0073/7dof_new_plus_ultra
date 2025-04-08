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

// 任务函数声明
void networkTask(void *parameter);
void controllerTask(void *parameter);
void networkServoTask(void *parameter); // 更改名称，明确功能
void displayAndTemperatureTask(void *parameter);
void setupTasks();

// 外部变量声明
extern QueueHandle_t servoCommandQueue;

#endif // TASKS_H
