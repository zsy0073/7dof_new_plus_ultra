#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Config.h"
#include "ServoControl.h"
#include "DisplayUtils.h"
#include "NetworkManager.h"

// 任务函数声明
void networkTask(void *parameter);
void controllerTask(void *parameter);
void displayAndTemperatureTask(void *parameter);
void setupTasks();

#endif // TASKS_H
