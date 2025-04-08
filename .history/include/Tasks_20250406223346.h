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

// 前向声明PS2Controller.h中的内容，避免循环引用
enum ControlMode;
bool initPS2Controller();
void handlePS2Input();
ControlMode getCurrentControlMode();
int getCurrentJointIndex();

// 舵机命令结构体
struct ServoCommand {
  int servoId;
  int position;
  bool isReset;
};

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
