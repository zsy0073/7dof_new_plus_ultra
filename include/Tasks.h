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
#include "ArmKinematics.h" // 添加运动学头文件

// 机械臂运动学命令类型枚举
typedef enum {
    MOVE_TO_POSITION, // 移动到指定位置
    MOVE_TO_JOINTS,   // 移动到指定关节角度
    SET_VELOCITY,     // 设置运动速度
    ABORT_MOVEMENT    // 中止当前运动
} KinematicsCmdType;

// 机械臂运动学命令结构体
typedef struct {
    KinematicsCmdType type;    // 命令类型
    union {
        struct {                // 位置命令
            float x, y, z;      // 目标位置
            float rx, ry, rz;   // 目标姿态(弧度)
        } position;
        struct {                // 关节命令
            float joints[ARM_DOF]; // 目标关节角度(弧度)
        } joints;
        struct {                // 速度命令
            float linear;       // 线速度比例 (0.0-1.0)
            float angular;      // 角速度比例 (0.0-1.0)
        } velocity;
    } data;
} KinematicsCommand;

// 任务函数声明 - 保留所有任务函数，它们都在使用中
void networkTask(void *parameter);
void servoControlTask(void *parameter);
void displayAndTemperatureTask(void *parameter);
void ps2ControllerTask(void *parameter);
void kinematicsTask(void *parameter); // 添加运动学任务
void setupTasks();

// 外部变量声明
extern QueueHandle_t servoCommandQueue;
extern QueueHandle_t kinematicsCommandQueue; // 添加运动学命令队列
extern volatile bool isCommandExecuting;
extern ArmKinematics armKinematics; // 机械臂运动学对象

// 辅助函数
bool moveToPosition(float x, float y, float z, float rx, float ry, float rz);
bool moveToJoints(const float joints[ARM_DOF]);

#endif // TASKS_H
