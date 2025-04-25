#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <LobotServoController.h>
#include "Config.h"

// 使用条件编译避免重复定义
#ifndef SERVO_MIN_ANGLE
#define SERVO_MIN_ANGLE 0.0f
#endif

#ifndef SERVO_MAX_ANGLE
#define SERVO_MAX_ANGLE 240.0f
#endif

extern LobotServoController servos;
extern unsigned long servoLastMoveEndTime;

// 舵机命令结构体
struct ServoCommand {
    int servoId;
    int position;
    bool isReset;
};

// 角度转换函数 - 用于240度舵机
int angleToPulse(float angle);
float pulseToAngle(int pulse);

// 新增：轨迹规划角度和舵机物理角度之间的转换函数
// 将轨迹规划角度(-120到+120度，0为中位)转换为舵机物理角度(0-240度)
float trajAngleToServoAngle(float trajAngle);
// 将舵机物理角度(0-240度)转换为轨迹规划角度(-120到+120度，0为中位)
float servoAngleToTrajAngle(float servoAngle);
// 直接将轨迹规划角度转换为舵机控制值
int trajAngleToPulse(float trajAngle);
// 直接将舵机控制值转换为轨迹规划角度
float pulseToTrajAngle(int pulse);

// 基本舵机控制函数
void initServo();
int calculateMoveTime(int distance);
int calculateAutoResetTime();
int getServoIndex(int servoId);
void moveServo(int servoId, int position);
void resetServos();
void handleServoControl(int servoId, int position);

// 批量控制多个舵机
void moveMultipleServos(LobotServo servoArray[], int servoCount, int moveTime);

// 轨迹执行辅助函数
bool isServosReady();  // 检查所有舵机是否就绪
void getCurrentJointAngles(float angles[7]);  // 获取当前关节角度（度）
void setJointAngles(const float angles[7], int moveTime);  // 设置关节角度（度）

// 新增：获取和设置轨迹规划角度（-120到+120度）
void getCurrentTrajAngles(float angles[7]);  // 获取当前关节的轨迹规划角度
void setTrajAngles(const float angles[7], int moveTime);  // 设置关节的轨迹规划角度

#endif  // SERVO_CONTROL_H
