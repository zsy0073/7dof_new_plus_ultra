#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <Arduino.h>
#include "PS2X_lib.h"
#include "Config.h"
#include "ServoControl.h"

// PS2手柄引脚定义
#define PS2_DAT_PIN 37
#define PS2_CMD_PIN 35
#define PS2_ATT_PIN 36
#define PS2_CLK_PIN 38

// 定义摇杆死区
#define JOYSTICK_DEADZONE 20

// 定义手柄按键到舵机的映射常量
#define PS2_INCREMENT_STEP 10  // 每次按键增减的位置步长值

// 手柄控制模式
enum ControlMode {
  MODE_JOINT,   // 关节控制模式
  MODE_GRIPPER, // 夹持器控制模式
  MODE_MAX      // 模式总数
};

// 初始化PS2手柄
bool initPS2Controller();

// 处理PS2手柄输入
void handlePS2Input();

// 获取当前控制模式
ControlMode getCurrentControlMode();

// 切换控制模式
void switchControlMode();

// 根据摇杆值调整舵机位置
void adjustServoByJoystick(int jointIndex, int stickValue);

// 获取当前选择的关节索引
int getCurrentJointIndex();

#endif // PS2_CONTROLLER_H
