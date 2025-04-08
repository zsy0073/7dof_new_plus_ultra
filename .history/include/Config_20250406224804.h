#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 定义板载LED引脚
#define LED_PIN 4

// 定义PS2手柄引脚
#define PS2_DAT_PIN 12
#define PS2_CMD_PIN 11
#define PS2_SEL_PIN 10
#define PS2_CLK_PIN 9

// 定义舵机ID
extern const int jointServos[7];
extern const int gripperServo;

// 基本舵机控制参数
#define SERVO_MOVE_TIME 500     // 标准移动时间(毫秒)
#define MIN_MOVE_TIME 100       // 最小移动时间(毫秒)
#define MAX_MOVE_TIME 1000      // 最大移动时间(毫秒)

// 舵机平滑移动参数
#define MOVE_STEP_TIME 50       // 每步默认移动时间(毫秒)
#define POSITION_DEADZONE 5     // 位置死区(忽略小变化)
#define MIN_COMMAND_INTERVAL 20 // 命令最小间隔(毫秒)
#define CENTER_POSITION 500     // 中心位置值

// 定义串口引脚
#define HANDLE_SERIAL_RX_PIN 18
#define HANDLE_SERIAL_TX_PIN 19
#define SERVO_SERIAL_RX_PIN 16
#define SERVO_SERIAL_TX_PIN 17
// 定义 I2C 引脚
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 14
// 定义调试开关和日志级别
#define DEBUG_MODE true
#define LOG_INFO(msg) // 移除日志
#define LOG_ERROR(msg) // 移除日志
#define LOG_DEBUG(msg, val) // 移除日志

// WiFi配置
extern const char* ssid;
extern const char* password;

// 学号和姓名
extern const char* studentID;
extern const char* studentName;

// 定义机械臂状态结构体
struct ArmStatus {
  float temperature = 0.0;
  int joints[7] = {500, 500, 500, 500, 500, 500, 500}; // 初始化为中心位置
  int gripper = 500; // 初始化为中心位置
  bool updated = false;
  unsigned long lastCommandTime = 0; // 最后一次命令时间记录
};

// 舵机控制状态追踪结构体 - 简化版
struct ServoState {
  int currentPos = 500;    // 当前位置
};

extern ArmStatus armStatus;
extern ServoState servoStates[8]; // 7个关节 + 1个夹持器

#endif // CONFIG_H
