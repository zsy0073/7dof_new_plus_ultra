#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 定义板载LED引脚
#define LED_PIN 4

// 定义舵机ID
extern const int jointServos[7];
extern const int gripperServo;

// 定义舵机运动时间（单位：毫秒）
#define SERVO_MOVE_TIME 500
// 添加舵机移动参数
#define MIN_MOVE_TIME 200  // 最小移动时间
#define MAX_MOVE_TIME 1500 // 最大移动时间
#define DISTANCE_FACTOR 1  // 每单位距离多少毫秒
// 添加命令处理间隔控制
#define MIN_COMMAND_INTERVAL 20  // 最小命令间隔(ms)
// 添加平滑移动参数
#define SMOOTH_STEPS 5     // 平滑移动的步数(增加)
#define SMOOTH_DELAY 60    // 每步之间的延迟(ms)(调整)
// 添加加速减速参数
#define ACCEL_STEPS 10     // 加速减速总步数(增加)
#define ACCEL_DELAY_MAX 150 // 最大步延迟(ms)(增加)
#define ACCEL_DELAY_MIN 40  // 最小步延迟(ms)(调整)
// 添加非阻塞控制参数
#define USE_NON_BLOCKING_MOVES true // 启用非阻塞移动

// 摇杆释放检测参数
#define STICK_RELEASE_TIME_THRESHOLD 120  // 摇杆释放时间阈值(ms)(增加)
#define STICK_CENTER_RANGE 100           // 中心位置范围(500±100)
#define STICK_DIRECTION_CHANGE_THRESHOLD 100 // 方向变化阈值
#define DECELERATION_STEPS 5             // 摇杆释放时的减速步数
#define DECELERATION_FACTOR 3.0          // 减速因子(越大减速越明显)

// 运动状态跟踪参数
#define MOTION_HISTORY_SIZE 5            // 记录历史运动的数量
#define VELOCITY_THRESHOLD 10            // 速度阈值，低于此值视为停止

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
  unsigned long lastCommandTime = 0; // 添加最后一次命令时间记录
};

extern ArmStatus armStatus;

#endif // CONFIG_H
