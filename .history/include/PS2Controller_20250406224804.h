#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <Arduino.h>
#include "PS2X_lib.h"
#include "Config.h"

// PS2手柄引脚定义
#define PS2_DAT 12
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_CLK 9

// 控制参数
#define DEAD_ZONE 70          // 摇杆死区值
#define MAX_SPEED 2           // 最大速度增量
#define BUTTON_SPEED 1        // 按钮控制时的速度增量
#define BRAKE_DELAY 200       // 停止延迟(ms)
#define GRIPPER_MIN 0         // 夹爪最小值
#define GRIPPER_MAX 1000      // 夹爪最大值

// 手柄控制器类
class PS2Controller {
private:
    PS2X ps2x;
    bool isConnected;
    unsigned long lastUpdateTime;
    unsigned long lastDebounceTime;
    
    // 关节状态结构体
    struct JointState {
        int currentPos;    // 当前位置 
        int targetSpeed;   // 目标速度
    };
    
    JointState joints[7];  // 7个关节的状态
    int gripperPos;        // 夹爪位置
    
    // 内部方法
    void handleJointControl();
    void handleGripper();
    void resetAll();

public:
    PS2Controller();
    void init();
    void update();
    bool isControllerConnected() { return isConnected; }
};

extern PS2Controller ps2Controller;

#endif // PS2_CONTROLLER_H
