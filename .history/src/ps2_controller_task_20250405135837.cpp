#include <Arduino.h>
#include <PS2X_lib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LobotServoController.h>

// 定义手柄引脚
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14

// 定义舵机控制对象
extern LobotServoController servos;

// 定义舵机ID
extern const int jointServos[7];
extern const int gripperServo;

// 创建PS2手柄对象
PS2X ps2x;

// 定义舵机目标位置数组
int targetPositions[7] = {500, 500, 500, 500, 500, 500, 500};
int gripperTargetPosition = 500;

// 指令处理函数
void handleCommand(const String &command) {
    int colonPos = command.indexOf(':');
    if (colonPos != -1) {
        String cmdType = command.substring(0, 1);
        int id = command.substring(1, colonPos).toInt();
        int position = command.substring(colonPos + 1).toInt();

        // 确保位置在 0 到 1000 范围内
        position = constrain(position, 0, 1000);

        if (cmdType == "J" && id >= 0 && id < 7) {
            targetPositions[id] = position;
            servos.moveServo(jointServos[id], position, 100);
        } else if (cmdType == "G") {
            gripperTargetPosition = position;
            servos.moveServo(gripperServo, position, 100);
        }
    } else if (command == "R") {
        // 复位所有舵机
        for (int i = 0; i < 7; i++) {
            targetPositions[i] = 500;
            servos.moveServo(jointServos[i], 500, 100);
        }
        gripperTargetPosition = 500;
        servos.moveServo(gripperServo, 500, 100);
    }
}

// 手柄控制任务
void ps2ControllerTask(void *parameter) {
    // 初始化手柄
    if (ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false) != 0) {
        Serial.println("[ERROR] PS2 Controller not found or not connected.");
        vTaskDelete(NULL);
    }

    Serial.println("[INFO] PS2 Controller initialized successfully.");

    while (1) {
        ps2x.read_gamepad();

        // 根据手柄模拟摇杆值调整舵机目标位置
        for (int i = 0; i < 7; i++) {
            int analogValue = ps2x.Analog(PSS_LY + i); // 获取模拟摇杆值
            if (analogValue > 128 + 10 || analogValue < 128 - 10) { // 添加死区
                int delta = map(analogValue, 0, 255, -10, 10); // 将摇杆值映射为增量
                targetPositions[i] = constrain(targetPositions[i] + delta, 0, 1000);
                servos.moveServo(jointServos[i], targetPositions[i], 100);
            }
        }

        // 控制夹爪
        int gripperAnalogValue = ps2x.Analog(PSS_RY); // 获取右摇杆值
        if (gripperAnalogValue > 128 + 10 || gripperAnalogValue < 128 - 10) { // 添加死区
            int delta = map(gripperAnalogValue, 0, 255, -10, 10); // 将摇杆值映射为增量
            gripperTargetPosition = constrain(gripperTargetPosition + delta, 0, 1000);
            servos.moveServo(gripperServo, gripperTargetPosition, 100);
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 延时以降低任务频率
    }
}
