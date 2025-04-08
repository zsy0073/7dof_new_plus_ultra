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
            servos.moveServo(jointServos[id], position, 1000);
        } else if (cmdType == "G") {
            servos.moveServo(gripperServo, position, 1000);
        }
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

        // 根据手柄按键生成指令并调用指令处理函数
        if (ps2x.Button(PSB_PAD_UP)) {
            handleCommand("J0:1000"); // 舵机1上移
        } else if (ps2x.Button(PSB_PAD_DOWN)) {
            handleCommand("J0:0"); // 舵机1下移
        }

        if (ps2x.Button(PSB_PAD_LEFT)) {
            handleCommand("J1:1000"); // 舵机2左移
        } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            handleCommand("J1:0"); // 舵机2右移
        }

        if (ps2x.Button(PSB_TRIANGLE)) {
            handleCommand("J2:1000"); // 舵机3上移
        } else if (ps2x.Button(PSB_CROSS)) {
            handleCommand("J2:0"); // 舵机3下移
        }

        if (ps2x.Button(PSB_SQUARE)) {
            handleCommand("J3:1000"); // 舵机4左移
        } else if (ps2x.Button(PSB_CIRCLE)) {
            handleCommand("J3:0"); // 舵机4右移
        }

        if (ps2x.Button(PSB_L1)) {
            handleCommand("J4:1000"); // 舵机5上移
        } else if (ps2x.Button(PSB_L2)) {
            handleCommand("J4:0"); // 舵机5下移
        }

        if (ps2x.Button(PSB_R1)) {
            handleCommand("J5:1000"); // 舵机6上移
        } else if (ps2x.Button(PSB_R2)) {
            handleCommand("J5:0"); // 舵机6下移
        }

        if (ps2x.Button(PSB_SELECT)) {
            handleCommand("J6:1000"); // 舵机7上移
        } else if (ps2x.Button(PSB_START)) {
            handleCommand("J6:0"); // 舵机7下移
        }

        // 夹爪控制
        if (ps2x.Button(PSB_L3)) {
            handleCommand("G:1000"); // 夹爪打开
        } else if (ps2x.Button(PSB_R3)) {
            handleCommand("G:0"); // 夹爪关闭
        }

        // 复位键
        if (ps2x.ButtonPressed(PSB_START)) {
            handleCommand("R"); // 复位所有舵机
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 延时以降低任务频率
    }
}
