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

        // 分配按键控制舵机
        if (ps2x.Button(PSB_PAD_UP)) {
            servos.moveServo(jointServos[0], 1000, 500); // 舵机1上移
        } else if (ps2x.Button(PSB_PAD_DOWN)) {
            servos.moveServo(jointServos[0], 0, 500); // 舵机1下移
        }

        if (ps2x.Button(PSB_PAD_LEFT)) {
            servos.moveServo(jointServos[1], 1000, 500); // 舵机2左移
        } else if (ps2x.Button(PSB_PAD_RIGHT)) {
            servos.moveServo(jointServos[1], 0, 500); // 舵机2右移
        }

        if (ps2x.Button(PSB_TRIANGLE)) {
            servos.moveServo(jointServos[2], 1000, 500); // 舵机3上移
        } else if (ps2x.Button(PSB_CROSS)) {
            servos.moveServo(jointServos[2], 0, 500); // 舵机3下移
        }

        if (ps2x.Button(PSB_SQUARE)) {
            servos.moveServo(jointServos[3], 1000, 500); // 舵机4左移
        } else if (ps2x.Button(PSB_CIRCLE)) {
            servos.moveServo(jointServos[3], 0, 500); // 舵机4右移
        }

        if (ps2x.Button(PSB_L1)) {
            servos.moveServo(jointServos[4], 1000, 500); // 舵机5上移
        } else if (ps2x.Button(PSB_L2)) {
            servos.moveServo(jointServos[4], 0, 500); // 舵机5下移
        }

        if (ps2x.Button(PSB_R1)) {
            servos.moveServo(jointServos[5], 1000, 500); // 舵机6上移
        } else if (ps2x.Button(PSB_R2)) {
            servos.moveServo(jointServos[5], 0, 500); // 舵机6下移
        }

        if (ps2x.Button(PSB_SELECT)) {
            servos.moveServo(jointServos[6], 1000, 500); // 舵机7上移
        } else if (ps2x.Button(PSB_START)) {
            servos.moveServo(jointServos[6], 0, 500); // 舵机7下移
        }

        // 夹爪控制
        if (ps2x.Button(PSB_L3)) {
            servos.moveServo(gripperServo, 1000, 500); // 夹爪打开
        } else if (ps2x.Button(PSB_R3)) {
            servos.moveServo(gripperServo, 0, 500); // 夹爪关闭
        }

        // 复位键
        if (ps2x.ButtonPressed(PSB_START)) {
            for (int i = 0; i < 7; i++) {
                servos.moveServo(jointServos[i], 500, 1000); // 所有舵机复位到中间位置
            }
            servos.moveServo(gripperServo, 500, 1000); // 夹爪复位到中间位置
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 延时以降低任务频率
    }
}
