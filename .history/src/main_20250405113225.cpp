#include <Arduino.h>
#include <LobotServoController.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 定义串口引脚
#define HANDLE_SERIAL_RX_PIN 18
#define HANDLE_SERIAL_TX_PIN 19
#define SERVO_SERIAL_RX_PIN 16
#define SERVO_SERIAL_TX_PIN 17

// 为每个关节舵机创建独立任务
void jointServoTask(void *pvParameters) {
  int servoIndex = (int)pvParameters;
  while (1) {
    int angle = random(0, 180);
    servos.moveServo(jointServos[servoIndex], angle, 1000);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// 为夹爪舵机创建独立任务
void gripperServoTask(void *pvParameters) {
  while (1) {
    int gripperAngle = random(0, 180);
    servos.moveServo(gripperServo, gripperAngle, 1000);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void setup() {
  // 初始化串口通信
  Serial.begin(115200);  // USB-CDC串口，用于调试
  Serial1.begin(9600, SERIAL_8N1, HANDLE_SERIAL_RX_PIN, HANDLE_SERIAL_TX_PIN);  // 手柄控制
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);    // 舵机控制

  // 初始化舵机角度
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], 90, 1000);
  }
  servos.moveServo(gripperServo, 90, 1000);

  // 创建关节舵机控制任务
  for (int i = 0; i < 7; i++) {
    xTaskCreate(jointServoTask, "JointServoTask", 2048, (void *)i, 1, NULL);
  }

  // 创建夹爪舵机控制任务
  xTaskCreate(gripperServoTask, "GripperServoTask", 2048, NULL, 1, NULL);
}

// 定义调试开关
bool DEBUG_MODE = false;

void parseCommand(String command) {
  if (command.startsWith("$J")) {
    int colonIndex = command.indexOf(":");
    if (colonIndex != -1) {
      int servoId = command.substring(2, colonIndex).toInt();
      int angle = command.substring(colonIndex + 1, command.length() - 1).toInt();
      // 修改所有输出调试信息的地方，例如：
      if (DEBUG_MODE) {
        Serial.print("Parsed servo ID: ");
        Serial.println(servoId);
        Serial.print("Parsed angle: ");
        Serial.println(angle);
      }
      servos.moveServo(servoId, angle, 1000);
    }
  }
}

void loop() {
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('*');
    if (DEBUG_MODE) {
      Serial.print("Received command: ");
      Serial.println(command);
    }
    parseCommand(command);
  }
}