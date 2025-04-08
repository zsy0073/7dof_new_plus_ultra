#include <Arduino.h>
#include <LobotServoController.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 定义串口引脚 (UART0使用默认引脚)
#define DEBUG_SERIAL_RX_PIN 18    // UART1用于调试
#define DEBUG_SERIAL_TX_PIN 19
#define SERVO_SERIAL_RX_PIN 16    // UART2用于舵机
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
  Serial.begin(9600);     // UART0，用于手柄控制
  Serial1.begin(115200, SERIAL_8N1, DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);  // 调试用
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
      if (DEBUG_MODE) {
        Serial1.print("Parsed servo ID: ");  // 改用Serial1输出调试信息
        Serial1.println(servoId);
        Serial1.print("Parsed angle: ");
        Serial1.println(angle);
      }
      servos.moveServo(servoId, angle, 1000);
    }
  }
}

void loop() {
  if (Serial.available() > 0) {   // 使用UART0(Serial)读取手柄数据
    String command = Serial.readStringUntil('*');
    if (DEBUG_MODE) {
      Serial1.print("Received command: ");   // 改用Serial1输出调试信息
      Serial1.println(command);
    }
    parseCommand(command);
  }
}