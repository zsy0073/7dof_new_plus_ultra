#include <Arduino.h>
#include <LobotServoController.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "webpage.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// 定义板载LED引脚
#define LED_PIN 4

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 定义舵机运动时间（单位：毫秒）
#define SERVO_MOVE_TIME 500

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
const char* ssid = "404notfound";
const char* password = "404notfound";

// 创建Web服务器和WebSocket对象
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// OLED 显示屏和温度传感器对象
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// 学号和姓名
const char* studentID = "20250001";
const char* studentName = "ZSY";

void displayMessage(const String& line1, const String& line2 = "", const String& line3 = "") {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(line1);
  if (!line2.isEmpty()) {
    display.setCursor(0, 10);
    display.print(line2);
  }
  if (!line3.isEmpty()) {
    display.setCursor(0, 20);
    display.print(line3);
  }
  display.display();
}

void initDisplayAndSensor() {
  if (!mlx.begin()) { // 使用默认I2C地址初始化 MLX90614
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 使用默认I2C地址初始化 OLED
    while (1);
  }

  // 显示初始化动画
  for (int size = 0; size <= 64; size += 4) {
    display.clearDisplay();
    display.drawRect(0, 0, size * 2, size, WHITE);
    display.display();
    delay(100);
  }

  displayMessage("System Ready!");
  delay(2000);

  displayMessage("ID: " + String(studentID), "Name: " + String(studentName));
  delay(3000);
}

// 定义机械臂状态结构体
struct {
  float temperature = 0.0;
  int joints[7] = {90, 90, 90, 90, 90, 90, 90};
  int gripper = 90;
  bool updated = false;
} armStatus;

// 网络服务任务
void networkTask(void *parameter) {
  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // 配置WebSocket事件处理
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
      case WS_EVT_CONNECT:
        break;
      case WS_EVT_DISCONNECT:
        break;
      case WS_EVT_DATA: {
        String command = String((char*)data).substring(0, len);
        Serial.print("Received command: "); // 添加调试信息
        Serial.println(command);
        
        if (command == "R") {
          // 复位所有舵机
          for (int i = 0; i < 7; i++) {
            servos.moveServo(jointServos[i], 500, 1000); // 中心位置调整为 500
            armStatus.joints[i] = 500;
          }
          servos.moveServo(gripperServo, 500, 1000); // 中心位置调整为 500
          armStatus.gripper = 500;
        } else {
          // 解析舵机控制命令
          int colonPos = command.indexOf(':');
          if (colonPos != -1) {
            String cmdType = command.substring(0, 1);
            int id = command.substring(1, colonPos).toInt();
            int position = command.substring(colonPos + 1).toInt();
            
            // 确保位置在 0 到 1000 范围内
            position = constrain(position, 0, 1000);

            if (cmdType == "J" && id >= 0 && id < 7) {
              servos.moveServo(jointServos[id], position, 1000);
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              servos.moveServo(gripperServo, position, 1000);
              armStatus.gripper = position;
            }
          }
        }
        armStatus.updated = true;
        break;
      }
    }
  });

  // 配置Web服务器路由
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PAGE_HTML);
  });

  // 启动服务器
  server.begin();

  // 状态更新循环
  while (1) {
    if (armStatus.updated) {
      String json = "{\"temp\":" + String(armStatus.temperature, 1) + 
                   ",\"joints\":[";
      for (int i = 0; i < 7; i++) {
        json += String(armStatus.joints[i]);
        if (i < 6) json += ",";
      }
      json += "],\"gripper\":" + String(armStatus.gripper) + "}";
      ws.textAll(json);
      armStatus.updated = false;
    }
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void controllerTask(void *parameter) {
  while (1) {
    if (Serial1.available() > 0) {
      String command = Serial1.readStringUntil('\n'); // 按行读取命令
      command.trim(); // 去除可能的换行符或空格
      if (command.length() > 0) {
        Serial.print("Received command: "); // 添加调试信息
        Serial.println(command);

        // 点亮板载LED
        digitalWrite(LED_PIN, HIGH);

        if (command == "R") {
          // 复位所有舵机
          for (int i = 0; i < 7; i++) {
            servos.moveServo(jointServos[i], 90, SERVO_MOVE_TIME); // 使用统一的运动时间
            armStatus.joints[i] = 90;
          }
          servos.moveServo(gripperServo, 90, SERVO_MOVE_TIME); // 使用统一的运动时间
          armStatus.gripper = 90;
        } else {
          // 解析舵机控制命令
          int colonPos = command.indexOf(':');
          if (colonPos != -1) {
            String cmdType = command.substring(0, 1);
            int id = command.substring(1, colonPos).toInt();
            int position = command.substring(colonPos + 1).toInt();
            
            // 确保位置在 0 到 1000 范围内
            position = constrain(position, 0, 1000);

            if (cmdType == "J" && id >= 0 && id < 7) {
              servos.moveServo(jointServos[id], position, SERVO_MOVE_TIME); // 使用统一的运动时间
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              servos.moveServo(gripperServo, position, SERVO_MOVE_TIME); // 使用统一的运动时间
              armStatus.gripper = position;
            }
          } else {
          }
        }
        armStatus.updated = true;

        // 处理完成后关闭LED
        digitalWrite(LED_PIN, LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 添加短暂延时以避免任务占用过多资源
  }
}

// 修改初始化I2C函数，使用 GPIO13 (SDA) 和 GPIO14 (SCL)
void initI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // 使用 GPIO13 和 GPIO14 作为 I2C 引脚
}

void displayAndTemperatureTask(void *parameter) {
  bool showTemperature = true;

  while (1) {
    if (showTemperature) {
      float ambientTemp = mlx.readAmbientTempC();
      displayMessage("Ambient Temp: " + String(ambientTemp) + " C", 
                     "ID: " + String(studentID), 
                     "Name: " + String(studentName));
    } else {
      displayMessage("ID: " + String(studentID), "Name: " + String(studentName));
    }
    showTemperature = !showTemperature;
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup() {
  // 初始化串口通信
  Serial.begin(115200);  // USB-CDC串口，用于调试
  Serial1.begin(9600, SERIAL_8N1, HANDLE_SERIAL_RX_PIN, HANDLE_SERIAL_TX_PIN);  // 手柄控制
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);    // 舵机控制
  

  // 初始化舵机角度
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], 90, SERVO_MOVE_TIME); // 使用统一的运动时间
    delay(100); // 添加短暂延时确保命令发送
  }
  
  servos.moveServo(gripperServo, 90, SERVO_MOVE_TIME); // 使用统一的运动时间
  

  // 初始化板载LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 默认关闭LED

  // 创建网络服务任务在核心1上运行
  xTaskCreatePinnedToCore(
    networkTask,     // 任务函数
    "NetworkTask",   // 任务名称
    8192,           // 堆栈大小
    NULL,           // 任务参数
    1,              // 任务优先级
    NULL,           // 任务句柄
    1               // 运行的核心（Core 1）
  );

  // 创建手柄控制任务在核心0上运行
  xTaskCreatePinnedToCore(
    controllerTask,  // 任务函数
    "ControllerTask", // 任务名称
    4096,            // 堆栈大小
    NULL,            // 任务参数
    2,               // 任务优先级
    NULL,            // 任务句柄
    1                // 运行的核心（Core 0）
  );

  initI2C(); // 初始化I2C接口
  initDisplayAndSensor(); // 初始化显示屏和传感器

  // 创建屏幕显示与温度采集任务在核心0上运行
  xTaskCreatePinnedToCore(
    displayAndTemperatureTask, // 任务函数
    "DisplayAndTempTask",      // 任务名称
    4096,                      // 堆栈大小
    NULL,                      // 任务参数
    1,                         // 任务优先级
    NULL,                      // 任务句柄
    0                          // 运行的核心（Core 0）
  );
}

void loop() {
  // 空循环，所有逻辑已移至任务
}