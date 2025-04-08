#include <Arduino.h>
#include <LobotServoController.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "webpage.h"  

// 定义板载LED引脚
#define LED_PIN 2

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

// 定义调试开关和日志级别
#define DEBUG_MODE true
#define LOG_INFO(msg) if(DEBUG_MODE) { Serial1.println("[INFO] " msg); }
#define LOG_ERROR(msg) if(DEBUG_MODE) { Serial1.println("[ERROR] " msg); }
#define LOG_DEBUG(msg, val) if(DEBUG_MODE) { Serial1.print("[DEBUG] " msg); Serial1.println(val); }

// WiFi配置
const char* ssid = "404notfound";
const char* password = "404notfound";

// 创建Web服务器和WebSocket对象
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
  LOG_INFO("WiFi connected");
  LOG_DEBUG("IP Address: ", WiFi.localIP().toString().c_str());

  // 配置WebSocket事件处理
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
      case WS_EVT_CONNECT:
        LOG_INFO("WebSocket client connected");
        break;
      case WS_EVT_DISCONNECT:
        LOG_INFO("WebSocket client disconnected");
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

void setup() {
  // 初始化串口通信
  Serial.begin(115200);  // USB-CDC串口，用于调试
  Serial1.begin(9600, SERIAL_8N1, HANDLE_SERIAL_RX_PIN, HANDLE_SERIAL_TX_PIN);  // 手柄控制
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);    // 舵机控制
  
  LOG_INFO("7DOF Robot Arm System Starting...");
  LOG_INFO("Initializing servos...");

  // 初始化舵机角度
  for (int i = 0; i < 7; i++) {
    LOG_DEBUG("Setting joint servo to center position, ID: ", jointServos[i]);
    servos.moveServo(jointServos[i], 90, 1000);
    delay(100); // 添加短暂延时确保命令发送
  }
  
  LOG_DEBUG("Setting gripper to center position, ID: ", gripperServo);
  servos.moveServo(gripperServo, 90, 1000);
  
  LOG_INFO("System initialization complete!");

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
}

void parseCommand(String command) {
  if (command.startsWith("$J")) {
    int colonIndex = command.indexOf(":");
    if (colonIndex != -1) {
      int servoId = command.substring(2, colonIndex).toInt();
      int position = command.substring(colonIndex + 1, command.length() - 1).toInt();
      
      // 确保位置在 0 到 1000 范围内
      position = constrain(position, 0, 1000);

      LOG_DEBUG("Command received for Servo ID: ", servoId);
      LOG_DEBUG("Target position: ", position);
      
      if (position >= 0 && position <= 1000) {
        servos.moveServo(servoId, position, 1000);
        LOG_INFO("Servo movement command sent successfully");
      } else {
        LOG_ERROR("Invalid position value received");
      }
    } else {
      LOG_ERROR("Invalid command format");
    }
  } else {
    LOG_ERROR("Unknown command received");
  }
}

void loop() {
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('*');
    LOG_DEBUG("Raw command received: ", command);
    LOG_DEBUG("Handle Command: ", command);

    // 点亮板载LED
    digitalWrite(LED_PIN, HIGH);

    parseCommand(command);

    // 处理完成后关闭LED
    digitalWrite(LED_PIN, LOW);
  }
}