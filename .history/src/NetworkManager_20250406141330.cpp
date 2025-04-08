#include "NetworkManager.h"
#include "webpage.h"
#include "DisplayUtils.h" // 添加头文件以访问mlx对象

// 创建Web服务器和WebSocket对象
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 处理WebSocket消息
void processWebSocketMessage(uint8_t *data, size_t len) {
  String command = String((char*)data).substring(0, len);
  
  // 处理按钮和滑块命令
  if (command == "R") {
    // 复位所有舵机
    resetServos();
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
        // 记录命令接收
        Serial.print("WebSocket J命令 - ID: ");
        Serial.print(id);
        Serial.print(", 位置: ");
        Serial.println(position);
        
        // 控制关节舵机
        handleServoControl(jointServos[id], position);
        armStatus.joints[id] = position;
      } else if (cmdType == "G") {
        Serial.print("WebSocket G命令 - 位置: ");
        Serial.println(position);
        
        // 控制夹持器舵机
        handleServoControl(gripperServo, position);
        armStatus.gripper = position;
      }
    }
  }
  
  armStatus.updated = true;
  armStatus.lastCommandTime = millis();
}

// 发送状态更新到所有WebSocket客户端
void sendStatusUpdate() {
  if (armStatus.updated) {
    // 更新温度数据
    armStatus.temperature = mlx.readAmbientTempC();
    
    // 构建状态JSON并发送
    String json = "{\"temp\":" + String(armStatus.temperature, 1) + 
                 ",\"joints\":[";
    for (int i = 0; i < 7; i++) {
      json += String(armStatus.joints[i]);
      if (i < 6) json += ",";
    }
    json += "],\"gripper\":" + String(armStatus.gripper) + "}";
    
    // 输出发送的JSON数据以便调试
    Serial.print("发送状态JSON: ");
    Serial.println(json);
    
    ws.textAll(json);
    armStatus.updated = false;
  }
}

// 设置Web服务器
void setupWebServer() {
  // 配置WebSocket事件处理
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
      case WS_EVT_CONNECT:
        // 当客户端连接时，立即发送一次状态数据
        {
          float temp = mlx.readAmbientTempC();
          String json = "{\"temp\":" + String(temp, 1) + 
                      ",\"joints\":[";
          for (int i = 0; i < 7; i++) {
            json += String(armStatus.joints[i]);
            if (i < 6) json += ",";
          }
          json += "],\"gripper\":" + String(armStatus.gripper) + "}";
          client->text(json);
        }
        break;
      case WS_EVT_DISCONNECT:
        break;
      case WS_EVT_DATA:
        processWebSocketMessage(data, len);
        break;
    }
  });

  // 配置Web服务器路由
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PAGE_HTML);
  });

  // 启动服务器
  server.begin();
}
