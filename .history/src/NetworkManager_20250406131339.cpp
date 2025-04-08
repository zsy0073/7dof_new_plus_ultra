#include "NetworkManager.h"
#include "webpage.h"
#include "DisplayUtils.h" // 添加这行以访问mlx对象

// 创建Web服务器和WebSocket对象
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 处理WebSocket消息
void processWebSocketMessage(uint8_t *data, size_t len) {
  String command = String((char*)data).substring(0, len);
  
  // 处理按钮和滑块命令
  if (command == "R") {
    // 使用优化的复位功能
    smoothAccelResetServosOptimized();
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
        // 使用非阻塞移动函数
        smoothAccelMoveServoNonBlocking(jointServos[id], armStatus.joints[id], position);
        armStatus.joints[id] = position;
      } else if (cmdType == "G") {
        // 使用非阻塞移动函数
        smoothAccelMoveServoNonBlocking(gripperServo, armStatus.gripper, position);
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
    String json = "{\"temp\":" + String(armStatus.temperature, 1) + eadAmbientTempC();
                 ",\"joints\":[";
    for (int i = 0; i < 7; i++) {
      json += String(armStatus.joints[i]);" + String(armStatus.temperature, 1) + 
      if (i < 6) json += ",";            ",\"joints\":[";
    }
    json += "],\"gripper\":" + String(armStatus.gripper) + "}";armStatus.joints[i]);
    ws.textAll(json);
    armStatus.updated = false; }
  }   json += "],\"gripper\":" + String(armStatus.gripper) + "}";
}    ws.textAll(json);
us.updated = false;
// 设置Web服务器
void setupWebServer() {
  // 配置WebSocket事件处理
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                AwsEventType type, void *arg, uint8_t *data, size_t len) {r() {
    switch(type) {
      case WS_EVT_CONNECT:连接时，立即发送一次状态数据](AsyncWebSocket *server, AsyncWebSocketClient *client, 
        break; = mlx.readAmbientTempC(); // 更新温度数据type, void *arg, uint8_t *data, size_t len) {
      case WS_EVT_DISCONNECT: json = "{\"temp\":" + String(armStatus.temperature, 1) + e) {
        break;\"joints\":[";CT:
      case WS_EVT_DATA:
        processWebSocketMessage(data, len); += String(armStatus.joints[i]);EVT_DISCONNECT:
        break;     if (i < 6) json += ",";   break;
    }   } case WS_EVT_DATA:
  });        json += "],\"gripper\":" + String(armStatus.gripper) + "}";        processWebSocketMessage(data, len);
>text(json); // 只发送给新连接的客户端
  // 配置Web服务器路由
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PAGE_HTML); case WS_EVT_DATA:配置Web服务器路由
  });        processWebSocketMessage(data, len);  server.addHandler(&ws);
eak;n("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  // 启动服务器html", PAGE_HTML);
  server.begin(); }); });
}











}  server.begin();  // 启动服务器  });    request->send(200, "text/html", PAGE_HTML);  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {  server.addHandler(&ws);  // 配置Web服务器路由  // 启动服务器
  server.begin();
}
