#include "NetworkManager.h"
#include "webpage.h"
#include "DisplayUtils.h"
#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件

// 创建Web服务器和WebSocket对象
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// 处理WebSocket消息
void processWebSocketMessage(uint8_t *data, size_t len) {
  String command = String((char*)data).substring(0, len);
  
  // 打印接收到的WebSocket指令
  Serial.print("WebSocket收到指令: [");
  Serial.print(command);
  Serial.println("]");
  
  // 声明命令结构体
  struct ServoCommand {
    int servoId;
    int position;
    bool isReset;
  } cmd;
  
  // 处理动作组命令
  if (command == "action_record_start") {
    startRecordActionGroup();
    return;
  } else if (command == "action_record_stop") {
    stopRecordActionGroup();
    return;
  } else if (command == "action_play") {
    playActionGroup();
    return;
  }
  
  // 处理按钮和滑块命令
  if (command == "R") {
    Serial.println("WebSocket加入队列: [复位所有舵机]");
    // 创建复位命令
    cmd.isReset = true;
    xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
  } else {
    // 解析舵机控制命令
    int colonPos = command.indexOf(':');
    if (colonPos != -1 && colonPos < command.length()-1) {
      String cmdType = command.substring(0, 1);
      
      // 验证命令类型
      if (cmdType != "J" && cmdType != "G") {
        Serial.println("WebSocket: 无效的命令类型: " + cmdType);
        return;
      }
      
      // 提取并验证ID部分
      String idStr = command.substring(1, colonPos);
      bool isValidID = true;
      
      // 确保ID字符串只包含数字
      for (int i = 0; i < idStr.length(); i++) {
        if (!isDigit(idStr.charAt(i))) {
          isValidID = false;
          break;
        }
      }
      
      if (!isValidID) {
        Serial.println("WebSocket: 无效的ID格式: " + idStr);
        return;
      }
      
      int id = idStr.toInt();
      
      // 提取并验证位置部分
      String posStr = command.substring(colonPos + 1);
      bool isValidPos = true;
      
      // 确保位置字符串只包含数字
      for (int i = 0; i < posStr.length(); i++) {
        if (!isDigit(posStr.charAt(i))) {
          isValidPos = false;
          break;
        }
      }
      
      if (!isValidPos) {
        Serial.println("WebSocket: 无效的位置格式: " + posStr);
        return;
      }
      
      int position = posStr.toInt();
      
      // 打印解析结果用于调试
      Serial.print("WebSocket 解析命令: 类型=");
      Serial.print(cmdType);
      Serial.print(", ID=");
      Serial.print(id);
      Serial.print(", 位置=");
      Serial.println(position);
      
      // 确保位置在 0 到 1000 范围内
      position = constrain(position, 0, 1000);
      cmd.isReset = false;

      if (cmdType == "J" && id >= 0 && id < 7) {
        // 记录命令接收
        Serial.print("WebSocket J命令 - ID: ");
        Serial.print(id);
        Serial.print(", 位置: ");
        Serial.println(position);
        
        // 使用队列发送命令
        cmd.servoId = jointServos[id];
        cmd.position = position;
        xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
        
        // 更新状态
        armStatus.joints[id] = position;
      } else if (cmdType == "G") {
        Serial.print("WebSocket G命令 - 位置: ");
        Serial.println(position);
        
        // 使用队列发送命令
        cmd.servoId = gripperServo;
        cmd.position = position;
        xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
        
        // 更新状态
        armStatus.gripper = position;
      } else {
        Serial.println("WebSocket: 无效的命令类型或ID范围");
      }
    } else {
      Serial.println("WebSocket: 命令格式错误");
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
    json += "],\"gripper\":" + String(armStatus.gripper);
    
    // 添加动作组状态 (改为英文)
    if (isRecording) {
      json += ",\"actionStatus\":\"Recording actions - " + String(getActionCount()) + " actions recorded\"";
    } else if (isPlaying) {
      json += ",\"actionStatus\":\"Playing action sequence\"";
    } else if (getActionCount() > 0) {
      json += ",\"actionStatus\":\"" + String(getActionCount()) + " actions recorded, ready to play\"";
    } else {
      json += ",\"actionStatus\":\"\"";
    }
    
    json += "}";
    
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
