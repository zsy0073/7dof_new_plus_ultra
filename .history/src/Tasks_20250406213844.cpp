#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件

// 实例化机械臂状态
ArmStatus armStatus;

// 舵机控制队列
QueueHandle_t servoCommandQueue;
const int queueSize = 20;  // 队列大小

// 舵机命令结构体
struct ServoCommand {
  int servoId;
  int position;
  bool isReset;
};

// 命令状态跟踪
volatile bool isCommandExecuting = false;

// 网络服务任务
void networkTask(void *parameter) {
  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // 设置并启动Web服务器
  setupWebServer();

  // 状态更新循环
  while (1) {
    sendStatusUpdate(); // 发送状态更新
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(50)); // 更新间隔
  }
}

// 专门处理串口命令的任务函数
void controllerTask(void *parameter) {
  while (1) {
    if (Serial1.available() > 0) {
      String command = Serial1.readStringUntil('\n'); // 按行读取命令
      command.trim(); // 去除可能的换行符或空格
      
      // 添加打印接收到的指令
      Serial.print("收到指令: [");
      Serial.print(command);
      Serial.println("]");
      
      digitalWrite(LED_PIN, HIGH); // 点亮指示灯

      if (command.length() > 0) {
        ServoCommand cmd;
        
        if (command == "R") {
          Serial.println("加入队列: [复位所有舵机]");
          // 创建复位命令并加入队列
          cmd.isReset = true;
          xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
        } else {
          // 解析舵机控制命令
          int colonPos = command.indexOf(':');
          if (colonPos != -1 && colonPos < command.length()-1) {
            String cmdType = command.substring(0, 1);
            
            // 验证命令类型
            if (cmdType != "J" && cmdType != "G") {
              Serial.println("无效的命令类型: " + cmdType);
              digitalWrite(LED_PIN, LOW);
              vTaskDelay(1);
              continue;
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
              Serial.println("无效的ID格式: " + idStr);
              digitalWrite(LED_PIN, LOW);
              vTaskDelay(1);
              continue;
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
              Serial.println("无效的位置格式: " + posStr);
              digitalWrite(LED_PIN, LOW);
              vTaskDelay(1);
              continue;
            }
            
            int position = posStr.toInt();
            
            // 打印解析结果用于调试
            Serial.print("解析命令: 类型=");
            Serial.print(cmdType);
            Serial.print(", ID=");
            Serial.print(id);
            Serial.print(", 位置=");
            Serial.println(position);
            
            // 确保位置在 0 到 1000 范围内
            position = constrain(position, 0, 1000);
            cmd.isReset = false;
            
            if (cmdType == "J" && id >= 0 && id < 7) {
              Serial.print("加入队列: [关节 ");
              Serial.print(id);
              Serial.print(" 移动到位置 ");
              Serial.print(position);
              Serial.println("]");
              
              // 创建关节舵机命令并加入队列
              cmd.servoId = jointServos[id];
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
              
              // 更新全局状态
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              Serial.print("加入队列: [夹持器移动到位置 ");
              Serial.print(position);
              Serial.println("]");
              
              // 创建夹持器命令并加入队列
              cmd.servoId = gripperServo;
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
              
              // 更新全局状态
              armStatus.gripper = position;
            } else {
              Serial.println("无效的指令类型或ID范围");
            }
          } else {
            Serial.println("命令格式错误");
          }
        }
        
        armStatus.updated = true;
        armStatus.lastCommandTime = millis();
      }
      
      digitalWrite(LED_PIN, LOW); // 关闭指示灯
    }
    
    // 检查动作组播放状态
    checkActionGroupStatus();
    
    vTaskDelay(1); // 最小延迟，提高串口响应速度
  }
}

// 处理所有舵机命令的任务 (网络和串口共用)
void servoControlTask(void *parameter) {
  ServoCommand cmd;
  // 限流变量，用于防止命令发送过快
  unsigned long lastCommandTime = 0;
  const unsigned long MIN_COMMAND_INTERVAL_MS = 50; // 最小命令间隔(ms)
  
  while (1) {
    unsigned long currentTime = millis();
    
    // 只有在没有命令执行且没有播放动作组且命令间隔足够时才获取新命令
    if (!isCommandExecuting && !isPlaying && 
        (currentTime - lastCommandTime >= MIN_COMMAND_INTERVAL_MS) && 
        xQueueReceive(servoCommandQueue, &cmd, 0) == pdTRUE) {
      
      // 设置命令正在执行标志
      isCommandExecuting = true;
      lastCommandTime = currentTime;
      
      if (cmd.isReset) {
        Serial.println("执行命令: [复位所有舵机]");
        // 处理复位命令
        resetServos();
      } else {
        Serial.print("执行命令: [舵机 ID=");
        Serial.print(cmd.servoId);
        Serial.print(" 位置=");
        Serial.print(cmd.position);
        Serial.println("]");
        
        // 处理普通舵机控制命令
        handleServoControl(cmd.servoId, cmd.position);
      }
    }
    
    // 检查当前命令是否执行完毕
    if (isCommandExecuting && currentTime >= servoLastMoveEndTime) {
      Serial.println("命令执行完毕，可以执行下一个命令");
      // 重置命令执行标志
      isCommandExecuting = false;
    }
    
    // 检查动作组播放状态
    checkActionGroupStatus();
    
    vTaskDelay(10); // 短暂延时
  }
}

// 显示和温度监测任务
void displayAndTemperatureTask(void *parameter) {
  while (1) {
    float ambientTemp = mlx.readAmbientTempC();
    armStatus.temperature = ambientTemp; // 更新温度数据
    displayMessage();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// 创建并启动所有任务
void setupTasks() {
  // 创建命令队列
  servoCommandQueue = xQueueCreate(queueSize, sizeof(ServoCommand));
  
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
    3,               // 提高任务优先级
    NULL,            // 任务句柄
    0                // 运行的核心（Core 0）- 修改到核心0，减少与网络任务竞争
  );

  // 修改舵机控制任务名称
  xTaskCreatePinnedToCore(
    servoControlTask,  // 任务函数
    "ServoControlTask", // 任务名称
    4096,              // 堆栈大小
    NULL,              // 任务参数
    2,                 // 优先级比控制任务低
    NULL,              // 任务句柄
    0                  // 运行的核心（Core 0）
  );

  // 创建屏幕显示与温度采集任务在核心1上运行
  xTaskCreatePinnedToCore(
    displayAndTemperatureTask, // 任务函数
    "DisplayAndTempTask",      // 任务名称
    4096,                      // 堆栈大小
    NULL,                      // 任务参数
    1,                         // 任务优先级
    NULL,                      // 任务句柄
    1                          // 修改到核心1，不干扰控制任务
  );
}
