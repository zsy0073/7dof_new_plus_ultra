#include "Tasks.h"

// 实例化机械臂状态
ArmStatus armStatus;

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
    checkAndUpdateServoStatus(); // 定期检查舵机状态
    sendStatusUpdate(); // 发送状态更新
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(50)); // 减少任务延迟以提高响应速度
  }
}

// 添加命令队列来平滑处理连续命令
const int CMD_QUEUE_SIZE = 8;
struct CmdQueueItem {
  String command;
  unsigned long timestamp;
};
CmdQueueItem cmdQueue[CMD_QUEUE_SIZE];
int cmdQueueHead = 0;
int cmdQueueTail = 0;
SemaphoreHandle_t cmdQueueMutex = NULL;

// 添加命令到队列
void addCommandToQueue(String command) {
  if (xSemaphoreTake(cmdQueueMutex, portMAX_DELAY)) {
    cmdQueue[cmdQueueTail].command = command;
    cmdQueue[cmdQueueTail].timestamp = millis();
    cmdQueueTail = (cmdQueueTail + 1) % CMD_QUEUE_SIZE;
    // 如果队列满，覆盖最老的命令
    if (cmdQueueTail == cmdQueueHead) {
      cmdQueueHead = (cmdQueueHead + 1) % CMD_QUEUE_SIZE;
    }
    xSemaphoreGive(cmdQueueMutex);
  }
}

// 处理队列中的命令
void processCommandQueue() {
  if (xSemaphoreTake(cmdQueueMutex, portMAX_DELAY)) {
    unsigned long currentTime = millis();
    
    // 处理队列中的所有命令，但最多处理一个命令以避免阻塞太久
    if (cmdQueueHead != cmdQueueTail) {
      String command = cmdQueue[cmdQueueHead].command;
      unsigned long cmdTime = cmdQueue[cmdQueueHead].timestamp;
      cmdQueueHead = (cmdQueueHead + 1) % CMD_QUEUE_SIZE;
      xSemaphoreGive(cmdQueueMutex);
      
      // 点亮板载LED
      digitalWrite(LED_PIN, HIGH);

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
      armStatus.lastCommandTime = currentTime;
      
      // 立即发送状态更新到网页
      String json = "{\"temp\":" + String(armStatus.temperature, 1) + 
                   ",\"joints\":[";
      for (int i = 0; i < 7; i++) {
        json += String(armStatus.joints[i]);
        if (i < 6) json += ",";
      }
      json += "],\"gripper\":" + String(armStatus.gripper) + "}";
      ws.textAll(json);
      
      // 处理完成后关闭LED
      digitalWrite(LED_PIN, LOW);
    } else {
      xSemaphoreGive(cmdQueueMutex);
    }
  }
}

//controllerTask 是一个用于处理串口命令的任务函数
void controllerTask(void *parameter) {
  // 创建命令队列互斥锁
  cmdQueueMutex = xSemaphoreCreateMutex();
  
  while (1) {
    if (Serial1.available() > 0) {
      String command = Serial1.readStringUntil('\n'); // 按行读取命令
      command.trim(); // 去除可能的换行符或空格
      
      if (command.length() > 0) {
        // 将命令添加到队列而不是直接处理
        addCommandToQueue(command);
      }
    }
    
    // 处理队列中的命令
    processCommandQueue();
    
    checkAndUpdateServoStatus(); // 定期检查舵机状态
    vTaskDelay(pdMS_TO_TICKS(5)); // 减少任务延迟以提高响应速度
  }
}

//用于显示温度和学生信息以及指令信息的任务函数
void displayAndTemperatureTask(void *parameter) {
  while (1) {
    float ambientTemp = mlx.readAmbientTempC();
    armStatus.temperature = ambientTemp;
    displayMessage();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// 创建并启动所有任务
void setupTasks() {
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

  // 创建手柄控制任务在核心1上运行
  xTaskCreatePinnedToCore(
    controllerTask,  // 任务函数
    "ControllerTask", // 任务名称
    4096,            // 堆栈大小
    NULL,            // 任务参数
    2,               // 任务优先级
    NULL,            // 任务句柄
    1                // 运行的核心（Core 1）
  );

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
