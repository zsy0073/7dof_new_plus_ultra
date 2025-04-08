#include "Tasks.h"

// 实例化机械臂状态
ArmStatus armStatus;

// 舵机控制队列
QueueHandle_t servoCommandQueue;
const int queueSize = 20;  // 扩大队列大小，避免命令丢失

// 舵机命令结构体
struct ServoCommand {
  int servoId;
  int position;
  bool isReset;
  unsigned long timestamp;  // 增加时间戳，用于平滑控制
};

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
      
      // 点亮板载LED指示命令处理
      digitalWrite(LED_PIN, HIGH);

      if (command.length() > 0) {
        if (command == "R") {
          // 创建一个复位命令
          ServoCommand cmd = {0, 0, true, millis()};
          xQueueSend(servoCommandQueue, &cmd, 0); // 使用非阻塞发送，避免卡住串口接收
        } else {
          // 解析舵机控制命令
          int colonPos = command.indexOf(':');
          if (colonPos != -1) {
            String cmdType = command.substring(0, 1);
            int id = command.substring(1, colonPos).toInt();
            int position = command.substring(colonPos + 1).toInt();
            
            // 确保位置在 0 到 1000 范围内
            position = constrain(position, 0, 1000);

            ServoCommand cmd;
            cmd.isReset = false;
            cmd.timestamp = millis();
            
            if (cmdType == "J" && id >= 0 && id < 7) {
              cmd.servoId = jointServos[id];
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, 0); // 非阻塞发送
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              cmd.servoId = gripperServo;
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, 0); // 非阻塞发送
              armStatus.gripper = position;
            }
          }
        }
        
        armStatus.updated = true;
        armStatus.lastCommandTime = millis();
      }
      
      // 处理完成后关闭LED
      digitalWrite(LED_PIN, LOW);
    }
    // 减少任务执行间隔，提高串口响应速度
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// 专门处理舵机控制的任务
void servoControlTask(void *parameter) {
  ServoCommand cmd;
  unsigned long lastExecutionTime = 0;
  
  // 命令合并缓冲区，只保留每个舵机的最新命令
  ServoCommand latestCommands[8];  // 7个关节 + 1个夹持器
  bool hasNewCommand[8] = {false};
  
  for (int i = 0; i < 8; i++) {
    latestCommands[i].servoId = (i < 7) ? jointServos[i] : gripperServo;
    latestCommands[i].position = (i < 7) ? armStatus.joints[i] : armStatus.gripper;
    latestCommands[i].isReset = false;
    latestCommands[i].timestamp = 0;
  }
  
  while (1) {
    // 先清空队列中的所有命令，将同一舵机的多个命令合并为最新的一个
    while (xQueueReceive(servoCommandQueue, &cmd, 0) == pdTRUE) {
      if (cmd.isReset) {
        // 复位命令直接执行，不进行合并
        resetServos();
        
        // 清空命令缓冲区
        for (int i = 0; i < 8; i++) {
          hasNewCommand[i] = false;
        }
        
        // 更新最后执行时间
        lastExecutionTime = millis();
        break;  // 执行复位后退出当前循环
      } else {
        // 获取舵机索引
        int index = getServoIndex(cmd.servoId);
        if (index >= 0 && index < 8) {
          // 更新缓冲区中的命令
          latestCommands[index] = cmd;
          hasNewCommand[index] = true;
        }
      }
    }
    
    // 执行所有有新命令的舵机
    unsigned long currentTime = millis();
    // 每15ms执行一批命令，避免频繁发送导致的问题
    if (currentTime - lastExecutionTime >= 15) {
      bool commandExecuted = false;
      
      for (int i = 0; i < 8; i++) {
        if (hasNewCommand[i]) {
          // 使用moveServoSmoothly进行平滑控制
          moveServoSmoothly(latestCommands[i].servoId, latestCommands[i].position);
          hasNewCommand[i] = false;
          commandExecuted = true;
        }
      }
      
      if (commandExecuted) {
        lastExecutionTime = currentTime;
      }
    }
    
    // 更新舵机状态
    updateServoStatus();
    
    // 适当延时，避免占用太多CPU时间
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// 显示和温度监测任务
void displayAndTemperatureTask(void *parameter) {
  while (1) {
    float ambientTemp = mlx.readAmbientTempC();
    armStatus.temperature = ambientTemp; // 更新温度数据
    displayMessage();
    vTaskDelay(pdMS_TO_TICKS(2000)); // 温度和显示更新间隔2秒
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
    2,              // 任务优先级（提高优先级）
    NULL,           // 任务句柄
    1               // 运行的核心（Core 1）
  );

  // 创建手柄控制任务在核心1上运行
  xTaskCreatePinnedToCore(
    controllerTask,  // 任务函数
    "ControllerTask", // 任务名称
    4096,            // 堆栈大小
    NULL,            // 任务参数
    3,               // 任务优先级（最高优先级）
    NULL,            // 任务句柄
    1                // 运行的核心（Core 1）
  );

  // 创建舵机控制任务在核心0上运行
  xTaskCreatePinnedToCore(
    servoControlTask,  // 任务函数
    "ServoControlTask", // 任务名称
    4096,              // 堆栈大小
    NULL,              // 任务参数
    3,                 // 任务优先级（同样高优先级）
    NULL,              // 任务句柄
    0                  // 运行的核心（Core 0）
  );

  // 创建屏幕显示与温度采集任务在核心0上运行
  xTaskCreatePinnedToCore(
    displayAndTemperatureTask, // 任务函数
    "DisplayAndTempTask",      // 任务名称
    4096,                      // 堆栈大小
    NULL,                      // 任务参数
    1,                         // 任务优先级（低优先级）
    NULL,                      // 任务句柄
    0                          // 运行的核心（Core 0）
  );
}
