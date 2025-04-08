#include "Tasks.h"

// 实例化机械臂状态
ArmStatus armStatus;

// 舵机控制队列
QueueHandle_t servoCommandQueue;
const int queueSize = 10;  // 队列大小

// 舵机命令结构体
struct ServoCommand {
  int servoId;
  int position;
  bool isReset;
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
          ServoCommand cmd = {0, 0, true};
          xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
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
            
            if (cmdType == "J" && id >= 0 && id < 7) {
              cmd.servoId = jointServos[id];
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              cmd.servoId = gripperServo;
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
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
    vTaskDelay(pdMS_TO_TICKS(5)); // 任务间隔
  }
}

// 专门处理舵机控制的任务
void servoControlTask(void *parameter) {
  ServoCommand cmd;
  
  while (1) {
    if (xQueueReceive(servoCommandQueue, &cmd, portMAX_DELAY)) {
      if (cmd.isReset) {
        // 处理复位命令
        resetServos();
      } else {
        // 处理普通舵机控制命令
        handleServoControl(cmd.servoId, cmd.position);
      }
    }
    
    // 在每次处理命令后更新舵机状态
    updateServoStatus();
    vTaskDelay(pdMS_TO_TICKS(10));
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

  // 创建舵机控制任务在核心0上运行
  xTaskCreatePinnedToCore(
    servoControlTask,  // 任务函数
    "ServoControlTask", // 任务名称
    4096,              // 堆栈大小
    NULL,              // 任务参数
    3,                 // 任务优先级（高于其他任务）
    NULL,              // 任务句柄
    0                  // 运行的核心（Core 0）
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
