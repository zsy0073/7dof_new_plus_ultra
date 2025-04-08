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
    sendStatusUpdate(); // 发送状态更新
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(50)); // 更新间隔
  }
}

// 处理串口命令的任务函数
void controllerTask(void *parameter) {
  while (1) {
    if (Serial1.available() > 0) {
      String command = Serial1.readStringUntil('\n'); // 按行读取命令
      command.trim(); // 去除可能的换行符或空格
      
      // 添加命令节流，避免命令发送过快
      unsigned long currentTime = millis();
      if (currentTime - armStatus.lastCommandTime < MIN_COMMAND_INTERVAL) {
        vTaskDelay(pdMS_TO_TICKS(MIN_COMMAND_INTERVAL - (currentTime - armStatus.lastCommandTime)));
      }
      
      if (command.length() > 0) {
        // 点亮板载LED指示命令处理
        digitalWrite(LED_PIN, HIGH);

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
              // 控制关节舵机
              handleServoControl(jointServos[id], position);
            } else if (cmdType == "G") {
              // 控制夹持器舵机
              handleServoControl(gripperServo, position);
            }
          }
        }
        
        // 处理完成后关闭LED
        digitalWrite(LED_PIN, LOW);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); // 任务间隔
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

  // 创建屏幕显示与温度采集任务在核心1上运行 (改为核心1，因为舵机控制在核心0)
  xTaskCreatePinnedToCore(
    displayAndTemperatureTask, // 任务函数
    "DisplayAndTempTask",      // 任务名称
    4096,                      // 堆栈大小
    NULL,                      // 任务参数
    1,                         // 任务优先级
    NULL,                      // 任务句柄
    1                          // 运行的核心（Core 1）
  );
}
