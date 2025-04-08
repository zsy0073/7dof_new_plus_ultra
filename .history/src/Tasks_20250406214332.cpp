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
  // 限流变量，确保命令不会过快发送
  unsigned long lastCmdTime = 0;
  const unsigned long cmdInterval = 200; // 最小命令间隔(毫秒)
  
  while (1) {
    unsigned long currentTime = millis();
    
    // 检查是否有新命令，并保证命令有足够的间隔
    if (Serial1.available() > 0 && (currentTime - lastCmdTime >= cmdInterval)) {
      String command = Serial1.readStringUntil('\n');
      command.trim();
      
      Serial.print("收到指令: [");
      Serial.print(command);
      Serial.println("]");
      
      digitalWrite(LED_PIN, HIGH); // 点亮指示灯
      lastCmdTime = currentTime; // 更新上次命令时间

      if (command.length() > 0) {
        ServoCommand cmd;
        
        if (command == "R") {
          Serial.println("加入队列: [复位所有舵机]");
          // 复位命令
          cmd.isReset = true;
          xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
        } else {
          // 解析舵机控制命令
          int colonPos = command.indexOf(':');
          if (colonPos != -1) {
            String cmdType = command.substring(0, 1);
            
            // 简单验证 - 只接受J和G指令
            if (cmdType != "J" && cmdType != "G") {
              digitalWrite(LED_PIN, LOW);
              continue;
            }
            
            int id = command.substring(1, colonPos).toInt();
            int position = command.substring(colonPos + 1).toInt();
            
            // 确保位置在 0 到 1000 范围内
            position = constrain(position, 0, 1000);
            cmd.isReset = false;
            
            if (cmdType == "J" && id >= 0 && id < 7) {
              // 关节舵机命令
              cmd.servoId = jointServos[id];
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              // 夹持器命令
              cmd.servoId = gripperServo;
              cmd.position = position;
              xQueueSend(servoCommandQueue, &cmd, portMAX_DELAY);
              armStatus.gripper = position;
            }
          }
        }
        
        armStatus.updated = true;
        armStatus.lastCommandTime = currentTime;
      }
      
      digitalWrite(LED_PIN, LOW); // 关闭指示灯
    }
    
    // 检查动作组播放状态
    checkActionGroupStatus();
    
    vTaskDelay(10); // 增加延时，减少CPU占用
  }
}

// 处理所有舵机命令的任务 (网络和串口共用)
void servoControlTask(void *parameter) {
  ServoCommand cmd;
  unsigned long lastExecTime = 0;
  const unsigned long minExecInterval = 100; // 最小执行间隔(毫秒)
  
  while (1) {
    unsigned long currentTime = millis();
    
    // 命令执行频率限制，控制单个任务中命令的执行速率
    if (currentTime - lastExecTime >= minExecInterval) {
      // 只有在没有命令执行且没有播放动作组时才获取新命令
      if (!isCommandExecuting && !isPlaying && 
          xQueueReceive(servoCommandQueue, &cmd, 0) == pdTRUE) {
        
        // 设置命令正在执行标志
        isCommandExecuting = true;
        lastExecTime = currentTime;
        
        if (cmd.isReset) {
          // 处理复位命令
          resetServos();
        } else {
          // 处理普通舵机控制命令
          handleServoControl(cmd.servoId, cmd.position);
        }
      }
    }
    
    // 检查当前命令是否执行完毕
    if (isCommandExecuting && currentTime >= servoLastMoveEndTime) {
      // 重置命令执行标志
      isCommandExecuting = false;
    }
    
    vTaskDelay(20); // 增加延时，减少CPU占用
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
