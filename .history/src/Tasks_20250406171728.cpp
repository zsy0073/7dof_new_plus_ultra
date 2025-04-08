#include "Tasks.h"
#include "ActionGroup.h"  

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
        if (command == "R") {
          Serial.println("执行: [复位所有舵机]");
          // 直接调用复位函数，不再使用队列
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
              Serial.print("执行: [关节 ");
              Serial.print(id);
              Serial.print(" 移动到位置 ");
              Serial.print(position);
              Serial.println("]");
              
              // 直接调用舵机控制函数
              moveServo(jointServos[id], position);
              
              // 更新全局状态
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              Serial.print("执行: [夹持器移动到位置 ");
              Serial.print(position);
              Serial.println("]");
              
              // 直接调用舵机控制函数
              moveServo(gripperServo, position);
              
              // 更新全局状态
              armStatus.gripper = position;
            } else {
              Serial.println("无效的指令类型");
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

  // 创建屏幕显示与温度采集任务在核心1上运行
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
