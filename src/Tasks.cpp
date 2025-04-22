#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件

// 实例化机械臂状态
ArmStatus armStatus;

// 全局队列和状态变量
QueueHandle_t servoCommandQueue;
QueueHandle_t trajectoryCommandQueue;
const int queueSize = 20;  // 队列大小

// 状态标志
volatile bool isCommandExecuting = false;
volatile bool isTrajectoryRunning = false;

// 全局轨迹执行器
TrajectoryExecutor* trajectoryExecutor = nullptr;

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

// 处理所有舵机命令的任务 (网络和PS2手柄共用)
void servoControlTask(void *parameter) {
  ServoCommand cmd;
  
  while (1) {
    // 只有在没有命令执行且没有播放动作组时才获取新命令
    if (!isCommandExecuting && !isPlaying && xQueueReceive(servoCommandQueue, &cmd, 0) == pdTRUE) {
      // 设置命令正在执行标志
      isCommandExecuting = true;
      
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
    if (isCommandExecuting && millis() >= servoLastMoveEndTime) {
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

// 添加PS2控制器任务
void ps2ControllerTask(void *parameter) {
  // 初始化PS2控制器
  ps2Controller.init();
  
  // PS2控制循环
  while (1) {
    // 更新PS2控制器状态和处理控制命令
    ps2Controller.update();
    
    // 短暂延时以提高系统稳定性
    vTaskDelay(1); // 最小延迟
  }
}

// 轨迹执行任务
void trajectoryTask(void *parameter) {
    String cmdBuffer = "";
    
    while(1) {
        // 处理串口命令
        while(Serial.available()) {
            char c = Serial.read();
            if(c == '\n') {
                // 解析并执行轨迹命令
                if(cmdBuffer.startsWith("JOINT") || 
                   cmdBuffer.startsWith("LINE") || 
                   cmdBuffer.startsWith("ARC") || 
                   cmdBuffer.startsWith("PICK_PLACE")) {
                    
                    if(!isTrajectoryRunning && trajectoryExecutor != nullptr) {
                        TrajectoryCommand cmd;
                        if(trajectoryExecutor->parseCommand(cmdBuffer, cmd)) {
                            if(trajectoryExecutor->executeCommand(cmd)) {
                                isTrajectoryRunning = true;
                                Serial.println("轨迹执行开始");
                            } else {
                                Serial.println("轨迹执行失败");
                            }
                        } else {
                            Serial.println("命令格式错误");
                        }
                    } else {
                        Serial.println("上一个轨迹正在执行中");
                    }
                }
                cmdBuffer = "";
            } else {
                cmdBuffer += c;
            }
        }
        
        // 更新轨迹执行
        if(isTrajectoryRunning && trajectoryExecutor != nullptr) {
            trajectoryExecutor->update();
            
            // 检查轨迹是否完成
            if(trajectoryExecutor->isTrajectoryFinished()) {
                isTrajectoryRunning = false;
                Serial.println("轨迹执行完成");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms更新频率
    }
}

// 创建并启动所有任务
void setupTasks() {
  // 创建命令队列
  servoCommandQueue = xQueueCreate(queueSize, sizeof(ServoCommand));
  trajectoryCommandQueue = xQueueCreate(queueSize, sizeof(TrajectoryCommand));
  
  // 创建轨迹执行器实例
  static TrajectoryPlanner planner;
  static RobotKinematics kinematics;
  trajectoryExecutor = new TrajectoryExecutor(planner, kinematics);
  
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

  // 创建PS2控制器任务在核心0上运行，优先级高
  xTaskCreatePinnedToCore(
    ps2ControllerTask, // 任务函数
    "PS2ControlTask",  // 任务名称
    4096,             // 堆栈大小
    NULL,             // 任务参数
    3,                // 高优先级
    NULL,             // 任务句柄
    0                 // 运行的核心（Core 0）
  );

  // 舵机控制任务
  xTaskCreatePinnedToCore(
    servoControlTask,  // 任务函数
    "ServoControlTask", // 任务名称
    4096,              // 堆栈大小
    NULL,              // 任务参数
    1,                 // 优先级比PS2控制器任务低
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

  // 创建轨迹执行任务在核心1上运行
  xTaskCreatePinnedToCore(
    trajectoryTask,      // 任务函数
    "TrajectoryTask",    // 任务名称
    8192,               // 堆栈大小
    NULL,               // 任务参数
    1,                  // 任务优先级
    NULL,               // 任务句柄
    1                   // 运行的核心（Core 1）
  );
}
