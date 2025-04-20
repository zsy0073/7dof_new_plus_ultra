#include "Tasks.h"
#include "ActionGroup.h"  // 添加动作组头文件

// 实例化机械臂状态
ArmStatus armStatus;

// 实例化机械臂运动学对象
ArmKinematics armKinematics;

// 舵机控制队列
QueueHandle_t servoCommandQueue;
// 运动学命令队列
QueueHandle_t kinematicsCommandQueue;
const int queueSize = 20;  // 队列大小

// 命令状态跟踪
volatile bool isCommandExecuting = false;
// 运动学命令执行状态
volatile bool isKinematicsCommandExecuting = false;

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

// 运动学任务 - 处理机械臂运动学相关的指令
void kinematicsTask(void *parameter) {
  KinematicsCommand cmd;
  
  Serial.println("运动学任务已启动");
  
  // 当前关节角度（弧度）
  float currentJoints[ARM_DOF] = {0};
  
  while (1) {
    // 只有在没有命令执行时才获取新命令
    if (!isKinematicsCommandExecuting && xQueueReceive(kinematicsCommandQueue, &cmd, 0) == pdTRUE) {
      // 设置命令正在执行标志
      isKinematicsCommandExecuting = true;
      
      switch (cmd.type) {
        case MOVE_TO_POSITION: {
          Serial.println("执行运动学命令: [移动到指定位置]");
          Serial.printf("目标位置: X=%.2f Y=%.2f Z=%.2f\n", 
                        cmd.data.position.x, cmd.data.position.y, cmd.data.position.z);
          Serial.printf("目标姿态: RX=%.2f RY=%.2f RZ=%.2f\n", 
                        cmd.data.position.rx, cmd.data.position.ry, cmd.data.position.rz);
          
          // 创建目标位姿矩阵
          Matrix4x4 targetPose;
          ArmKinematics::createPoseMatrix(
            cmd.data.position.x, cmd.data.position.y, cmd.data.position.z,
            cmd.data.position.rx, cmd.data.position.ry, cmd.data.position.rz,
            &targetPose
          );
          
          // 计算逆运动学
          float resultJoints[ARM_DOF];
          int ikResult = armKinematics.inverseKinematics(&targetPose, currentJoints, 0, 0, resultJoints);
          
          if (ikResult == IK_SUCCESS) {
            // 逆解成功，保存当前关节角度
            memcpy(currentJoints, resultJoints, sizeof(currentJoints));
            
            // 转换到舵机命令并发送
            for (int i = 0; i < ARM_DOF; i++) {
              // 将弧度转换为舵机角度值（需要根据实际标定进行调整）
              int servoPosition = (int)(resultJoints[i] * 180.0f / M_PI) + 1500; // 假设中心位置为1500
              
              // 创建舵机命令
              ServoCommand servoCmd;
              servoCmd.servoId = i + 1; // 舵机ID从1开始
              servoCmd.position = servoPosition;
              servoCmd.isReset = false;
              
              // 发送到舵机控制队列
              xQueueSend(servoCommandQueue, &servoCmd, portMAX_DELAY);
              
              // 等待上一个舵机命令执行完成
              vTaskDelay(pdMS_TO_TICKS(50));
            }
            
            Serial.println("逆运动学解算成功并执行");
          } else {
            Serial.printf("逆运动学计算失败，错误码: %d\n", ikResult);
          }
          break;
        }
        
        case MOVE_TO_JOINTS: {
          Serial.println("执行运动学命令: [移动到指定关节角度]");
          
          // 保存当前关节角度
          memcpy(currentJoints, cmd.data.joints.joints, sizeof(currentJoints));
          
          // 输出关节角度
          Serial.print("目标关节角度(弧度): ");
          for (int i = 0; i < ARM_DOF; i++) {
            Serial.printf("%.2f ", currentJoints[i]);
          }
          Serial.println();
          
          // 转换到舵机命令并发送
          for (int i = 0; i < ARM_DOF; i++) {
            // 将弧度转换为舵机角度值
            int servoPosition = (int)(currentJoints[i] * 180.0f / M_PI) + 1500; // 假设中心位置为1500
            
            // 创建舵机命令
            ServoCommand servoCmd;
            servoCmd.servoId = i + 1; // 舵机ID从1开始
            servoCmd.position = servoPosition;
            servoCmd.isReset = false;
            
            // 发送到舵机控制队列
            xQueueSend(servoCommandQueue, &servoCmd, portMAX_DELAY);
            
            // 等待上一个舵机命令执行完成
            vTaskDelay(pdMS_TO_TICKS(50));
          }
          break;
        }
        
        case SET_VELOCITY: {
          Serial.printf("执行运动学命令: [设置速度] 线速度=%.2f 角速度=%.2f\n", 
                        cmd.data.velocity.linear, cmd.data.velocity.angular);
          // 这里可以实现速度控制逻辑
          break;
        }
        
        case ABORT_MOVEMENT: {
          Serial.println("执行运动学命令: [中止当前运动]");
          // 停止当前运动逻辑
          break;
        }
      }
      
      // 命令执行完毕
      isKinematicsCommandExecuting = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延时
  }
}

// 创建并启动所有任务
void setupTasks() {
  // 创建命令队列
  servoCommandQueue = xQueueCreate(queueSize, sizeof(ServoCommand));
  kinematicsCommandQueue = xQueueCreate(queueSize, sizeof(KinematicsCommand));
  
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
  
  // 创建运动学任务在核心0上运行
  xTaskCreatePinnedToCore(
    kinematicsTask,   // 任务函数
    "KinematicsTask", // 任务名称
    8192,            // 堆栈大小(运动学计算需要更多堆栈)
    NULL,            // 任务参数
    2,               // 任务优先级(比PS2控制器低，比舵机控制高)
    NULL,            // 任务句柄
    0                // 运行的核心（Core 0）
  );
}

// 辅助函数 - 移动到指定位置
bool moveToPosition(float x, float y, float z, float rx, float ry, float rz) {
  if (isKinematicsCommandExecuting) {
    return false; // 有命令在执行中，无法执行新命令
  }
  
  // 创建运动学命令
  KinematicsCommand cmd;
  cmd.type = MOVE_TO_POSITION;
  cmd.data.position.x = x;
  cmd.data.position.y = y;
  cmd.data.position.z = z;
  cmd.data.position.rx = rx;
  cmd.data.position.ry = ry;
  cmd.data.position.rz = rz;
  
  // 发送到运动学命令队列
  return xQueueSend(kinematicsCommandQueue, &cmd, 0) == pdTRUE;
}

// 辅助函数 - 移动到指定关节角度
bool moveToJoints(const float joints[ARM_DOF]) {
  if (isKinematicsCommandExecuting) {
    return false; // 有命令在执行中，无法执行新命令
  }
  
  // 创建运动学命令
  KinematicsCommand cmd;
  cmd.type = MOVE_TO_JOINTS;
  memcpy(cmd.data.joints.joints, joints, sizeof(float) * ARM_DOF);
  
  // 发送到运动学命令队列
  return xQueueSend(kinematicsCommandQueue, &cmd, 0) == pdTRUE;
}
