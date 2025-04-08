#include "ServoTasks.h"

// 舵机任务句柄
TaskHandle_t servoTaskHandles[8]; // 7个关节 + 1个夹持器

// 舵机命令队列
QueueHandle_t servoQueues[8];     // 每个舵机的命令队列

// 舵机互斥锁，用于保护共享资源（如Serial2总线）
SemaphoreHandle_t allServosMutex = NULL;

// 舵机移动完成信号量组
SemaphoreHandle_t servoCompleteSemaphores[8];

// 舵机控制任务函数 - 每个舵机一个实例
void servoMoveTask(void *parameter) {
  int taskIndex = (int)parameter;
  int servoId = (taskIndex == 7) ? gripperServo : jointServos[taskIndex];
  ServoTaskParams params;
  
  Serial.print("舵机任务启动: ID=");
  Serial.println(servoId);
  
  while (true) {
    // 等待队列中的命令
    if (xQueueReceive(servoQueues[taskIndex], &params, portMAX_DELAY) == pdTRUE) {
      // 获取舵机参数
      int targetPosition = params.targetPosition;
      int moveTime = params.moveTime;
      SemaphoreHandle_t doneSemaphore = params.done;
      
      // 计算距离
      int currentPos = servoStates[taskIndex].currentPos;
      int distance = abs(targetPosition - currentPos);
      
      // 打印调试信息
      Serial.print("舵机异步移动: ID=");
      Serial.print(servoId);
      Serial.print(", 当前位置=");
      Serial.print(currentPos);
      Serial.print(", 目标位置=");
      Serial.print(targetPosition);
      Serial.print(", 时间=");
      Serial.print(moveTime);
      Serial.println("ms");
      
      // 获取互斥锁保护舵机总线
      if (xSemaphoreTake(allServosMutex, portMAX_DELAY) == pdTRUE) {
        // 执行舵机移动命令
        servos.moveServo(servoId, targetPosition, moveTime);
        
        // 释放互斥锁
        xSemaphoreGive(allServosMutex);
        
        // 更新舵机状态
        servoStates[taskIndex].currentPos = targetPosition;
        
        // 更新界面状态数据
        if (taskIndex == 7) {
          armStatus.gripper = targetPosition;
        } else {
          armStatus.joints[taskIndex] = targetPosition;
        }
        armStatus.updated = true;
        
        // 如果在记录模式，则记录动作
        if (isRecording && millis() - lastActionTime >= ACTION_INTERVAL) {
          recordAction(servoId, targetPosition, moveTime);
          lastActionTime = millis();
        }
        
        // 等待移动完成
        vTaskDelay(pdMS_TO_TICKS(moveTime + 20)); // 额外延迟确保动作完成
        
        // 如果提供了完成信号量，则发出信号
        if (doneSemaphore != NULL) {
          xSemaphoreGive(doneSemaphore);
        }
      }
    }
  }
}

// 设置舵机任务
void setupServoTasks() {
  // 创建互斥锁
  allServosMutex = xSemaphoreCreateMutex();
  
  // 创建每个舵机的队列和任务
  for (int i = 0; i < 8; i++) {
    // 创建命令队列
    servoQueues[i] = xQueueCreate(5, sizeof(ServoTaskParams));
    
    // 创建完成信号量
    servoCompleteSemaphores[i] = xSemaphoreCreateBinary();
    
    // 创建舵机控制任务
    xTaskCreatePinnedToCore(
      servoMoveTask,
      ("Servo" + String(i)).c_str(),
      2048,
      (void*)i,
      1,
      &servoTaskHandles[i],
      1  // 在核心1上运行所有舵机任务
    );
  }
  
  Serial.println("已设置所有舵机任务");
}

// 异步移动单个舵机
void moveServoAsync(int servoId, int position, int moveTime) {
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 确定舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 创建参数结构
  ServoTaskParams params;
  params.servoId = servoId;
  params.targetPosition = position;
  params.moveTime = moveTime;
  params.done = NULL; // 不需要等待完成
  
  // 发送到对应舵机的队列
  xQueueSend(servoQueues[index], &params, portMAX_DELAY);
}

// 等待所有舵机完成移动
void waitAllServosComplete() {
  // 创建临时信号量
  SemaphoreHandle_t allDone = xSemaphoreCreateCounting(8, 0);
  
  // 检查每个舵机队列是否为空
  for (int i = 0; i < 8; i++) {
    if (uxQueueMessagesWaiting(servoQueues[i]) == 0) {
      // 如果队列已空，增加计数
      xSemaphoreGive(allDone);
    } else {
      // 将完成信号量设置为最后一个命令
      ServoTaskParams params;
      
      // 清空队列，只保留最后一个命令
      while (uxQueueMessagesWaiting(servoQueues[i]) > 1) {
        xQueueReceive(servoQueues[i], &params, 0);
      }
      
      // 获取最后一个命令
      xQueuePeek(servoQueues[i], &params, 0);
      params.done = allDone;
      
      // 替换队列中的命令（先接收再发送）
      xQueueReceive(servoQueues[i], NULL, 0);
      xQueueSend(servoQueues[i], &params, 0);
    }
  }
  
  // 等待所有舵机完成
  for (int i = 0; i < 8; i++) {
    xSemaphoreTake(allDone, portMAX_DELAY);
  }
  
  // 删除临时信号量
  vSemaphoreDelete(allDone);
}

// 异步复位所有舵机
void resetServosAsync() {
  Serial.println("异步复位所有舵机");
  
  // 计算最大距离和移动时间
  int maxDistance = 0;
  for (int i = 0; i < 8; i++) {
    int currentPos = servoStates[i].currentPos;
    int distance = abs(CENTER_POSITION - currentPos);
    maxDistance = max(maxDistance, distance);
  }
  
  int moveTime = calculateMoveTime(maxDistance);
  
  // 如果在记录模式，记录复位动作
  if (isRecording && millis() - lastActionTime >= ACTION_INTERVAL) {
    recordAction(RESET_ACTION_ID, 0, moveTime);
    lastActionTime = millis();
  }
  
  // 为每个舵机发送复位命令
  for (int i = 0; i < 8; i++) {
    int servoId = (i == 7) ? gripperServo : jointServos[i];
    moveServoAsync(servoId, CENTER_POSITION, moveTime);
  }
}
