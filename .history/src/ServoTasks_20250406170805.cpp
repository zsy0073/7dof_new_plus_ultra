#include "ServoTasks.h"
#include "ActionGroup.h"

// 舵机任务句柄
TaskHandle_t servoTaskHandles[8]; // 7个关节 + 1个夹持器

// 舵机命令队列
QueueHandle_t servoQueues[8];     // 每个舵机的命令队列

// 舵机互斥锁，用于保护Serial2总线
SemaphoreHandle_t servoMutex = NULL;

// 舵机运动状态标志
bool servoMovingStatus[8] = {false};

// 任务同步事件组
EventGroupHandle_t servoEventGroup = NULL;
#define SERVO_COMPLETE_BIT(i) (1 << i)
#define ALL_SERVOS_COMPLETE 0xFF

// 舵机控制任务函数
void servoControlTask(void *parameter) {
  int servoIndex = (int)parameter;
  ServoTaskCommand cmd;
  int servoId = (servoIndex == 7) ? gripperServo : jointServos[servoIndex];
  
  Serial.printf("舵机任务 %d (ID=%d) 已启动在核心 %d\n", servoIndex, servoId, xPortGetCoreID());
  
  while (true) {
    // 等待队列中的命令
    if (xQueueReceive(servoQueues[servoIndex], &cmd, portMAX_DELAY) == pdTRUE) {
      // 设置舵机为运动状态
      servoMovingStatus[servoIndex] = true;
      
      // 清除完成位
      xEventGroupClearBits(servoEventGroup, SERVO_COMPLETE_BIT(servoIndex));
      
      int moveTime = cmd.moveTime;
      int targetPosition = cmd.position;
      
      switch (cmd.type) {
        case CMD_MOVE: {
          int currentPos = servoStates[servoIndex].currentPos;
          int distance = abs(targetPosition - currentPos);
          
          if (moveTime <= 0) {
            // 如果没有指定时间，计算一个适合的时间
            moveTime = calculateMoveTime(distance);
          }
          
          Serial.printf("舵机 %d: 移动从 %d 到 %d, 时间 %d ms\n", 
                      servoId, currentPos, targetPosition, moveTime);
          
          // 获取互斥锁
          if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // 执行舵机移动
            servos.moveServo(servoId, targetPosition, moveTime);
            xSemaphoreGive(servoMutex);
            
            // 更新舵机状态
            servoStates[servoIndex].currentPos = targetPosition;
            
            // 记录动作到动作组（如果正在记录）
            if (isRecording && millis() - lastActionTime >= ACTION_INTERVAL) {
              recordAction(servoId, targetPosition, moveTime);
              lastActionTime = millis();
            }
            
            // 更新界面状态
            if (servoIndex == 7) {
              armStatus.gripper = targetPosition;
            } else {
              armStatus.joints[servoIndex] = targetPosition;
            }
            armStatus.updated = true;
          }
          break;
        }
        
        case CMD_RESET: {
          int currentPos = servoStates[servoIndex].currentPos;
          int distance = abs(CENTER_POSITION - currentPos);
          
          if (moveTime <= 0) {
            moveTime = calculateMoveTime(distance);
          }
          
          Serial.printf("舵机 %d: 复位到 %d, 时间 %d ms\n", servoId, CENTER_POSITION, moveTime);
          
          if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            servos.moveServo(servoId, CENTER_POSITION, moveTime);
            xSemaphoreGive(servoMutex);
            
            servoStates[servoIndex].currentPos = CENTER_POSITION;
            
            if (servoIndex == 7) {
              armStatus.gripper = CENTER_POSITION;
            } else {
              armStatus.joints[servoIndex] = CENTER_POSITION;
            }
            armStatus.updated = true;
          }
          break;
        }
        
        case CMD_STOP:
          // 舵机没有停止命令，此处作为预留
          break;
      }
      
      // 等待移动完成的时间
      vTaskDelay(pdMS_TO_TICKS(moveTime + 50));
      
      // 设置完成标志
      servoMovingStatus[servoIndex] = false;
      xEventGroupSetBits(servoEventGroup, SERVO_COMPLETE_BIT(servoIndex));
      
      // 更新最后完成时间（用于向后兼容）
      if (cmd.notifyComplete) {
        servoLastMoveEndTime = millis();
      }
    }
  }
}

// 初始化舵机任务系统
void initServoTasks() {
  // 创建互斥锁
  servoMutex = xSemaphoreCreateMutex();
  
  // 创建事件组
  servoEventGroup = xEventGroupCreate();
  
  // 设置所有舵机初始状态为完成
  xEventGroupSetBits(servoEventGroup, ALL_SERVOS_COMPLETE);
  
  // 创建每个舵机的队列和任务
  for (int i = 0; i < 8; i++) {
    servoQueues[i] = xQueueCreate(5, sizeof(ServoTaskCommand));
    
    xTaskCreatePinnedToCore(
      servoControlTask,
      ("Servo" + String(i)).c_str(),
      2048,
      (void*)i,
      1, // 使用较低优先级
      &servoTaskHandles[i],
      0  // 在核心0上运行所有舵机任务
    );
  }
  
  Serial.println("舵机任务系统初始化完成");
}

// 异步移动单个舵机
void moveServoAsync(int servoId, int position) {
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  ServoTaskCommand cmd;
  cmd.type = CMD_MOVE;
  cmd.position = constrain(position, 0, 1000);
  cmd.moveTime = 0;  // 0表示自动计算
  cmd.notifyComplete = true;
  
  xQueueSend(servoQueues[index], &cmd, 0);
}

// 使用自定义参数移动舵机
void moveServoWithParamsAsync(int servoId, int position, int moveTime, bool waitComplete) {
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  ServoTaskCommand cmd;
  cmd.type = CMD_MOVE;
  cmd.position = constrain(position, 0, 1000);
  cmd.moveTime = moveTime;
  cmd.notifyComplete = waitComplete;
  
  xQueueSend(servoQueues[index], &cmd, 0);
  
  if (waitComplete) {
    // 等待此舵机完成
    xEventGroupWaitBits(
      servoEventGroup, 
      SERVO_COMPLETE_BIT(index),
      pdFALSE,   // 不清除位
      pdFALSE,   // 任意位满足即可
      portMAX_DELAY
    );
  }
}

// 复位单个舵机
void resetServoAsync(int servoId) {
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  ServoTaskCommand cmd;
  cmd.type = CMD_RESET;
  cmd.position = CENTER_POSITION;
  cmd.moveTime = 0;  // 自动计算
  cmd.notifyComplete = true;
  
  xQueueSend(servoQueues[index], &cmd, 0);
}

// 复位所有舵机
void resetAllServosAsync() {
  // 计算最远距离以获取统一的移动时间
  int maxDistance = 0;
  for (int i = 0; i < 8; i++) {
    int distance = abs(CENTER_POSITION - servoStates[i].currentPos);
    maxDistance = max(maxDistance, distance);
  }
  int moveTime = calculateMoveTime(maxDistance);
  
  // 如果正在记录动作组，则记录复位动作
  if (isRecording && millis() - lastActionTime >= ACTION_INTERVAL) {
    recordAction(RESET_ACTION_ID, 0, moveTime);
    lastActionTime = millis();
  }
  
  // 为所有舵机发送复位命令
  for (int i = 0; i < 8; i++) {
    ServoTaskCommand cmd;
    cmd.type = CMD_RESET;
    cmd.position = CENTER_POSITION;
    cmd.moveTime = moveTime; // 使用统一的时间
    cmd.notifyComplete = (i == 7); // 只在最后一个舵机通知完成
    
    xQueueSend(servoQueues[i], &cmd, 0);
  }
  
  // 更新预计完成时间（向后兼容）
  servoLastMoveEndTime = millis() + moveTime + 100;
}

// 等待所有舵机动作完成
bool waitForAllServosComplete(int timeoutMs) {
  EventBits_t bits = xEventGroupWaitBits(
    servoEventGroup,
    ALL_SERVOS_COMPLETE,
    pdFALSE,    // 不清除位
    pdTRUE,     // 所有位都必须满足
    pdMS_TO_TICKS(timeoutMs)
  );
  
  return (bits & ALL_SERVOS_COMPLETE) == ALL_SERVOS_COMPLETE;
}

// 检查特定舵机是否处于运动状态
bool isServoMoving(int servoId) {
  int index = getServoIndex(servoId);
  if (index < 0) return false;
  
  return servoMovingStatus[index];
}

// 检查是否有舵机正在运动
bool areAnyServosMoving() {
  for (int i = 0; i < 8; i++) {
    if (servoMovingStatus[i]) return true;
  }
  return false;
}
