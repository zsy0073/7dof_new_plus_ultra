#include "ServoControl.h"
#include "ActionGroup.h"

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 队列和任务句柄
QueueHandle_t servoQueues[MAX_SERVOS];
TaskHandle_t servoTasks[MAX_SERVOS];

// 舵机状态追踪
ServoState servoStates[MAX_SERVOS]; // 7个关节 + 1个夹持器
volatile bool servosBusy[MAX_SERVOS] = {false}; // 舵机忙碌状态

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 记录最后一次舵机移动预计完成的时间
unsigned long servoLastMoveEndTime = 0;

// 单个舵机的控制任务
void servoTask(void* parameters) {
  int servoIndex = (int)parameters;
  int servoId = (servoIndex < 7) ? jointServos[servoIndex] : gripperServo;
  ServoCommand_t cmd;
  
  Serial.print("舵机任务启动: 索引=");
  Serial.print(servoIndex);
  Serial.print(", ID=");
  Serial.println(servoId);
  
  while (1) {
    // 等待队列中的命令
    if (xQueueReceive(servoQueues[servoIndex], &cmd, portMAX_DELAY) == pdTRUE) {
      // 标记为忙
      servosBusy[servoIndex] = true;
      
      // 处理移动命令
      if (cmd.moveTime <= 0) {
        // 计算移动时间
        int distance = abs(cmd.position - servoStates[servoIndex].currentPos);
        cmd.moveTime = calculateMoveTime(distance);
      }
      
      // 发送舵机命令
      servos.moveServo(servoId, cmd.position, cmd.moveTime);
      
      // 打印舵机移动信息
      Serial.print("[舵机");
      Serial.print(servoIndex);
      Serial.print("(ID:");
      Serial.print(servoId);
      Serial.print(")] 移动: ");
      Serial.print(servoStates[servoIndex].currentPos);
      Serial.print(" -> ");
      Serial.print(cmd.position);
      Serial.print(", 时间: ");
      Serial.print(cmd.moveTime);
      Serial.println("ms");
      
      // 更新当前位置记录
      servoStates[servoIndex].currentPos = cmd.position;
      
      // 等待移动完成（带额外缓冲）
      vTaskDelay(pdMS_TO_TICKS(cmd.moveTime + 50));
      
      // 标记为不忙
      servosBusy[servoIndex] = false;
      
      // 记录动作组（如果正在记录）
      if (isRecording && (millis() - lastActionTime) >= ACTION_INTERVAL) {
        recordAction(servoId, cmd.position, cmd.moveTime);
        lastActionTime = millis();
      }
    }
  }
}

// 创建舵机控制任务
void createServoTasks() {
  // 为每个舵机创建队列和任务
  for (int i = 0; i < MAX_SERVOS; i++) {
    // 创建队列
    servoQueues[i] = xQueueCreate(5, sizeof(ServoCommand_t));
    
    // 创建任务
    char taskName[16];
    snprintf(taskName, sizeof(taskName), "Servo%d", i);
    
    xTaskCreatePinnedToCore(
      servoTask,           // 任务函数
      taskName,            // 任务名称
      2048,                // 堆栈大小
      (void*)i,            // 任务参数 - 舵机索引
      1,                   // 任务优先级
      &servoTasks[i],      // 任务句柄
      0                    // 运行的核心
    );
  }
}

// 初始化舵机
void initServo() {
  Serial.println("初始化舵机控制...");
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);
  
  // 创建舵机控制任务
  createServoTasks();
  
  // 等待所有舵机任务启动
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // 初始化所有舵机到中心位置
  resetServos();
  
  // 等待复位完成
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  Serial.println("所有舵机已初始化到中心位置");
}

// 根据移动距离计算移动时间
int calculateMoveTime(int distance) {
  if (distance < 50) {
    return 200; // 短距离用固定短时间
  } else if (distance < 200) {
    return 400; // 中距离用固定中等时间
  } else {
    return 600; // 长距离用固定长时间
  }
}

// 获取舵机索引
int getServoIndex(int servoId) {
  if (servoId == gripperServo) return 7;
  if (servoId >= 1 && servoId <= 7) return servoId - 1;
  return -1; // 无效ID
}

// 异步控制舵机移动
void moveServo(int servoId, int position, int moveTime) {
  // 约束位置范围
  position = constrain(position, 0, 1000);
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 准备命令
  ServoCommand_t cmd;
  cmd.position = position;
  cmd.moveTime = moveTime;
  cmd.timestamp = millis();
  
  // 发送命令到对应的舵机队列
  if (xQueueSend(servoQueues[index], &cmd, 0) != pdTRUE) {
    // 如果队列满了，先清空队列
    xQueueReset(servoQueues[index]);
    // 再次尝试发送
    xQueueSend(servoQueues[index], &cmd, 0);
  }
}

// 复位所有舵机到中心位置
void resetServos() {
  Serial.println("复位所有舵机到中心位置");
  
  // 同时发送所有舵机复位命令
  for (int i = 0; i < 7; i++) {
    // 发送复位命令到队列
    ServoCommand_t cmd;
    cmd.position = CENTER_POSITION;
    cmd.moveTime = 800; // 使用固定时间进行复位
    cmd.timestamp = millis();
    
    // 发送命令
    xQueueSend(servoQueues[i], &cmd, 0);
    
    // 更新全局状态
    armStatus.joints[i] = CENTER_POSITION;
  }
  
  // 发送夹持器复位命令
  ServoCommand_t cmd;
  cmd.position = CENTER_POSITION;
  cmd.moveTime = 800;
  cmd.timestamp = millis();
  xQueueSend(servoQueues[7], &cmd, 0);
  armStatus.gripper = CENTER_POSITION;
  
  // 标记为已更新
  armStatus.updated = true;
  
  // 如果正在记录动作组，记录复位动作
  if (isRecording && (millis() - lastActionTime) >= ACTION_INTERVAL) {
    recordAction(RESET_ACTION_ID, 0, 800);
    lastActionTime = millis();
  }
}

// 处理舵机控制
void handleServoControl(int servoId, int position) {
  // 如果正在播放动作组，则不处理外部舵机控制命令
  if (isPlaying) {
    return;
  }
  
  // 获取舵机索引
  int index = getServoIndex(servoId);
  if (index < 0) return;
  
  // 异步移动舵机
  moveServo(servoId, position);
  
  // 更新全局状态
  if (servoId == gripperServo) {
    armStatus.gripper = position;
  } else if (index >= 0 && index < 7) {
    armStatus.joints[index] = position;
  }
  
  // 标记为已更新
  armStatus.updated = true;
  armStatus.lastCommandTime = millis();
}
