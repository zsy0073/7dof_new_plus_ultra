#include <Arduino.h>
#include <LobotServoController.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "webpage.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// 定义板载LED引脚
#define LED_PIN 4

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 定义舵机运动时间（单位：毫秒）
#define SERVO_MOVE_TIME 500
// 添加舵机移动参数
#define MIN_MOVE_TIME 200  // 最小移动时间
#define MAX_MOVE_TIME 1500 // 最大移动时间
#define DISTANCE_FACTOR 1  // 降低每单位距离的毫秒数 (从2改为1)
// 添加命令处理间隔控制
#define MIN_COMMAND_INTERVAL 20  // 减少命令间隔(从50ms改为20ms)
// 添加平滑移动参数
#define SMOOTH_STEPS 3     // 减少步骤数量(从5改为3)
#define SMOOTH_DELAY 80    // 减少每步延迟(从100ms改为80ms)
// 添加加速减速参数
#define ACCEL_STEPS 8      // 减少加速步数(从15改为8)
#define ACCEL_DELAY_MAX 100 // 减少最大延迟(从150ms改为100ms)
#define ACCEL_DELAY_MIN 30  // 减少最小延迟(从50ms改为30ms)
// 添加非阻塞控制参数
#define USE_NON_BLOCKING_MOVES true // 启用非阻塞移动

// 定义串口引脚
#define HANDLE_SERIAL_RX_PIN 18
#define HANDLE_SERIAL_TX_PIN 19
#define SERVO_SERIAL_RX_PIN 16
#define SERVO_SERIAL_TX_PIN 17
// 定义 I2C 引脚
#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 14
// 定义调试开关和日志级别
#define DEBUG_MODE true
#define LOG_INFO(msg) // 移除日志
#define LOG_ERROR(msg) // 移除日志
#define LOG_DEBUG(msg, val) // 移除日志

// WiFi配置
const char* ssid = "404notfound";
const char* password = "404notfound";

// 创建Web服务器和WebSocket对象
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// OLED 显示屏和温度传感器对象
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// 学号和姓名
const char* studentID = "20250001";
const char* studentName = "ZSY";

// 定义机械臂状态结构体
struct {
  float temperature = 0.0;
  int joints[7] = {500, 500, 500, 500, 500, 500, 500}; // 初始化为中心位置
  int gripper = 500; // 初始化为中心位置
  bool updated = false;
  unsigned long lastCommandTime = 0; // 添加最后一次命令时间记录
} armStatus;

// 非阻塞舵机控制状态结构
struct ServoMoveStatus {
  bool isMoving = false;
  unsigned long moveStartTime = 0;
  unsigned long moveDuration = 0;
  int targetPosition = 0;
  int servoId = 0;
};

ServoMoveStatus servoStatus[8]; // 7个关节 + 1个夹持器

// 计算适合的移动时间
int calculateMoveTime(int currentPos, int targetPos) {
  int distance = abs(targetPos - currentPos);
  int moveTime = distance * DISTANCE_FACTOR;
  
  // 确保时间在合理范围内
  moveTime = constrain(moveTime, MIN_MOVE_TIME, MAX_MOVE_TIME);
  
  return moveTime;
}

// 计算S曲线速度值 (0.0 - 1.0)
float calculateSCurveValue(int step, int totalSteps) {
  float x = (float)step / totalSteps;
  // 使用三次函数来生成S曲线: f(x) = -2x³ + 3x²
  // 这个函数在 x=0 处的值是0，在 x=1 处的值是1，且在两端的导数为0
  return -2.0 * x * x * x + 3.0 * x * x;
}

// 计算每步延迟时间，实现加速减速
int calculateStepDelay(int step, int totalSteps) {
  // 使用S曲线速度曲线计算延迟
  float speed;
  
  // 前半部分是加速阶段（步数从0增加）
  if (step < totalSteps / 2) {
    speed = calculateSCurveValue(step, totalSteps / 2);
  }
  // 后半部分是减速阶段（步数从最大减少）
  else {
    speed = calculateSCurveValue(totalSteps - step - 1, totalSteps / 2);
  }
  
  // 将速度转换为延迟时间（速度大时延迟小）
  float delay = ACCEL_DELAY_MAX - (ACCEL_DELAY_MAX - ACCEL_DELAY_MIN) * speed;
  
  return (int)delay;
}

// 非阻塞延迟函数
void nonBlockingDelay(unsigned long delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
    // 允许其他任务执行
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// 检查舵机是否仍在移动
void checkAndUpdateServoStatus() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 8; i++) {
    if (servoStatus[i].isMoving) {
      if (currentTime - servoStatus[i].moveStartTime >= servoStatus[i].moveDuration) {
        // 移动完成
        servoStatus[i].isMoving = false;
      }
    }
  }
}

// 启动舵机移动，但不等待
void startServoMove(int servoId, int position, int moveTime) {
  servos.moveServo(servoId, position, moveTime);
  
  // 更新状态
  int index = servoId - 1;
  if (servoId == gripperServo) index = 7;
  
  servoStatus[index].isMoving = true;
  servoStatus[index].moveStartTime = millis();
  servoStatus[index].moveDuration = moveTime;
  servoStatus[index].targetPosition = position;
  servoStatus[index].servoId = servoId;
}

// 带匀加速匀减速的平滑移动函数
void smoothAccelMoveServo(int servoId, int currentPos, int targetPos) {
  int distance = abs(targetPos - currentPos);
  
  // 对于小范围移动，直接使用计算的移动时间
  if (distance < 50) {
    int moveTime = calculateMoveTime(currentPos, targetPos);
    servos.moveServo(servoId, targetPos, moveTime);
    return;
  }
  
  // 计算步进值
  int direction = (targetPos > currentPos) ? 1 : -1;
  int stepSize = max(1, distance / ACCEL_STEPS);
  int position = currentPos;
  
  // 执行加速减速移动
  for (int i = 0; i < ACCEL_STEPS; i++) {
    // 计算本步移动距离
    int nextPosition;
    
    // 最后一步确保到达目标位置
    if (i == ACCEL_STEPS - 1) {
      nextPosition = targetPos;
    } else {
      nextPosition = position + direction * stepSize;
      
      // 确保不会越过目标位置
      if ((direction > 0 && nextPosition > targetPos) ||
          (direction < 0 && nextPosition < targetPos)) {
        nextPosition = targetPos;
      }
    }
    
    // 计算本步延迟时间（基于S曲线）
    int stepDelay = calculateStepDelay(i, ACCEL_STEPS);
    
    // 移动到新位置
    servos.moveServo(servoId, nextPosition, stepDelay);
    
    // 等待移动完成
    delay(stepDelay);
    
    position = nextPosition;
    
    // 如果已经到达目标位置，提前结束循环
    if (position == targetPos) break;
  }
}

// 优化的平滑加速移动函数 - 非阻塞版本
void smoothAccelMoveServoNonBlocking(int servoId, int currentPos, int targetPos) {
  int distance = abs(targetPos - currentPos);
  
  // 对于小范围移动，直接使用计算的移动时间
  if (distance < 30) { // 减小阈值以加快小动作响应
    int moveTime = max(100, calculateMoveTime(currentPos, targetPos)); // 确保至少有100ms移动时间
    startServoMove(servoId, targetPos, moveTime);
    return;
  }
  
  // 计算步进值
  int direction = (targetPos > currentPos) ? 1 : -1;
  int stepSize = max(5, distance / ACCEL_STEPS); // 增大最小步长以加快响应
  int position = currentPos;
  
  // 执行首段加速移动
  int nextPosition = position + direction * stepSize * 2; // 第一步加大步长
  if ((direction > 0 && nextPosition > targetPos) ||
      (direction < 0 && nextPosition < targetPos)) {
    nextPosition = targetPos;
  }
  
  // 使用较短的延迟启动移动
  startServoMove(servoId, nextPosition, ACCEL_DELAY_MIN);
  
  // 立即返回，不阻塞进程
}

// 带匀加速匀减速的平滑复位函数
void smoothAccelResetServos() {
  // 记录当前位置
  int currentPositions[8];
  for (int i = 0; i < 7; i++) {
    currentPositions[i] = armStatus.joints[i];
  }
  currentPositions[7] = armStatus.gripper;
  
  // 计算每个舵机到中心位置的总距离
  int maxDistance = 0;
  for (int i = 0; i < 7; i++) {
    maxDistance = max(maxDistance, abs(500 - currentPositions[i]));
  }
  maxDistance = max(maxDistance, abs(500 - currentPositions[7]));
  
  if (maxDistance < 50) {
    // 对于小范围移动，直接复位
    for (int i = 0; i < 7; i++) {
      servos.moveServo(jointServos[i], 500, 500);
      armStatus.joints[i] = 500;
    }
    servos.moveServo(gripperServo, 500, 500);
    armStatus.gripper = 500;
    return;
  }
  
  // 计算每步的大小
  float stepRatio = (float)maxDistance / ACCEL_STEPS;
  
  // 逐步移动所有舵机，实现匀加速匀减速
  for (int step = 1; step <= ACCEL_STEPS; step++) {
    // 计算S曲线插值系数 (0.0 - 1.0)
    float progressRatio;
    
    if (step <= ACCEL_STEPS / 2) {
      // 加速阶段
      progressRatio = calculateSCurveValue(step, ACCEL_STEPS / 2);
    } else {
      // 减速阶段
      progressRatio = calculateSCurveValue(ACCEL_STEPS - step, ACCEL_STEPS / 2);
    }
    
    float progress = step * stepRatio;
    
    // 计算延迟时间
    int stepDelay = calculateStepDelay(step - 1, ACCEL_STEPS);
    
    // 同时移动所有关节舵机
    for (int i = 0; i < 7; i++) {
      int distance = 500 - currentPositions[i];
      int targetPos = currentPositions[i] + (int)(distance * step / ACCEL_STEPS);
      servos.moveServo(jointServos[i], targetPos, stepDelay);
    }
    
    // 移动夹持器舵机
    int gripperDistance = 500 - currentPositions[7];
    int gripperTargetPos = currentPositions[7] + (int)(gripperDistance * step / ACCEL_STEPS);
    servos.moveServo(gripperServo, gripperTargetPos, stepDelay);
    
    // 等待本步完成
    delay(stepDelay);
  }
  
  // 最终确保精确到达中心位置
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], 500, 200);
    armStatus.joints[i] = 500;
  }
  servos.moveServo(gripperServo, 500, 200);
  armStatus.gripper = 500;
}

// 优化的平滑复位函数 - 减少阻塞
void smoothAccelResetServosOptimized() {
  // 记录当前位置
  int currentPositions[8];
  for (int i = 0; i < 7; i++) {
    currentPositions[i] = armStatus.joints[i];
  }
  currentPositions[7] = armStatus.gripper;
  
  // 计算每个舵机到中心位置的总距离
  int maxDistance = 0;
  for (int i = 0; i < 7; i++) {
    maxDistance = max(maxDistance, abs(500 - currentPositions[i]));
  }
  maxDistance = max(maxDistance, abs(500 - currentPositions[7]));
  
  if (maxDistance < 30) { // 减小阈值以加快小动作响应
    // 对于小范围移动，直接复位
    for (int i = 0; i < 7; i++) {
      startServoMove(jointServos[i], 500, 300); // 减少移动时间
      armStatus.joints[i] = 500;
    }
    startServoMove(gripperServo, 500, 300);
    armStatus.gripper = 500;
    nonBlockingDelay(300); // 等待所有舵机完成移动，但使用非阻塞延迟
    return;
  }
  
  // 仅执行关键的几个步骤而不是全部步骤
  int steps[3] = {1, ACCEL_STEPS/2, ACCEL_STEPS}; // 只在开始、中间、结束发送命令
  
  for (int stepIdx = 0; stepIdx < 3; stepIdx++) {
    int step = steps[stepIdx];
    int stepDelay = (stepIdx == 1) ? ACCEL_DELAY_MIN : ACCEL_DELAY_MAX / 2;
    
    // 同时移动所有关节舵机
    for (int i = 0; i < 7; i++) {
      int distance = 500 - currentPositions[i];
      int targetPos = currentPositions[i] + (int)(distance * step / ACCEL_STEPS);
      startServoMove(jointServos[i], targetPos, stepDelay);
    }
    
    // 移动夹持器舵机
    int gripperDistance = 500 - currentPositions[7];
    int gripperTargetPos = currentPositions[7] + (int)(gripperDistance * step / ACCEL_STEPS);
    startServoMove(gripperServo, gripperTargetPos, stepDelay);
    
    // 使用非阻塞延迟
    nonBlockingDelay(stepDelay + 20); // 添加一点额外时间确保命令被处理
  }
  
  // 更新状态
  for (int i = 0; i < 7; i++) {
    armStatus.joints[i] = 500;
  }
  armStatus.gripper = 500;
}

//显示函数
void displayMessage() {
  // 读取环境温度
  float ambientTemp = mlx.readAmbientTempC();
  
  // 调试输出温度值
  // Serial.print("[DEBUG] Ambient Temp: ");
  // Serial.println(ambientTemp);

  // 在OLED上显示温度
  display.clearDisplay();
  display.setCursor(0, 0);
  
  if (isnan(ambientTemp)) {
    display.println("Sensor Error!");
  } else {
    display.print("Ambient Temp: ");
    display.print(ambientTemp);
    display.print(" C");
  }

  // 显示学号和姓名
  display.setCursor(0, 15);
  display.print("ID: ");
  display.print(studentID);
  display.setCursor(0, 25);
  display.print("Name: ");
  display.print(studentName);

  // 显示舵机状态
  display.setCursor(0, 35);
  display.print("J0-J2: ");
  for(int i=0; i<3;i++){
    display.print(armStatus.joints[i]);
    display.print(" ");
  }
  
  display.setCursor(0, 45);
  display.print("J3-J6: ");
  for(int i=3; i<7; i++){
    display.print(armStatus.joints[i]);
    display.print(" ");
  }
  display.print(" G:");
  display.print(armStatus.gripper);

  display.display(); // 确保更新显示
}

// 初始化显示屏和传感器
void initDisplayAndSensor() {
  if (!mlx.begin()) { // 使用默认I2C地址初始化 MLX90614
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 使用默认I2C地址初始化 OLED
    while (1);
  }
  // 清除屏幕并显示初始化完成信息
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("ready!");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  displayMessage(); // 添加调用displayMessage函数更新显示内容
}

// 网络服务任务
void networkTask(void *parameter) {
  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // 配置WebSocket事件处理
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
      case WS_EVT_CONNECT:
        break;
      case WS_EVT_DISCONNECT:
        break;
      case WS_EVT_DATA: {
        String command = String((char*)data).substring(0, len);
        // Serial.print("Received command: "); // 注释掉调试信息
        // Serial.println(command);
        
        if (command == "R") {
          // 使用优化的复位功能
          smoothAccelResetServosOptimized();
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
              // 使用非阻塞移动函数
              smoothAccelMoveServoNonBlocking(jointServos[id], armStatus.joints[id], position);
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              // 使用非阻塞移动函数
              smoothAccelMoveServoNonBlocking(gripperServo, armStatus.gripper, position);
              armStatus.gripper = position;
            }
          }
        }
        armStatus.updated = true;
        armStatus.lastCommandTime = millis();
        break;
      }
    }
  });

  // 配置Web服务器路由
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PAGE_HTML);
  });

  // 启动服务器
  server.begin();

  // 状态更新循环
  while (1) {
    checkAndUpdateServoStatus(); // 定期检查舵机状态
    
    if (armStatus.updated) {
      String json = "{\"temp\":" + String(armStatus.temperature, 1) + 
                   ",\"joints\":[";
      for (int i = 0; i < 7; i++) {
        json += String(armStatus.joints[i]);
        if (i < 6) json += ",";
      }
      json += "],\"gripper\":" + String(armStatus.gripper) + "}";
      ws.textAll(json);
      armStatus.updated = false;
    }
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(50)); // 减少任务延迟以提高响应速度
  }
}

//controllerTask 是一个用于处理串口命令的任务函数。它不断读取串口输入，根据接收到的命令控制舵机的位置，并更新机械臂的状态。
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
        // Serial.print("Received command: "); // 注释掉调试信息
        // Serial.println(command);

        // 点亮板载LED
        digitalWrite(LED_PIN, HIGH);

        if (command == "R") {
          // 使用优化的复位功能
          smoothAccelResetServosOptimized();
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
              // 使用非阻塞移动函数
              smoothAccelMoveServoNonBlocking(jointServos[id], armStatus.joints[id], position);
              armStatus.joints[id] = position;
            } else if (cmdType == "G") {
              // 使用非阻塞移动函数
              smoothAccelMoveServoNonBlocking(gripperServo, armStatus.gripper, position);
              armStatus.gripper = position;
            }
          }
        }
        armStatus.updated = true;
        armStatus.lastCommandTime = millis();
        
        // 立即发送状态更新到网页
        String json = "{\"temp\":" + String(armStatus.temperature, 1) + 
                     ",\"joints\":[";
        for (int i = 0; i < 7; i++) {
          json += String(armStatus.joints[i]);
          if (i < 6) json += ",";
        }
        json += "],\"gripper\":" + String(armStatus.gripper) + "}";
        ws.textAll(json);
        
        // 处理完成后关闭LED
        digitalWrite(LED_PIN, LOW);
      }
    }
    
    checkAndUpdateServoStatus(); // 定期检查舵机状态
    vTaskDelay(pdMS_TO_TICKS(5)); // 减少任务延迟以提高响应速度
  }
}

// 修改初始化I2C函数，使用 GPIO13 (SDA) 和 GPIO14 (SCL)
void initI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // 使用 GPIO13 和 GPIO14 作为 I2C 引脚
  delay(100); // 等待I2C初始化完成
}

//用于显示温度和学生信息以及指令信息的任务函数
void displayAndTemperatureTask(void *parameter) {
  while (1) {
    // Serial.println("[DEBUG] Temperature task running"); // 注释掉任务运行状态调试
    float ambientTemp = mlx.readAmbientTempC();
    displayMessage();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup() {
  // 初始化串口通信
  Serial.begin(115200);  // USB-CDC串口，用于调试
  Serial1.begin(9600, SERIAL_8N1, HANDLE_SERIAL_RX_PIN, HANDLE_SERIAL_TX_PIN);  // 手柄控制
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);    // 舵机控制
  

  // 使用匀加速匀减速方式初始化舵机角度
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], 500, SERVO_MOVE_TIME); 
    delay(100); // 添加短暂延时确保命令发送
  }
  
  servos.moveServo(gripperServo, 500, SERVO_MOVE_TIME); 
  

  // 初始化板载LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 默认关闭LED

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
    2,               // 任务优先级
    NULL,            // 任务句柄
    1                // 运行的核心（Core 1）
  );

  initI2C(); // 初始化I2C接口
  initDisplayAndSensor(); // 初始化显示屏和传感器

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

void loop() {
  // 空循环，所有逻辑已移至任务
}
