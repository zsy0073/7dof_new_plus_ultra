#include "ServoControl.h"

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 初始化舵机状态数组
ServoMoveStatus servoStatus[8]; // 7个关节 + 1个夹持器

// 增加舵机运动状态追踪
struct {
  bool isInMotion = false;
  int lastTargetPosition = 500;
  unsigned long lastCommandTime = 0;
  float currentSpeed = 0.0;
  bool wasMovingBeforeRelease = false;
} servoMotionState[8];

// 定义舵机ID
const int jointServos[7] = {1, 2, 3, 4, 5, 6, 7};
const int gripperServo = 8;

// 初始化舵机
void initServo() {
  Serial2.begin(9600, SERIAL_8N1, SERVO_SERIAL_RX_PIN, SERVO_SERIAL_TX_PIN);  // 舵机控制
  
  // 使用匀加速匀减速方式初始化舵机角度
  for (int i = 0; i < 7; i++) {
    servos.moveServo(jointServos[i], 500, SERVO_MOVE_TIME); 
    delay(100); // 添加短暂延时确保命令发送
  }
  
  servos.moveServo(gripperServo, 500, SERVO_MOVE_TIME); 
}

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
        
        // 如果是在运动中且不是已经处理过摇杆释放的情况
        if (servoMotionState[i].isInMotion && !servoMotionState[i].wasMovingBeforeRelease) {
          // 检查是否需要继续移动到最终目标位置
          int servoId = (i == 7) ? gripperServo : i + 1;
          int currentPos = servoStatus[i].targetPosition; // 当前已经到达的位置
          int finalPos = servoMotionState[i].lastTargetPosition; // 最终目标位置
          
          // 如果还没到最终位置，继续移动
          if (currentPos != finalPos) {
            // 计算合适的移动时间，确保平滑
            int moveTime = calculateMoveTime(currentPos, finalPos);
            startServoMove(servoId, finalPos, moveTime);
          } else {
            // 如果已到达最终位置，标记运动结束
            servoMotionState[i].isInMotion = false;
          }
        }
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
  // 获取舵机索引
  int index = servoId - 1;
  if (servoId == gripperServo) index = 7;
  
  // 检查是否是摇杆释放后的命令（判断依据：命令间隔较长且位置变化不大）
  unsigned long currentTime = millis();
  bool isProbablyStickRelease = false;
  
  if (servoMotionState[index].isInMotion && 
      (currentTime - servoMotionState[index].lastCommandTime > 200) && 
      abs(targetPos - servoMotionState[index].lastTargetPosition) < 20) {
    // 这可能是摇杆释放后的命令
    isProbablyStickRelease = true;
    servoMotionState[index].wasMovingBeforeRelease = true;
  }
  
  // 更新状态
  servoMotionState[index].lastCommandTime = currentTime;
  servoMotionState[index].lastTargetPosition = targetPos;
  servoMotionState[index].isInMotion = true;
  
  // 对于小范围移动，直接使用计算的移动时间
  if (distance < 30 && !isProbablyStickRelease) { 
    int moveTime = max(100, calculateMoveTime(currentPos, targetPos)); 
    startServoMove(servoId, targetPos, moveTime);
    return;
  }
  
  // 如果检测到可能是摇杆释放，使用更长的时间来平滑停止
  if (isProbablyStickRelease) {
    // 根据距离计算一个较长的停止时间，确保平滑减速
    int moveTime = max(300, calculateMoveTime(currentPos, targetPos) * 2);
    startServoMove(servoId, targetPos, moveTime);
    
    // 重置状态
    servoMotionState[index].wasMovingBeforeRelease = false;
    return;
  }
  
  // 正常的加速过程
  int direction = (targetPos > currentPos) ? 1 : -1;
  int stepSize = max(5, distance / ACCEL_STEPS); 
  int nextPosition = currentPos + direction * stepSize * 2; 
  
  if ((direction > 0 && nextPosition > targetPos) ||
      (direction < 0 && nextPosition < targetPos)) {
    nextPosition = targetPos;
  }
  
  // 使用较短的延迟启动移动
  startServoMove(servoId, nextPosition, ACCEL_DELAY_MIN);
  
  // 计算并更新当前速度
  servoMotionState[index].currentSpeed = direction * stepSize / (float)ACCEL_DELAY_MIN;
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
  // 在开始复位前重置所有舵机的运动状态
  for (int i = 0; i < 8; i++) {
    servoMotionState[i].isInMotion = false;
    servoMotionState[i].currentSpeed = 0.0;
    servoMotionState[i].wasMovingBeforeRelease = false;
  }
  
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
