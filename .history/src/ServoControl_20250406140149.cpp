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

// 改进非阻塞延迟函数，添加任务让步
void nonBlockingDelay(unsigned long delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
    // 允许其他任务执行，增加让步次数
    vTaskDelay(pdMS_TO_TICKS(1));
    // 检查是否有新命令到来，如果有则提前结束延迟
    if (Serial1.available() > 0 || Serial.available() > 0) {
      break;
    }
  }
}

// 检查舵机是否仍在移动 - 改进版本
void checkAndUpdateServoStatus() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 8; i++) {
    if (servoStatus[i].isMoving) {
      if (currentTime - servoStatus[i].moveStartTime >= servoStatus[i].moveDuration) {
        // 移动完成
        servoStatus[i].isMoving = false;
        
        // 如果这是多段移动的一部分
        if (servoMotionState[i].isInMotion) {
          // 获取舵机ID
          int servoId = (i == 7) ? gripperServo : jointServos[i];
          int currentPos = servoStatus[i].targetPosition; // 当前已经到达的位置
          int finalPos = servoMotionState[i].lastTargetPosition; // 最终目标位置
          
          // 如果还没有到达最终位置
          if (currentPos != finalPos) {
            // 计算剩余距离
            int remainingDistance = abs(finalPos - currentPos);
            
            if (remainingDistance > 50) {
              // 对于较长距离，继续使用加减速
              int midPoint = currentPos + (finalPos - currentPos) / 2;
              int moveTime = calculateMoveTime(currentPos, finalPos);
              
              // 发送到中间点的命令
              startServoMove(servoId, midPoint, moveTime / 2);
            } else {
              // 对于较短距离，平稳减速到目标位置
              int moveTime = max(200, remainingDistance * 2);
              startServoMove(servoId, finalPos, moveTime);
              // 标记最后阶段
              servoMotionState[i].wasMovingBeforeRelease = false;
            }
          } else {
            // 到达最终位置，标记移动完成
            servoMotionState[i].isInMotion = false;
          }
        }
      }
    }
  }
}

// 启动舵机移动，但不等待 - 改进版本
void startServoMove(int servoId, int position, int moveTime) {
  // 约束位置在有效范围内
  position = constrain(position, 0, 1000);
  
  servos.moveServo(servoId, position, moveTime);
  
  // 更新状态
  int index;
  if (servoId == gripperServo) {
    index = 7;
  } else {
    // 查找舵机ID对应的索引
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        index = i;
        break;
      }
    }
  }
  
  servoStatus[index].isMoving = true;
  servoStatus[index].moveStartTime = millis();
  servoStatus[index].moveDuration = moveTime;
  servoStatus[index].targetPosition = position;
  servoStatus[index].servoId = servoId;
}

// 完全重写的平滑加速移动函数 - 非阻塞版本
void smoothAccelMoveServoNonBlocking(int servoId, int currentPos, int targetPos) {
  // 定义舵机索引
  int index;
  
  if (servoId == gripperServo) {
    index = 7;
  } else {
    // 查找关节ID对应的索引
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        index = i;
        break;
      }
    }
  }
  
  // 确保位置在有效范围内
  targetPos = constrain(targetPos, 0, 1000);
  int distance = abs(targetPos - currentPos);
  
  // 获取当前时间和时间间隔
  unsigned long currentTime = millis();
  unsigned long timeSinceLastCommand = currentTime - servoMotionState[index].lastCommandTime;
  
  // 存储上一次的目标位置，用于后续比较
  int previousTarget = servoMotionState[index].lastTargetPosition;
  
  // 更新运动状态
  servoMotionState[index].lastCommandTime = currentTime;
  servoMotionState[index].lastTargetPosition = targetPos;
  servoMotionState[index].isInMotion = true;
  
  // 改进的摇杆释放检测算法
  bool isStickRelease = false;
  
  // 满足以下条件时认为摇杆已释放:
  // 1. 舵机当前处于运动状态
  // 2. 与上次命令间隔超过阈值(通常摇杆释放会有一段间隔)
  // 3. 目标位置突然向中间位置(500)大幅度偏移
  // 4. 上一次目标位置偏离中间位置较远
  if (servoMotionState[index].wasMovingBeforeRelease && 
      timeSinceLastCommand > 100 &&
      ((targetPos > 450 && targetPos < 550 && abs(previousTarget - 500) > 100) || 
       (abs(targetPos - previousTarget) > 150 && 
        ((previousTarget > 500 && targetPos < previousTarget) || 
         (previousTarget < 500 && targetPos > previousTarget))))) {
    isStickRelease = true;
  }
  
  // 记录当前运动状态用于下次判断
  servoMotionState[index].wasMovingBeforeRelease = (abs(targetPos - 500) > 100);
  
  // 针对摇杆释放的特殊处理 - 平滑减速
  if (isStickRelease) {
    // 计算一个更长的减速时间，防止突然停止
    int stopTime = max(800, distance * 5); // 显著增加停止时间
    
    // 使用更多步骤实现超平滑停止
    int numSteps = 5; // 增加步数
    
    // 创建平滑的减速路径
    for (int step = 1; step <= numSteps; step++) {
      // 使用更平滑的缓动曲线(easing curve)
      float t = (float)step / numSteps;
      // 三阶贝塞尔曲线: 更加平滑的减速
      float smoothRatio = t * t * (3.0f - 2.0f * t);
      
      // 计算这一步的目标位置
      int stepTargetPos = currentPos + (int)((targetPos - currentPos) * smoothRatio);
      // 计算这一步的移动时间，越接近目标越慢
      int stepTime = (int)(stopTime * (0.5f + 0.5f * smoothRatio));
      
      // 发送舵机移动命令
      startServoMove(servoId, stepTargetPos, stepTime);
      
      // 等待足够时间再发送下一个命令，但不阻塞整个系统
      nonBlockingDelay(stepTime / 2);
    }
    
    // 最终精确到达目标位置，使用较长时间确保平滑
    startServoMove(servoId, targetPos, stopTime / 2);
    return;
  }
  
  // 正常的移动处理(非摇杆释放情况)
  if (distance < 30) { 
    // 小范围移动，使用适当的时间
    int moveTime = max(150, distance * 5); // 增加小距离移动的时间
    startServoMove(servoId, targetPos, moveTime);
  } else if (distance < 100) {
    // 中等范围移动，两段式
    int midPoint = currentPos + (targetPos - currentPos) / 2;
    int moveTime = calculateMoveTime(currentPos, targetPos);
    startServoMove(servoId, midPoint, moveTime / 2);
    
    // 延迟部分时间再发送第二段命令
    nonBlockingDelay(moveTime / 4);
    startServoMove(servoId, targetPos, moveTime / 2);
  } else {
    // 大范围移动，使用三段式加减速
    int moveTime = calculateMoveTime(currentPos, targetPos);
    int firstPoint = currentPos + (targetPos - currentPos) / 4;
    int secondPoint = currentPos + (targetPos - currentPos) * 3 / 4;
    
    // 加速段 - 较快到达第一个点
    startServoMove(servoId, firstPoint, moveTime / 4);
    nonBlockingDelay(moveTime / 6);
    
    // 匀速段 - 较慢到达第二个点
    startServoMove(servoId, secondPoint, moveTime / 2);
    nonBlockingDelay(moveTime / 4);
    
    // 减速段 - 平滑到达最终位置
    startServoMove(servoId, targetPos, moveTime / 3);
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
        
        // 如果这是多段移动的一部分
        if (servoMotionState[i].isInMotion) {
          // 获取舵机ID
          int servoId = (i == 7) ? gripperServo : i + 1;
          int currentPos = servoStatus[i].targetPosition; // 当前已经到达的位置
          int finalPos = servoMotionState[i].lastTargetPosition; // 最终目标位置
          
          // 如果还没有到达最终位置
          if (currentPos != finalPos) {
            // 计算剩余距离
            int remainingDistance = abs(finalPos - currentPos);
            
            if (remainingDistance > 50) {
              // 对于较长距离，继续使用加减速
              int midPoint = currentPos + (finalPos - currentPos) / 2;
              int moveTime = calculateMoveTime(currentPos, finalPos);
              
              // 发送到中间点的命令
              startServoMove(servoId, midPoint, moveTime / 2);
            } else {
              // 对于较短距离，平稳减速到目标位置
              int moveTime = max(200, remainingDistance * 2);
              startServoMove(servoId, finalPos, moveTime);
              // 标记最后阶段
              servoMotionState[i].wasMovingBeforeRelease = false;
            }
          } else {
            // 到达最终位置，标记移动完成
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
  // 获取舵机索引 - 修复索引计算
  int index;
  
  if (servoId == gripperServo) {
    index = 7;
  } else {
    // 因为jointServos数组存储的是真实舵机ID，需要反向查找索引
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        index = i;
        break;
      }
    }
  }
  
  unsigned long currentTime = millis();
  unsigned long timeSinceLastCommand = currentTime - servoMotionState[index].lastCommandTime;
  
  // 检测摇杆释放的新算法
  bool isStickRelease = false;
  
  // 条件1: 舵机当前正在运动
  // 条件2: 与上次命令间隔较长(手柄释放通常会有较长停顿)
  // 条件3: 目标位置与之前的命令有明显差异(手柄释放通常导致回到中心位置)
  if (servoMotionState[index].isInMotion && 
      timeSinceLastCommand > 150 &&
      abs(targetPos - servoMotionState[index].lastTargetPosition) > 20) {
    isStickRelease = true;
  }
  
  // 更新舵机运动状态
  servoMotionState[index].lastCommandTime = currentTime;
  int previousTarget = servoMotionState[index].lastTargetPosition;
  servoMotionState[index].lastTargetPosition = targetPos;
  servoMotionState[index].isInMotion = true;
  
  // 针对摇杆释放的特殊处理
  if (isStickRelease) {
    // 计算一个基于当前速度和距离的缓慢停止时间
    int stopTime = max(500, distance * 3); // 至少500ms的停止时间
    
    // 分步执行减速移动来模拟平滑停止
    int numSteps = 3;
    for (int step = 1; step <= numSteps; step++) {
      float ratio = (float)step / numSteps;
      // 使用二次曲线实现更平滑的减速
      float smoothRatio = ratio * (2.0f - ratio);
      
      int stepTargetPos = currentPos + (int)((targetPos - currentPos) * smoothRatio);
      int stepTime = (int)(stopTime * (1.0f - (numSteps - step) / (float)numSteps));
      
      startServoMove(servoId, stepTargetPos, stepTime);
      
      // 等待部分时间再发送下一个命令
      nonBlockingDelay(stepTime / 2);
    }
    
    // 最终到达目标位置
    startServoMove(servoId, targetPos, stopTime / 3);
    return;
  }
  
  // 正常的移动处理(非摇杆释放情况)
  if (distance < 30) { 
    // 小范围移动
    int moveTime = max(100, calculateMoveTime(currentPos, targetPos)); 
    startServoMove(servoId, targetPos, moveTime);
  } else {
    // 大范围移动，使用匀加速处理
    // 首先确定总移动时间和中间点
    int moveTime = calculateMoveTime(currentPos, targetPos);
    int midPoint = currentPos + (targetPos - currentPos) / 2;
    
    // 先快速到达中间点（加速阶段）
    startServoMove(servoId, midPoint, moveTime / 3);
    
    // 指令已发送，但不等待其完成，让控制系统继续处理其他事务
    // 后续的移动将通过checkAndUpdateServoStatus函数处理
    
    // 更新中间状态，以便后续状态检查时继续移动
    servoMotionState[index].currentSpeed = (targetPos - currentPos) / (float)moveTime;
  }
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
  
  if (maxDistance < 30) {
    // 对于小范围移动，使用较长时间确保平滑
    int moveTime = 400; // 增大小范围移动的时间
    for (int i = 0; i < 7; i++) {
      startServoMove(jointServos[i], 500, moveTime);
      armStatus.joints[i] = 500;
    }
    startServoMove(gripperServo, 500, moveTime);
    armStatus.gripper = 500;
    nonBlockingDelay(moveTime * 0.8); // 等待大部分移动完成但不阻塞全程
    return;
  }
  
  // 增加步骤数量，使移动更加平滑
  int numSteps = 5; // 增加步骤数
  
  // 生成更平滑的S曲线步骤分布
  for (int stepIdx = 0; stepIdx < numSteps; stepIdx++) {
    // 使用S曲线计算当前进度比例 (0.0 到 1.0)
    float progress = (float)(stepIdx + 1) / numSteps;
    // 将线性进度转换为S曲线进度
    float sCurveProgress = -2.0f * progress * progress * progress + 3.0f * progress * progress;
    
    // 计算这一步的延迟时间 - 中间阶段速度最快
    int stepDelay;
    if (stepIdx < numSteps / 2) {
      // 加速阶段 - 延迟逐渐减小
      stepDelay = ACCEL_DELAY_MAX - (ACCEL_DELAY_MAX - ACCEL_DELAY_MIN) * (2.0f * stepIdx / numSteps);
    } else {
      // 减速阶段 - 延迟逐渐增加
      stepDelay = ACCEL_DELAY_MIN + (ACCEL_DELAY_MAX - ACCEL_DELAY_MIN) * (2.0f * (stepIdx - numSteps/2) / numSteps);
    }
    
    // 同时移动所有关节舵机
    for (int i = 0; i < 7; i++) {
      int distance = 500 - currentPositions[i];
      int targetPos = currentPositions[i] + (int)(distance * sCurveProgress);
      startServoMove(jointServos[i], targetPos, stepDelay);
    }
    
    // 移动夹持器舵机
    int gripperDistance = 500 - currentPositions[7];
    int gripperTargetPos = currentPositions[7] + (int)(gripperDistance * sCurveProgress);
    startServoMove(gripperServo, gripperTargetPos, stepDelay);
    
    // 使用非阻塞延迟，但确保有足够时间让舵机响应
    nonBlockingDelay(stepDelay + 30);
  }
  
  // 最终确保精确到达中心位置
  int finalDelay = 300; // 较长的最终定位时间
  for (int i = 0; i < 7; i++) {
    startServoMove(jointServos[i], 500, finalDelay);
    armStatus.joints[i] = 500;
  }
  startServoMove(gripperServo, 500, finalDelay);
  armStatus.gripper = 500;
  
  // 等待最终移动完成一部分
  nonBlockingDelay(finalDelay * 0.6);
}
