#include "ServoControl.h"

// 定义舵机控制对象（使用Serial2）
LobotServoController servos(Serial2);

// 初始化舵机状态数组
ServoMoveStatus servoStatus[8]; // 7个关节 + 1个夹持器

// 改进运动状态追踪结构，增加历史记录
struct ServoMotionHistory {
  int positions[MOTION_HISTORY_SIZE]; // 位置历史
  unsigned long timestamps[MOTION_HISTORY_SIZE]; // 时间戳历史
  int historyIndex = 0; // 当前历史索引
  int velocity = 0; // 当前速度(单位/秒)
  bool directionPositive = true; // 运动方向(正向/负向)
  bool hasReversed = false; // 是否发生方向反转
};

// 替换原有的简单状态跟踪结构
struct {
  bool isInMotion = false;
  int lastTargetPosition = 500;
  unsigned long lastCommandTime = 0;
  bool isDecelerating = false; // 是否正在减速
  ServoMotionHistory history; // 添加历史记录
  int releaseDetectionCount = 0; // 释放检测计数器
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
    
    // 检查是否有新命令到来，如果有则提前结束延迟
    if (Serial1.available() > 0) {
      break;
    }
  }
}

// 检查舵机是否仍在移动
void checkAndUpdateServoStatus() {
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 8; i++) {
    // 检查舵机移动是否完成
    if (servoStatus[i].isMoving) {
      if (currentTime - servoStatus[i].moveStartTime >= servoStatus[i].moveDuration) {
        // 标记移动完成
        servoStatus[i].isMoving = false;
        
        // 如果是多段移动的一部分
        if (servoMotionState[i].isInMotion && !servoMotionState[i].isDecelerating) {
          // 获取舵机ID和位置信息
          int servoId = (i == 7) ? gripperServo : jointServos[i];
          int currentPos = servoStatus[i].targetPosition; // 当前位置
          int finalPos = servoMotionState[i].lastTargetPosition; // 目标位置
          
          // 如果还未到达最终位置
          if (currentPos != finalPos) {
            int remainingDistance = abs(finalPos - currentPos);
            
            if (remainingDistance > 50) {
              // 继续平滑移动到目标位置
              int midPoint = currentPos + (finalPos - currentPos) / 2;
              int moveTime = calculateMoveTime(currentPos, finalPos);
              startServoMove(servoId, midPoint, moveTime / 2);
            } else {
              // 最后阶段，平稳减速
              int moveTime = max(200, remainingDistance * 2);
              startServoMove(servoId, finalPos, moveTime);
            }
          } else {
            // 已到达目标，结束移动
            servoMotionState[i].isInMotion = false;
          }
        } else if (servoMotionState[i].isDecelerating) {
          // 减速过程中移动完成，标记减速结束
          servoMotionState[i].isDecelerating = false;
        }
      }
    }
    
    // 检查速度是否已经很小，可以认为舵机停止
    if (servoMotionState[i].isInMotion && !servoStatus[i].isMoving) {
      if (abs(servoMotionState[i].history.velocity) < VELOCITY_THRESHOLD) {
        servoMotionState[i].isInMotion = false;
      }
    }
  }
}

// 启动舵机移动，但不等待
void startServoMove(int servoId, int position, int moveTime) {
  // 约束位置在有效范围内
  position = constrain(position, 0, 1000);
  
  // 发送命令到舵机
  servos.moveServo(servoId, position, moveTime);
  
  // 获取舵机索引
  int index;
  if (servoId == gripperServo) {
    index = 7;
  } else {
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        index = i;
        break;
      }
    }
  }
  
  // 更新运动历史
  updateMotionHistory(index, position);
  
  // 更新舵机状态
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
  // 首先约束目标位置
  targetPos = constrain(targetPos, 0, 1000);
  int distance = abs(targetPos - currentPos);
  
  // 获取舵机索引
  int index;
  if (servoId == gripperServo) {
    index = 7;
  } else {
    for (int i = 0; i < 7; i++) {
      if (jointServos[i] == servoId) {
        index = i;
        break;
      }
    }
  }
  
  // 获取当前时间和相关时间间隔
  unsigned long currentTime = millis();
  unsigned long timeSinceLastCommand = currentTime - servoMotionState[index].lastCommandTime;
  
  // 保存上一次目标位置和更新当前目标
  int previousTarget = servoMotionState[index].lastTargetPosition;
  
  // ===== 完全重构的摇杆释放检测算法 =====
  bool isStickRelease = false;
  
  // 检测条件组合:
  // 1. 舵机正在运动
  bool condition1 = servoMotionState[index].isInMotion;
  
  // 2. 位置变化特征:
  //    a) 新目标靠近中心位置(500±100)且之前远离中心
  bool movingToCenter = (abs(targetPos - 500) < STICK_CENTER_RANGE) && 
                        (abs(previousTarget - 500) >= STICK_CENTER_RANGE);
  
  //    b) 目标位置与上一个目标位置相差较大(方向变化)
  bool largePositionChange = false;
  if ((previousTarget > targetPos && servoMotionState[index].history.directionPositive) ||
      (previousTarget < targetPos && !servoMotionState[index].history.directionPositive)) {
    largePositionChange = abs(targetPos - previousTarget) > STICK_DIRECTION_CHANGE_THRESHOLD;
  }
  
  // 3. 时序特征: 命令间隔比平常大(表明摇杆被松开有短暂停顿)
  bool largeTimeGap = timeSinceLastCommand > STICK_RELEASE_TIME_THRESHOLD;
  
  // 组合这些条件判断是否是摇杆释放
  if (condition1 && (movingToCenter || largePositionChange) && largeTimeGap) {
    isStickRelease = true;
    servoMotionState[index].releaseDetectionCount++;
    
    // 防止误判，连续多次检测到才确认是释放
    if (servoMotionState[index].releaseDetectionCount >= 1) {
      servoMotionState[index].isDecelerating = true;
      
      // 调试输出
      Serial.print("检测到摇杆释放: 舵机");
      Serial.print(servoId);
      Serial.print(" 从位置 ");
      Serial.print(currentPos);
      Serial.print(" 到 ");
      Serial.println(targetPos);
    }
  } else {
    servoMotionState[index].releaseDetectionCount = 0;
  }
  
  // 更新运动状态
  servoMotionState[index].isInMotion = true;
  servoMotionState[index].lastTargetPosition = targetPos;
  servoMotionState[index].lastCommandTime = currentTime;
  
  // ===== 针对不同情况的移动策略 =====
  
  // 情况1: 摇杆释放 - 使用超平滑减速曲线
  if (servoMotionState[index].isDecelerating || isStickRelease) {
    // 重置减速状态
    servoMotionState[index].isDecelerating = true;
    
    // 计算当前速度基础上的平稳减速时间
    int velocity = abs(servoMotionState[index].history.velocity);
    int baseStopTime = max(300, distance * 2); // 基础停止时间
    int dynamicStopTime = baseStopTime;
    
    // 根据速度调整停止时间(速度越快停止需要越长时间)
    if (velocity > 0) {
      dynamicStopTime = max(baseStopTime, (velocity * distance * DECELERATION_FACTOR) / 500);
    }
    
    // 限制最大减速时间，防止过慢
    dynamicStopTime = min(dynamicStopTime, 1500);
    
    Serial.print("减速处理: 距离=");
    Serial.print(distance);
    Serial.print(", 速度=");
    Serial.print(velocity);
    Serial.print(", 时间=");
    Serial.println(dynamicStopTime);
    
    // 多步减速过程
    int steps = DECELERATION_STEPS;
    
    // 根据距离调整步数
    if (distance < 50) {
      steps = 3; // 短距离用更少步数
    } else if (distance > 200) {
      steps = 6; // 长距离用更多步数
    }
    
    // 使用贝塞尔缓动曲线创建平滑减速路径
    for (int step = 1; step <= steps; step++) {
      // 计算路径上的位置点(使用三阶贝塞尔曲线)
      float t = (float)step / steps;
      float bezierFactor = t*t*t*(10 + t*(-15 + 6*t)); // 更平滑的贝塞尔缓动函数
      
      // 计算这一步的目标位置
      int stepPos = currentPos + (int)((targetPos - currentPos) * bezierFactor);
      
      // 计算这一步的移动时间(减速过程中时间递增)
      int stepTime = dynamicStopTime / steps + (step * dynamicStopTime) / (steps * 3);
      
      // 发送舵机移动命令
      startServoMove(servoId, stepPos, stepTime);
      
      // 等待一小段时间再发送下一命令(但不阻塞太久)
      if (step < steps) {
        nonBlockingDelay(min(stepTime / 3, 50));
      }
    }
    
    // 最后一步，确保到达最终位置
    startServoMove(servoId, targetPos, dynamicStopTime / 3);
    
    // 最后一步之后，重置减速标志
    servoMotionState[index].isDecelerating = false;
    
    return;
  }
  
  // 情况2: 小范围移动 - 直接使用合适时间
  if (distance < 30) {
    int moveTime = max(150, distance * 5); // 小距离适当增加时间
    startServoMove(servoId, targetPos, moveTime);
    return;
  }
  
  // 情况3: 中等范围移动 - 两段式
  if (distance < 100) {
    int moveTime = calculateMoveTime(currentPos, targetPos);
    int midPoint = currentPos + (targetPos - currentPos) / 2;
    
    // 第一段 - 加速
    startServoMove(servoId, midPoint, moveTime / 2);
    
    // 短暂等待，然后发送第二段命令
    nonBlockingDelay(moveTime / 4);
    
    // 第二段 - 减速到目标
    startServoMove(servoId, targetPos, moveTime / 2 + 50); // 略微延长第二段时间
    return;
  }
  
  // 情况4: 大范围移动 - 三段式(加速-匀速-减速)
  {
    int moveTime = calculateMoveTime(currentPos, targetPos);
    
    // 加速段结束点(大约1/4距离)
    int accelPoint = currentPos + (targetPos - currentPos) / 4;
    // 减速段开始点(大约3/4距离)
    int decelPoint = currentPos + (targetPos - currentPos) * 3 / 4;
    
    // 加速段 - 快速
    int accelTime = moveTime / 4;
    startServoMove(servoId, accelPoint, accelTime);
    nonBlockingDelay(accelTime / 2);
    
    // 匀速段 - 稍慢，给系统时间反应
    int cruiseTime = moveTime / 2;
    startServoMove(servoId, decelPoint, cruiseTime);
    nonBlockingDelay(cruiseTime / 2);
    
    // 减速段 - 最慢，确保平稳到达
    int decelTime = moveTime / 3 + 50; // 略微延长减速时间
    startServoMove(servoId, targetPos, decelTime);
    return;
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

// 添加运动历史记录函数
void updateMotionHistory(int servoIndex, int position) {
  ServoMotionHistory &history = servoMotionState[servoIndex].history;
  
  // 添加新的位置和时间戳
  history.positions[history.historyIndex] = position;
  history.timestamps[history.historyIndex] = millis();
  
  // 计算速度(如果有足够历史)
  int prevIndex = (history.historyIndex + MOTION_HISTORY_SIZE - 1) % MOTION_HISTORY_SIZE;
  unsigned long timeDiff = history.timestamps[history.historyIndex] - history.timestamps[prevIndex];
  
  if (timeDiff > 0) {
    int posDiff = history.positions[history.historyIndex] - history.positions[prevIndex];
    history.velocity = (posDiff * 1000) / timeDiff; // 计算速度(单位/秒)
    
    // 检查方向变化
    bool newDirection = posDiff >= 0;
    if (history.directionPositive != newDirection && abs(posDiff) > STICK_DIRECTION_CHANGE_THRESHOLD) {
      history.hasReversed = true;
    }
    history.directionPositive = newDirection;
  }
  
  // 更新索引
  history.historyIndex = (history.historyIndex + 1) % MOTION_HISTORY_SIZE;
}
