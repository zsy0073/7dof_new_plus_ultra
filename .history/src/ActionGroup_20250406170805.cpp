#include "ActionGroup.h"
#include "ServoControl.h"
#include "ServoTasks.h" // 添加异步任务支持

// 动作组存储
ServoAction actionGroup[MAX_ACTIONS];
int actionCount = 0;

// 动作组状态变量
bool isRecording = false;
bool isPlaying = false;
unsigned long lastActionTime = 0;
const int ACTION_INTERVAL = 200; // 记录动作的最小间隔时间(毫秒)

// 播放相关变量
int currentActionIndex = 0;
unsigned long nextActionTime = 0;

// 记录一个动作
void recordAction(int servoId, int position, int moveTime) {
  if (actionCount >= MAX_ACTIONS) {
    Serial.println("动作组已满，无法继续记录");
    return;
  }
  
  // 存储动作
  actionGroup[actionCount].servoId = servoId;
  actionGroup[actionCount].position = position;
  actionGroup[actionCount].moveTime = moveTime;
  actionCount++;
  
  Serial.print("记录动作: ID=");
  Serial.print(servoId);
  Serial.print(", 位置=");
  Serial.print(position);
  Serial.print(", 时间=");
  Serial.print(moveTime);
  Serial.print(", 编号=");
  Serial.println(actionCount);
}

// 清除动作组
void clearActionGroup() {
  actionCount = 0;
  Serial.println("清除动作组");
}

// 获取动作组中的动作数量
int getActionCount() {
  return actionCount;
}

// 开始记录动作组
void startRecordActionGroup() {
  if (isPlaying) {
    Serial.println("正在播放动作组，无法开始记录");
    return;
  }
  
  if (isRecording) {
    Serial.println("已经在记录动作组");
    return;
  }
  
  clearActionGroup();  // 清除之前记录的动作组
  isRecording = true;
  lastActionTime = millis();
  Serial.println("开始记录动作组");
  
  // 通知所有WebSocket客户端
  armStatus.updated = true;
}

// 停止记录动作组
void stopRecordActionGroup() {
  if (!isRecording) {
    Serial.println("未在记录动作组");
    return;
  }
  
  isRecording = false;
  Serial.print("停止记录动作组，共记录 ");
  Serial.print(actionCount);
  Serial.println(" 个动作");
  
  // 通知所有WebSocket客户端
  armStatus.updated = true;
}

// 播放动作组
void playActionGroup() {
  if (isRecording) {
    Serial.println("正在记录动作组，无法播放");
    return;
  }
  
  if (isPlaying) {
    Serial.println("已经在播放动作组");
    return;
  }
  
  if (actionCount == 0) {
    Serial.println("动作组为空，无法播放");
    return;
  }
  
  isPlaying = true;
  Serial.println("开始播放动作组");
  
  // 启动播放过程
  startPlayActions();
  
  // 通知所有WebSocket客户端
  armStatus.updated = true;
}

// 开始播放动作组
void startPlayActions() {
  if (actionCount == 0) return;
  
  currentActionIndex = 0;
  nextActionTime = millis();
  
  Serial.print("开始播放动作组，共 ");
  Serial.print(actionCount);
  Serial.println(" 个动作");
}

// 更新动作组播放
// 返回值: true-还在播放, false-播放完成
bool updatePlayActions() {
  // 检查是否有动作要播放
  if (currentActionIndex >= actionCount) {
    return false; // 播放完成
  }
  
  // 检查是否到达下一个动作的时间
  if (millis() >= nextActionTime) {
    // 检查所有舵机是否已经完成之前的动作
    if (areAnyServosMoving()) {
      // 如果有舵机仍在移动，则稍后再检查
      return true;
    }
    
    // 获取当前动作
    ServoAction& action = actionGroup[currentActionIndex];
    
    // 检查是否是复位命令
    if (action.servoId == RESET_ACTION_ID) {
      Serial.print("播放动作 ");
      Serial.print(currentActionIndex + 1);
      Serial.print("/");
      Serial.print(actionCount);
      Serial.println(": [复位所有舵机]");
      
      // 异步执行复位动作
      resetAllServosAsync();
      
      // 更新下一个动作的时间 (使用记录的时间)
      nextActionTime = millis() + action.moveTime + 50; // 添加50ms的缓冲时间
    } else {
      // 执行常规动作
      int servoId = action.servoId;
      int position = action.position;
      int moveTime = action.moveTime;
      
      Serial.print("播放动作 ");
      Serial.print(currentActionIndex + 1);
      Serial.print("/");
      Serial.print(actionCount);
      Serial.print(": ID=");
      Serial.print(servoId);
      Serial.print(", 位置=");
      Serial.print(position);
      Serial.print(", 时间=");
      Serial.println(moveTime);
      
      // 使用异步带参数的控制方式移动舵机
      moveServoWithParamsAsync(servoId, position, moveTime, false);
      
      // 更新下一个动作的时间
      nextActionTime = millis() + moveTime;
    }
    
    // 移动到下一个动作
    currentActionIndex++;
    
    // 更新状态标记
    armStatus.updated = true;
  }
  
  return true; // 还在播放
}

// 检查动作组播放状态
void checkActionGroupStatus() {
  if (isPlaying) {
    // 检查动作组播放是否完成
    if (!updatePlayActions()) {
      isPlaying = false;
      Serial.println("动作组播放完成");
      armStatus.updated = true; // 更新状态通知WebSocket客户端
    }
  }
}
