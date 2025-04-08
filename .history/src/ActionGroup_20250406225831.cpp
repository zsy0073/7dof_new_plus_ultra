#include "ActionGroup.h"
#include "ServoControl.h"

// 动作组存储
ServoAction actionGroup[MAX_ACTIONS];
int actionCount = 0;

// 动作组状态变量 - 修改为volatile
volatile bool isRecording = false;
volatile bool isPlaying = false;
unsigned long lastActionTime = 0;
unsigned long recordStartTime = 0; // 记录开始的时间
const int ACTION_INTERVAL = 50;    // 减少记录间隔，提高精度

// 播放相关变量
int currentActionIndex = 0;
unsigned long nextActionTime = 0;
unsigned long playStartTime = 0;   // 播放开始的时间

// 记录一个动作
void recordAction(int servoId, int position, int moveTime) {
  if (actionCount >= MAX_ACTIONS) {
    Serial.println("动作组已满，无法继续记录");
    return;
  }
  
  // 计算相对时间戳
  unsigned long currentTime = millis();
  unsigned long relativeTimestamp = currentTime - recordStartTime;
  
  // 存储动作
  actionGroup[actionCount].servoId = servoId;
  actionGroup[actionCount].position = position;
  actionGroup[actionCount].moveTime = moveTime;
  actionGroup[actionCount].timestamp = relativeTimestamp; // 添加时间戳
  
  Serial.print("记录动作: ID=");
  Serial.print(servoId);
  Serial.print(", 位置=");
  Serial.print(position);
  Serial.print(", 时间=");
  Serial.print(moveTime);
  Serial.print(", 时间戳=");
  Serial.print(relativeTimestamp);
  Serial.print(", 编号=");
  Serial.println(actionCount + 1);
  
  actionCount++;
  lastActionTime = currentTime;
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
  recordStartTime = millis(); // 记录开始时间
  lastActionTime = recordStartTime;
  
  Serial.print("开始记录动作组，时间戳: ");
  Serial.println(recordStartTime);
  
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
  playStartTime = millis();  // 记录播放开始时间
  
  // 设置下一个动作的时间为立即执行第一个动作
  nextActionTime = playStartTime;
  
  Serial.print("开始播放动作组，共 ");
  Serial.print(actionCount);
  Serial.print(" 个动作，时间戳: ");
  Serial.println(playStartTime);
}

// 更新动作组播放
// 返回值: true-还在播放, false-播放完成
bool updatePlayActions() {
  // 检查是否有动作要播放
  if (currentActionIndex >= actionCount) {
    return false; // 播放完成
  }
  
  // 当前时间
  unsigned long currentTime = millis();
  
  // 获取当前动作
  ServoAction& action = actionGroup[currentActionIndex];
  
  // 检查是否到达当前动作的播放时间
  if (currentTime >= nextActionTime) {
    // 检查是否是复位命令
    if (action.servoId == RESET_ACTION_ID) {
      Serial.print("播放动作 ");
      Serial.print(currentActionIndex + 1);
      Serial.print("/");
      Serial.print(actionCount);
      Serial.print(" 时间戳: ");
      Serial.print(action.timestamp);
      Serial.println(" [复位所有舵机]");
      
      // 执行复位动作
      resetServos();
    } else {
      // 执行常规动作
      servos.moveServo(action.servoId, action.position, action.moveTime);
      
      // 更新舵机状态
      int index = getServoIndex(action.servoId);
      if (index >= 0) {
        servoStates[index].currentPos = action.position;
        
        // 更新全局状态
        if (action.servoId == gripperServo) {
          armStatus.gripper = action.position;
        } else if (index >= 0 && index < 7) {
          armStatus.joints[index] = action.position;
        }
      }
      
      Serial.print("播放动作 ");
      Serial.print(currentActionIndex + 1);
      Serial.print("/");
      Serial.print(actionCount);
      Serial.print(" 时间戳: ");
      Serial.print(action.timestamp);
      Serial.print(" ID=");
      Serial.print(action.servoId);
      Serial.print(", 位置=");
      Serial.print(action.position);
      Serial.print(", 时间=");
      Serial.println(action.moveTime);
    }
    
    // 更新最后一次舵机移动预计完成的时间
    servoLastMoveEndTime = currentTime + action.moveTime + 50;
    
    // 移动到下一个动作
    currentActionIndex++;
    
    // 如果还有下一个动作，设置下一个动作的时间
    if (currentActionIndex < actionCount) {
      // 获取下一个动作的时间戳，根据记录的相对时间计算
      unsigned long nextActionTimestamp = actionGroup[currentActionIndex].timestamp;
      // 计算下一个动作的播放时间 = 播放开始时间 + 下一个动作的相对时间戳
      nextActionTime = playStartTime + nextActionTimestamp;
    }
    
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
