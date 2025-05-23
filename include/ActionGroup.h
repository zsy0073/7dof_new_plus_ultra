#ifndef ACTION_GROUP_H
#define ACTION_GROUP_H

#include <Arduino.h>
#include "Config.h"

// 最大动作数
#define MAX_ACTIONS 5000

// 舵机ID的特殊值，表示这是一个复位命令
#define RESET_ACTION_ID 0xFF

// 单个动作的数据结构
typedef struct {
  int servoId;       // 舵机ID (特殊值 RESET_ACTION_ID 表示复位)
  int position;      // 目标位置
  int moveTime;      // 移动时间
  unsigned long timestamp; // 记录命令的时间戳，相对于开始记录的时间
} ServoAction;

// 动作组状态 - 修改为volatile以保持一致性
extern volatile bool isRecording;
extern volatile bool isPlaying;
extern unsigned long lastActionTime;
extern unsigned long recordStartTime; // 记录开始时间
extern const int ACTION_INTERVAL;

// 函数声明
void recordAction(int servoId, int position, int moveTime);
void clearActionGroup();
int getActionCount();
void startRecordActionGroup();
void stopRecordActionGroup();
void playActionGroup();
void startPlayActions();
bool updatePlayActions();
void checkActionGroupStatus();

#endif // ACTION_GROUP_H
