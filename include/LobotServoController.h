/******************************************************
* FileName:      LobotServoController.h
* Company:       乐幻索尔
* Date:          2016/07/02  16:53
* Description:   Lobot舵机控制板二次开发库的宏定义、类等
*****************************************************/

#ifndef LOBOTSERVOCONTROLLER_H
#define LOBOTSERVOCONTROLLER_H

#include <Arduino.h>

//发送部分的指令
#define FRAME_HEADER            0x55   //帧头
#define CMD_SERVO_MOVE          0x03   //舵机移动指令
// 移除未使用的命令定义
#define CMD_ACTION_GROUP_RUN    0x06   //运行动作组指令
#define CMD_ACTION_GROUP_STOP   0x07   //停止动作组运行指令
#define CMD_ACTION_GROUP_SPEED  0x0B   //设置动作组运行速度指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F   //获得电池电压指令
#define CMD_MULT_SERVO_UNLOAD   0x14   //多个舵机掉电指令
#define CMD_MULT_SERVO_POS_READ 0x15   //读取多个舵机的角度位置指令

//接收部分的指令
#define BATTERY_VOLTAGE       0x0F  //电池电压
#define ACTION_GROUP_RUNNING  0x06  //动作组被运行
#define ACTION_GROUP_STOPPED  0x07  //动作组被停止
#define ACTION_GROUP_COMPLETE 0x08  //动作组完成

struct LobotServo {  //舵机ID和位置结构体
  uint8_t  ID;       //舵机ID
  uint16_t Position; //舵机数据
};

class LobotServoController {
  public:
    LobotServoController();
    LobotServoController(HardwareSerial &A);
    ~LobotServoController();

    void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
    // 其他方法保留，因为它们可能被第三方库内部调用
    void moveServos(LobotServo servos[], uint8_t Num, uint16_t Time);
    void moveServos(uint8_t Num,uint16_t Time, ...);
    void runActionGroup(uint8_t NumOfAction, uint16_t Times);
    void stopActionGroup(void);
    void setActionGroupSpeed(uint8_t NumOfAction, uint16_t Speed);
    void setAllActionGroupSpeed(uint16_t Speed);
    void setServoUnload(uint8_t numOfServos, ...);
    byte getServosPos(uint8_t numOfServos, ...);

    int LobotSerialServoReceiveHandle(byte *ret);
    
    void getBatteryVoltage(void);
    void receiveHandle(void);

  public:
    // 保留所有公共成员，它们可能被第三方库内部使用
    uint8_t  numOfActinGroupRunning; 
    uint16_t actionGroupRunTimes; 
    bool isRunning; 
    uint16_t batteryVoltage; 
    int servosPos[128]; 

    HardwareSerial *SerialX;
};
#endif
