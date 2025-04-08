/******************************************************
* FileName:      LobotServoController.h
* Company:       乐幻索尔
* Date:          2016/07/02  16:53
* Description:   Lobot舵机控制板二次开发库的宏定义、类等
*****************************************************/

#ifndef LOBOT_SERVO_CONTROLLER_H
#define LOBOT_SERVO_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>

//舵机写入方式
#define WRITE_FROM_STRUCT        0   //通过结构体

//舵机运行状态
#define ACTION_NO_ERROR          0   //无误差
#define ACTION_HEADER_ERROR      1   //帧头错误
#define ACTION_RANGE_ERROR       2   //范围错误

//指令定义
#define FRAME_HEADER            0x55 //帧头
#define CMD_SERVO_MOVE          1    //舵机移动指令
#define CMD_ACTION_GROUP_RUN    2    //运行动作组指令
#define CMD_ACTION_GROUP_STOP   3    //停止动作组运行
#define CMD_ACTION_GROUP_SPEED  4    //设置动作组速度
#define CMD_GET_BATTERY_VOLTAGE 5    //获取电池电压
#define CMD_MULT_SERVO_POS_READ 7    //获取多个舵机的位置
#define CMD_MULT_SERVO_UNLOAD   0x14 //卸载多个舵机指令

//接收指令定义
#define BATTERY_VOLTAGE         1    //电池电压
#define ACTION_GROUP_RUNNING    2    //有动作组在运行
#define ACTION_GROUP_STOPPED    3    //所有动作组已停止
#define ACTION_GROUP_COMPLETE   4    //动作组运行完成

struct LobotServo {
  uint8_t  ID;           //舵机ID
  uint16_t Position;     //目标位置
};

class LobotServoController {
  public:
    LobotServoController();
    LobotServoController(HardwareSerial &A);
    ~LobotServoController();
    void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
    void moveServos(LobotServo servos[], uint8_t Num, uint16_t Time);
    void moveServos(uint8_t Num, uint16_t Time, ...);
    void runActionGroup(uint8_t numOfAction, uint16_t Times);
    void stopActionGroup(void);
    void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
    void setAllActionGroupSpeed(uint16_t Speed);
    void getBatteryVoltage();
    void setServoUnload(uint8_t numOfServos, ...);
    uint8_t getServosPos(uint8_t numOfServos, ...);
    void receiveHandle();
    uint16_t batteryVoltage;
    uint8_t numOfActinGroupRunning;
    uint16_t actionGroupRunTimes;
    bool isRunning;
    uint16_t servosPos[32];
  private:
    HardwareSerial *SerialX;
};

#endif
