#include <Arduino.h>
#include "Config.h"
#include "ServoControl.h"
#include "DisplayUtils.h"
#include "NetworkManager.h"
#include "Tasks.h"
#include "ActionGroup.h" 
#include "ServoTasks.h" // 添加舵机任务头文件

void setup() {
  // 初始化串口通信
  Serial.begin(115200);  // USB-CDC串口，用于调试
  Serial1.begin(9600, SERIAL_8N1, HANDLE_SERIAL_RX_PIN, HANDLE_SERIAL_TX_PIN);  // 手柄控制
  
  // 初始化舵机
  initServo();

  // 初始化板载LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 默认关闭LED

  // 初始化I2C和显示设备
  initI2C();
  initDisplayAndSensor();
  
  // 设置并启动所有任务
  setupTasks();
}

void loop() {
  // 空循环，所有逻辑已移至任务
}
