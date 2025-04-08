#include <Arduino.h>
#include "Config.h"
#include "ServoControl.h"
#include "DisplayUtils.h"
#include "NetworkManager.h"
#include "Tasks.h"

void setup() {
  // 初始化串口通信
  Serial.begin(115200);  // USB-CDC串口，用于调试
  Serial.println("\n\n=========初始化开始=========");
  
  // 初始化舵机
  initServo();
  Serial.println("舵机控制初始化完成");

  // 初始化板载LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 默认关闭LED
  Serial.println("LED初始化完成");

  // 初始化I2C和显示设备
  initI2C();
  initDisplayAndSensor();
  Serial.println("显示设备初始化完成");
  
  // 手动初始化PS2手柄，确保其正常工作
  Serial.println("准备初始化PS2控制器...");
  ps2Controller.init();
  delay(100); // 短暂延时确保手柄初始化完成
  
  // 设置并启动所有任务
  Serial.println("准备启动任务调度...");
  setupTasks();
  Serial.println("=========初始化完成=========");
}

void loop() {
  // 空循环，所有逻辑已移至任务
}
