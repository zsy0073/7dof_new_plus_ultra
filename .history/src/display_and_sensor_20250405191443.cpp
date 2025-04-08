#include "display_and_sensor.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "log.h" // 引入日志宏定义

// OLED 显示屏和温度传感器对象
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// 学号和姓名
const char* studentID = "20250001";
const char* studentName = "ZSY";

void displayMessage(const String& line1, const String& line2, const String& line3) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(line1);
  if (!line2.isEmpty()) {
    display.setCursor(0, 10);
    display.print(line2);
  }
  if (!line3.isEmpty()) {
    display.setCursor(0, 20);
    display.print(line3);
  }
  display.display();
}

void readAndDisplayTemperature() {
  float ambientTemp = mlx.readAmbientTempC();
  displayMessage("Ambient Temp: " + String(ambientTemp) + " C", 
                 "ID: " + String(studentID), 
                 "Name: " + String(studentName));
}
