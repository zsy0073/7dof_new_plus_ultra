#include "display_and_sensor.h"
#include <Adafruit_MLX90614.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

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

void initDisplayAndSensor() {
  if (!mlx.begin()) { // 使用默认I2C地址初始化 MLX90614
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 使用默认I2C地址初始化 OLED
    while (1);
  }

  // 显示初始化动画
  for (int size = 0; size <= 64; size += 4) {
    display.clearDisplay();
    display.drawRect(0, 0, size * 2, size, WHITE);
    display.display();
    delay(100);
  }

  displayMessage("System Ready!");
  delay(2000);

  displayMessage("ID: " + String(studentID), "Name: " + String(studentName));
  delay(3000);
}

void readAndDisplayTemperature() {
  float ambientTemp = mlx.readAmbientTempC();
  displayMessage("Ambient Temp: " + String(ambientTemp) + " C", 
                 "ID: " + String(studentID), 
                 "Name: " + String(studentName));
}
