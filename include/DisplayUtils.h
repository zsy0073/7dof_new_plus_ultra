#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "Config.h"

extern Adafruit_MLX90614 mlx;
extern Adafruit_SSD1306 display;

void initI2C();
void initDisplayAndSensor();
void displayMessage();

// 添加进度显示相关函数声明
void displayProgressMessage(const String& task, int progress);
// 新增：带有计算点数信息的进度显示函数
void displayProgressWithPoints(const String& task, int progress, int currentPoints, int totalPoints);
void displayErrorMessage(const String& error);

#endif // DISPLAY_UTILS_H
