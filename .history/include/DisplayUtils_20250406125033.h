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

#endif // DISPLAY_UTILS_H
