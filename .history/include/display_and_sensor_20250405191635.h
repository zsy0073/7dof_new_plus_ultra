#ifndef DISPLAY_AND_SENSOR_H
#define DISPLAY_AND_SENSOR_H

#include <Arduino.h>

void initDisplayAndSensor();
void readAndDisplayTemperature();
void displayMessage(const String& line1, const String& line2 = "", const String& line3 = "");

#endif // DISPLAY_AND_SENSOR_H
