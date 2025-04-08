#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Config.h"
#include "ServoControl.h"

extern AsyncWebServer server;
extern AsyncWebSocket ws;

void setupWebServer();
void processWebSocketMessage(uint8_t *data, size_t len);
void sendStatusUpdate();

#endif // NETWORK_MANAGER_H
