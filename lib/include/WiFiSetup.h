//
// Created by Zhenyi Ye on 2019-08-27.
//

#ifndef HEALTHMONITOR_WIFISETUP_H
#define HEALTHMONITOR_WIFISETUP_H

//needed for library
#include <Arduino.h>
#include <SPIFFS.h>
#include <DNSServer.h>
#if defined(ESP8266)
#include <ESP8266WebServer.h>
#else
#include <WebServer.h>
#endif
#include "ESPAsyncWebServer.h"
#include "ESPAsyncWiFiManager.h"
#include "ArduinoJson-v5.h"

void saveConfigCallback();
void readConfiguration();
void configWifi();

#endif //HEALTHMONITOR_WIFISETUP_H
