#pragma once

// Wi-Fi Configuration
#define WIFI_SSID "Sebbos Wifi"
#define WIFI_PASSWORD "1t2gmwn0"

// SlimeVR Server Configuration
#define SLIMEVR_SERVER_IP "10.0.0.21" // Replace PC's IP address
#define SLIMEVR_SERVER_PORT 6969

// Debug Output
#define ENABLE_DEBUG_PRINT 1

#include <Arduino.h>

#if ENABLE_DEBUG_PRINT
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif
