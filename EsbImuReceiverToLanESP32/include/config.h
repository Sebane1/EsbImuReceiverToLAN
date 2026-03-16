#pragma once

// Wi-Fi Configuration (Saved to NVS via Portal or Serial)
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// SlimeVR Server Configuration
#define SLIMEVR_SERVER_IP ""
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
