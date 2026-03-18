#pragma once

// Wi-Fi Configuration (Saved to NVS via Portal or Serial)
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// SlimeVR Server Configuration
#define SLIMEVR_SERVER_IP ""
#define SLIMEVR_SERVER_PORT 6969

// Network Stability
#define HANDSHAKE_COOLDOWN_MS 30000     // Wait 30 seconds before re-handshaking after a timeout
#define HID_WATCHDOG_TIMEOUT_MS 10000   // Reboot if no HID data for 10 seconds

// Movement Thresholds (Optimization)
#define MOVEMENT_THRESHOLD_QUAT 0.0005f   // Threshold for quaternion change (dot product difference) — Slightly tighter for PPS reduction
#define MOVEMENT_THRESHOLD_ACCEL 0.05f    // Threshold for acceleration change (Euclidean distance)

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
