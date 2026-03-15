#pragma once

// Wi-Fi Configuration
#define WIFI_SSID "Sebbos Wifi"
#define WIFI_PASSWORD "1t2gmwn0"

// SlimeVR Server Configuration
#define SLIMEVR_SERVER_IP "10.0.0.21" // Replace with your PC's IP address
#define SLIMEVR_SERVER_PORT 6969

// Debug Output
#define ENABLE_DEBUG_PRINT 1

#include "UdpLogger.h"

#if ENABLE_DEBUG_PRINT
#define DEBUG_PRINT(x) UdpLogger.print(x)
#define DEBUG_PRINTLN(x) UdpLogger.println(x)
#define DEBUG_PRINTF(...) UdpLogger.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif
