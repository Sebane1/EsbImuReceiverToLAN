#pragma once

#include <Arduino.h>
#include <Preferences.h>

class WiFiConfig {
public:
    static void load(char* ssid, char* password);
    static void save(const char* ssid, const char* password);
    static bool hasCredentials();
    static void clear();

    // Helper to get a unique device name for AP Mode
    static String getDeviceName();
};
