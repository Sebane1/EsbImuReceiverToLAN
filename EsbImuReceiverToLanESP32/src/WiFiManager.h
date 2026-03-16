#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

class WiFiManager {
public:
    static void init();
    static void loop();
    static bool isConnected();
    static void reconnect();

private:
    static void startCaptivePortal();
    static void handleRoot();
    static void handleSave();
    static void handleNotFound();

    static WebServer server;
    static DNSServer dnsServer;
    static bool portalActive;
    static uint32_t portalStartTime;
};
