#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "UsbHidHandler.h"
#include "SlimeUdpClient.h"
#include "esp_log.h"

int custom_vprintf(const char *fmt, va_list ap) {
    char buf[512];
    int len = vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
    if (len > 0) {
        buf[len] = '\0';
        Serial.print(buf);
        UdpLogger.print(buf);
    }
    return len;
}

UsbHidHandler usbHandler;
SlimeUdpClient slimeClient;

static bool wasConnected = false;
static uint32_t lastConnectionCheck = 0;

void setupWifi() {
    DEBUG_PRINTLN();
    DEBUG_PRINT("Connecting to WiFi: ");
    DEBUG_PRINTLN(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void setup() {
    Serial.begin(115200);
    // Wait for serial if you want, but better to proceed without it (since it's a dongle adapter)
    delay(2000); 

    // Redirect ESP-IDF logs to UdpLogger
    esp_log_set_vprintf(custom_vprintf);

    DEBUG_PRINTLN("Starting EsbImuReceiverToLan ESP32 Port...");

    setupWifi();

    // Initialize UDP Client
    slimeClient.begin(SLIMEVR_SERVER_IP, SLIMEVR_SERVER_PORT);
}

void loop() {
    // Check WiFi Connection
    if (millis() - lastConnectionCheck > 5000) {
        lastConnectionCheck = millis();
        bool isConnected = (WiFi.status() == WL_CONNECTED);
        
        if (isConnected && !wasConnected) {
            wasConnected = true;
            
            // Start UDP Logger
            UdpLogger.begin(SLIMEVR_SERVER_IP, 6970);

            DEBUG_PRINTLN("");
            DEBUG_PRINTLN("WiFi connected!");
            DEBUG_PRINT("IP address: ");
            DEBUG_PRINTLN(WiFi.localIP());
            
            // Initialize USB Host HID AFTER network is ready so we can log any crashes!
            usbHandler.begin(&slimeClient);
            
        } else if (!isConnected && wasConnected) {
            DEBUG_PRINTLN("WiFi lost connection. Reconnecting...");
            wasConnected = false;
            WiFi.reconnect();
        } else if (!isConnected && !wasConnected) {
            DEBUG_PRINT("Still connecting to WiFi... Status: ");
            DEBUG_PRINTLN(WiFi.status());
        }
    }

    if (wasConnected) {
        // Run SlimeVR UDP Loop (Heartbeat, Retries, etc.)
        slimeClient.loop();
    }

    // Process USB HID events
    usbHandler.loop();
}
