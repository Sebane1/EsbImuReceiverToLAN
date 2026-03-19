#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"
#include "UsbHidHandler.h"
#include "SlimeUdpClient.h"
#include "WiFiManager.h"
#include "SerialManager.h"

UsbHidHandler usbHandler;
SlimeUdpClient slimeClient;

static bool wasConnected = false;
static uint32_t lastLoopCheck = 0;

void setup() {
    Serial.begin(115200);
    delay(2000); 

    DEBUG_PRINTLN("Starting EsbImuReceiverToLan ESP32 Port...");

    // Initialize Serial Manager FIRST to enable Native USB CDC logs
    SerialManager::init();
    
    // Initialize WiFi (Connects and issues discovery logs)
    WiFiManager::init();

    // Initialize UDP Client with Server Discovery
    // Passing no IP enables broadcast discovery mode
    slimeClient.begin("", SLIMEVR_SERVER_PORT);
}

void loop() {
    static bool usbInitialized = false;

    // Process WiFi Portal (if active)
    WiFiManager::loop();
    
    // Process Serial Commands (USB Provisioning)
    SerialManager::loop();

    // Check WiFi Connection for Data Flow
    if (millis() - lastLoopCheck > 500) {
        lastLoopCheck = millis();
        static uint32_t wifiConnectTime = 0;
        bool isConnected = WiFiManager::isConnected();

        if (isConnected && !wasConnected) {
            wasConnected = true;
            wifiConnectTime = millis();
            slimeClient.onWiFiConnect();
            
            DEBUG_PRINTLN("WiFi connected! Waiting for SlimeVR handshake before starting USB Host...");
            DEBUG_PRINT("IP address: ");
            DEBUG_PRINTLN(WiFi.localIP());
        } else if (!isConnected && wasConnected) {
            DEBUG_PRINTLN("WiFi connection lost.");
            wasConnected = false;
            slimeClient.onWiFiDisconnect();
            // We don't reset usbInitialized because once in Host mode, we stay there 
            // until reboot, but we could reset it if we wanted to support hot-swap.
            // For now, let's keep it simple.
        }

        // Deferred USB Host initialization
        if (wasConnected && !usbInitialized) {
            bool handshaked = slimeClient.isHandshakeSuccessful();
            bool timedOut = (millis() - wifiConnectTime > 10000);

            if (handshaked || timedOut) {
                if (handshaked) {
                    DEBUG_PRINTLN("Handshake successful! Switching Native USB to Host Mode.");
                } else {
                    DEBUG_PRINTLN("Handshake timeout (10s). Switching Native USB to Host Mode anyway.");
                }
                
                // Shut down Native USB Serial (CDC) before starting USB Host
                SerialManager::deinit();
                
                usbHandler.begin(&slimeClient);
                usbInitialized = true;
            }
        }
    }

    if (wasConnected) {
        slimeClient.loop();
    }

    // Process USB HID events
    usbHandler.loop();

    // HID Watchdog: Reboot if data stalls for too long
    if (usbInitialized && (millis() - usbHandler.getLastReportTime() > HID_WATCHDOG_TIMEOUT_MS)) {
        // Note: No Serial logging here because SerialManager is deinitialized in Host mode
        ESP.restart();
    }
}
