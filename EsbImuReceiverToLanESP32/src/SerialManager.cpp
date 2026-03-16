#include "SerialManager.h"
#include "WiFiConfig.h"
#include "WiFiManager.h"
#include <mbedtls/base64.h>
#include <USB.h>
#include <USBCDC.h>

static bool usbCDCEnabled = false;

void SerialManager::init() {
    // Hardware Serial is already started in main.cpp
    // Initialize Native USB CDC if it's not already handled by bootloader
    #if ARDUINO_USB_CDC_ON_BOOT == 0
    USBSerial.begin();
    usbCDCEnabled = true;
    #endif

    // Log identifiers to all ports on boot for discovery
    printStatus(Serial); 
}

void SerialManager::deinit() {
    #if ARDUINO_USB_CDC_ON_BOOT == 0
    if (usbCDCEnabled) {
        logToActivePorts("WiFi Connected. Switching Native USB port to USB Host Mode... Goodbye!");
        USBSerial.flush();
        delay(500); // Give time for message to send
        USBSerial.end();
        usbCDCEnabled = false;
    }
    #endif
}

void SerialManager::logToActivePorts(String message) {
    Serial.println(message);
    #if ARDUINO_USB_CDC_ON_BOOT == 0
    if (usbCDCEnabled) {
        USBSerial.println(message);
    }
    #endif
}

void SerialManager::loop() {
    // Check Hardware UART
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            processCommand(line, Serial);
        }
    }

    // Check Native USB CDC (if enabled and not in Host Mode yet)
    #if ARDUINO_USB_CDC_ON_BOOT == 0
    if (usbCDCEnabled && USBSerial.available()) {
        String line = USBSerial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            processCommand(line, USBSerial);
        }
    }
    #endif
}

void SerialManager::processCommand(String line, Stream& port) {
    if (line.startsWith("SET WIFI ")) {
        handleSetWifi(line.substring(9), port);
    } else if (line.startsWith("SET BWIFI ")) {
        handleSetBWifi(line.substring(10), port);
    } else if (line == "GET WIFI") {
        char ssid[33] = {0};
        WiFiConfig::load(ssid, NULL);
        port.print("WIFI: ");
        port.println(ssid);
    } else if (line == "GET INFO" || line == "STATUS") {
        printStatus(port);
    } else if (line == "REBOOT") {
        logToActivePorts("Rebooting...");
        delay(100);
        ESP.restart();
    } else if (line == "FRST" || line == "FACTORY RESET") {
        logToActivePorts("Factory resetting WiFi and rebooting...");
        WiFiConfig::clear();
        delay(500);
        ESP.restart();
    } else if (line == "GET WIFISCAN") {
        // Optional: Implement scan if needed, for now just acknowledge
        logToActivePorts("Scanning for WiFi networks...");
    } else if (line == "EXIT") {
        port.println("Exiting serial mode.");
    }
}

void SerialManager::handleSetWifi(String args, Stream& port) {
    // Syntax: SET WIFI "SSID" "PASS" OR SET WIFI SSID PASS
    int firstQuote = args.indexOf('"');
    int secondQuote = args.indexOf('"', firstQuote + 1);
    int thirdQuote = args.indexOf('"', secondQuote + 1);
    int fourthQuote = args.indexOf('"', thirdQuote + 1);

    String ssid, pass;

    if (firstQuote != -1 && secondQuote != -1) {
        ssid = args.substring(firstQuote + 1, secondQuote);
        if (thirdQuote != -1 && fourthQuote != -1) {
            pass = args.substring(thirdQuote + 1, fourthQuote);
        }
    } else {
        // Fallback for space-separated without quotes
        int space = args.indexOf(' ');
        ssid = (space == -1) ? args : args.substring(0, space);
        pass = (space == -1) ? "" : args.substring(space + 1);
    }

    if (ssid.length() > 0) {
        WiFiConfig::save(ssid.c_str(), pass.c_str());
        logToActivePorts("New wifi credentials set"); // Exactly what SlimeVR expects
        WiFiManager::reconnect(); // Force logs sequence (mac -> Searching...)
    } else {
        port.println("SET WIFI ERROR: Invalid syntax");
    }
}

void SerialManager::handleSetBWifi(String args, Stream& port) {
    int space = args.indexOf(' ');
    String b64ssid = (space == -1) ? args : args.substring(0, space);
    String b64pass = (space == -1) ? "" : args.substring(space + 1);

    String ssid = decodeBase64(b64ssid);
    String pass = (b64pass.length() > 0) ? decodeBase64(b64pass) : "";

    if (ssid.length() > 0) {
        WiFiConfig::save(ssid.c_str(), pass.c_str());
        logToActivePorts("New wifi credentials set"); // Consistency
        WiFiManager::reconnect(); // Force logs sequence
    } else {
        port.println("SET BWIFI ERROR: Invalid Base64");
    }
}

void SerialManager::printStatus(Stream& port) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[20];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // Format expected by ProvisioningHandler.java: "mac: XX:XX:XX:XX:XX:XX, "
    String macLog = "mac: ";
    macLog += macStr;
    macLog += ", ";
    logToActivePorts(macLog);

    String statusLog = "STATUS: ";
    statusLog += (WiFiManager::isConnected() ? "CONNECTED" : "DISCONNECTED");
    statusLog += " | SSID: ";
    char ssid[33] = {0};
    WiFiConfig::load(ssid, NULL);
    statusLog += ssid;
    statusLog += " | IP: ";
    statusLog += WiFi.localIP().toString();
    logToActivePorts(statusLog);
}

String SerialManager::decodeBase64(String input) {
    size_t out_len = 0;
    mbedtls_base64_decode(NULL, 0, &out_len, (const unsigned char*)input.c_str(), input.length());
    if (out_len == 0) return "";

    unsigned char *buffer = (unsigned char *)malloc(out_len + 1);
    mbedtls_base64_decode(buffer, out_len, &out_len, (const unsigned char*)input.c_str(), input.length());
    buffer[out_len] = '\0';
    
    String result = (char*)buffer;
    free(buffer);
    return result;
}
